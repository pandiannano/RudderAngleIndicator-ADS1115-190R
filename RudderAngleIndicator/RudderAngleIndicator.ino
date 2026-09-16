// RudderAngleIndicator.ino
//
// ESP32-C3 + ADS1115 (I2C) + TJC/Nextion-protocol HMI (UART)
// 0-190 ohm sender wired as a 0-12V bias/voltage divider -> AIN0.
// AIN1/AIN2/AIN3 tied to GND as specified by the hardware design.
//
// Noise-reduction pipeline (see Filtering.h / Config.h for the tunables):
//   1. ADS1115 PGA gain chosen to match the divided-down signal swing, and
//      a moderate data rate to benefit from the chip's internal sigma-delta
//      (digital) filtering                      -> "ADC / internal filter"
//   2. A hardware RC low-pass at the ADS1115 input (see Config.h comment)   -> "noise filter from ADC"
//   3. A batch of raw samples per update, trimmed-mean averaged to reject
//      spikes                                    -> "mean value averaging"
//   4. An exponential moving average (software IIR low-pass) across update
//      cycles                                    -> "software filter"
//   5. A slew-rate limiter + display deadband to remove any remaining
//      jitter without adding perceptible lag.
//
// Calibration (3-point: full-port / midships / full-starboard) is stored in
// NVS flash and can be set either from on-screen HMI buttons or the USB
// serial console - see README.md.
//
// AIN1 addition: a second sender, a floating (float-arm) level sender that
// only ever rests at 10 discrete resistance steps, is now also read on
// AIN1 and decoded/filtered independently (see FloatLevel.h). This did not
// require changing the AIN0 angle pipeline above, other than re-asserting
// the ADS1115's PGA gain before each AIN0 batch, since the gain register is
// shared by all four ADS1115 inputs and the AIN1 code now also changes it.

#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include "Config.h"
#include "Filtering.h"
#include "Calibration.h"
#include "FloatLevel.h"
#include "HmiLink.h"

static Adafruit_ADS1115 ads;
static Calibration calibration;
static EmaFilter emaFilter(EMA_ALPHA);
static SlewLimiter slewLimiter(SLEW_MAX_DEG_PER_SEC);

static float g_lastFilteredSenderVolts = 0.0f;
static float g_lastDisplayedAngle = NAN;
static bool  g_lastFaultState = false;

static FloatLevelSensor floatLevel;
static float g_lastFloatSenderVolts = 0.0f;
static int   g_lastDisplayedLevel = -1;
static bool  g_lastLevelFaultState = false;
static int   g_levelCalSlot = 0; // slot selected for HMI NEXT/CAPTURE buttons

// ---------------------------------------------------------------------------
// Debug helper
// ---------------------------------------------------------------------------
#if ENABLE_SERIAL_DEBUG
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
#endif

// ---------------------------------------------------------------------------
// Calibration actions shared by the HMI buttons and the USB serial console
// ---------------------------------------------------------------------------
static void doCalSetMin() {
  calibration.setMinFromVoltage(g_lastFilteredSenderVolts);
  hmiSendText(HMI_COMP_FAULT_TXT, "MIN SET");
  DBG("[CAL] MIN set to %.3f V\n", g_lastFilteredSenderVolts);
}

static void doCalSetCenter() {
  calibration.setCenterFromVoltage(g_lastFilteredSenderVolts);
  hmiSendText(HMI_COMP_FAULT_TXT, "CENTER SET");
  DBG("[CAL] CENTER set to %.3f V\n", g_lastFilteredSenderVolts);
}

static void doCalSetMax() {
  calibration.setMaxFromVoltage(g_lastFilteredSenderVolts);
  hmiSendText(HMI_COMP_FAULT_TXT, "MAX SET");
  DBG("[CAL] MAX set to %.3f V\n", g_lastFilteredSenderVolts);
}

static void doCalSave() {
  bool ok = calibration.save();
  hmiSendText(HMI_COMP_FAULT_TXT, ok ? "CAL SAVED" : "SAVE FAILED");
  DBG("[CAL] save %s\n", ok ? "OK" : "FAILED");
}

static void doCalReset() {
  bool ok = calibration.resetToDefaults();
  hmiSendText(HMI_COMP_FAULT_TXT, ok ? "CAL RESET" : "RESET FAILED");
  DBG("[CAL] reset %s\n", ok ? "OK" : "FAILED");
}

// ---------------------------------------------------------------------------
// AIN1 float-level calibration actions (new; mirrors the pattern above)
// ---------------------------------------------------------------------------
static void doLvlNext() {
  g_levelCalSlot = (g_levelCalSlot + 1) % FLOAT_LEVEL_COUNT;
  char msg[16];
  snprintf(msg, sizeof(msg), "SLOT %d", g_levelCalSlot);
  hmiSendText(HMI_COMP_LEVEL_FAULT_TXT, msg);
  DBG("[LVL] slot selected: %d\n", g_levelCalSlot);
}

static void doLvlCapture() {
  floatLevel.captureLevel(g_levelCalSlot, g_lastFloatSenderVolts);
  char msg[24];
  snprintf(msg, sizeof(msg), "SLOT %d SET", g_levelCalSlot);
  hmiSendText(HMI_COMP_LEVEL_FAULT_TXT, msg);
  DBG("[LVL] slot %d set to %.3f V\n", g_levelCalSlot, g_lastFloatSenderVolts);
}

static void doLvlSave() {
  bool ok = floatLevel.save();
  hmiSendText(HMI_COMP_LEVEL_FAULT_TXT, ok ? "LVL SAVED" : "SAVE FAILED");
  DBG("[LVL] save %s\n", ok ? "OK" : "FAILED");
}

static void doLvlReset() {
  bool ok = floatLevel.resetToDefaults();
  hmiSendText(HMI_COMP_LEVEL_FAULT_TXT, ok ? "LVL RESET" : "RESET FAILED");
  DBG("[LVL] reset %s\n", ok ? "OK" : "FAILED");
}

// Called by HmiLink.cpp whenever a complete touch-event frame is received.
void onHmiTouchEvent(uint8_t pageId, uint8_t componentId, uint8_t eventType) {
  const uint8_t RELEASE = 0x00; // act on release, like a normal button click
  if (pageId != HMI_PAGE_MAIN || eventType != RELEASE) return;

  switch (componentId) {
    case HMI_BTN_CAL_MIN_ID:    doCalSetMin();    break;
    case HMI_BTN_CAL_CENTER_ID: doCalSetCenter(); break;
    case HMI_BTN_CAL_MAX_ID:    doCalSetMax();    break;
    case HMI_BTN_CAL_SAVE_ID:   doCalSave();      break;
    case HMI_BTN_CAL_RESET_ID:  doCalReset();     break;
    case HMI_BTN_LVL_NEXT_ID:    doLvlNext();    break;
    case HMI_BTN_LVL_CAPTURE_ID: doLvlCapture(); break;
    case HMI_BTN_LVL_SAVE_ID:    doLvlSave();    break;
    case HMI_BTN_LVL_RESET_ID:   doLvlReset();   break;
    default: break;
  }
}

// Lightweight bench-calibration console over USB serial.
static void pollDebugSerial() {
#if ENABLE_SERIAL_DEBUG
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      line.toUpperCase();
      if (line == "MIN") doCalSetMin();
      else if (line == "CENTER") doCalSetCenter();
      else if (line == "MAX") doCalSetMax();
      else if (line == "SAVE") doCalSave();
      else if (line == "RESET") doCalReset();
      else if (line == "LVLSAVE") doLvlSave();
      else if (line == "LVLRESET") doLvlReset();
      else if (line.startsWith("LVL") && line.length() == 4 && line[3] >= '0' && line[3] <= '9') {
        int idx = line[3] - '0';
        floatLevel.captureLevel(idx, g_lastFloatSenderVolts);
        DBG("[LVL] slot %d set to %.3f V (LVLSAVE to persist)\n", idx, g_lastFloatSenderVolts);
      }
      else if (line == "STATUS") {
        const CalPoints &p = calibration.points();
        DBG("[STATUS] senderV=%.3f angle=%.2f cal(min=%.3f,center=%.3f,max=%.3f)\n",
            g_lastFilteredSenderVolts, g_lastDisplayedAngle, p.vAtMin, p.vAtCenter, p.vAtMax);
        DBG("[STATUS] levelSenderV=%.3f level=%d/%d table=[",
            g_lastFloatSenderVolts, g_lastDisplayedLevel, FLOAT_LEVEL_COUNT - 1);
        for (int i = 0; i < FLOAT_LEVEL_COUNT; i++) {
          DBG("%.2f%s", floatLevel.levelVoltage(i), (i < FLOAT_LEVEL_COUNT - 1) ? "," : "]\n");
        }
      } else if (line.length() > 0) {
        DBG("[CMD] unknown: %s (try MIN/CENTER/MAX/SAVE/RESET/LVL0../LVL9/LVLSAVE/LVLRESET/STATUS)\n", line.c_str());
      }
      line = "";
    } else {
      line += c;
    }
  }
#endif
}

// ---------------------------------------------------------------------------
// Sensor sampling + filtering + HMI update, one full cycle
// ---------------------------------------------------------------------------
static void sampleFilterAndUpdate() {
  // The ADS1115's PGA gain is a single register shared by all 4 inputs.
  // The AIN1 float-level read (below) may have changed it, so it must be
  // re-asserted here before every AIN0 batch. This is the only change made
  // to the previously-working angle pipeline.
  ads.setGain(ADS1115_GAIN);

  float samples[RAW_SAMPLES_PER_BATCH];

  for (int i = 0; i < RAW_SAMPLES_PER_BATCH; i++) {
    int16_t raw = ads.readADC_SingleEnded(ADS1115_CHANNEL);
    samples[i] = ads.computeVolts(raw);

    // Keep the HMI and USB console responsive even though each ADS1115
    // conversion blocks for ~1/SPS seconds.
    hmiPoll();
    pollDebugSerial();
  }

  float adcVolts = trimmedMean(samples, RAW_SAMPLES_PER_BATCH, TRIM_COUNT);
  float senderVolts = adcVolts * DIVIDER_RATIO;

  bool fault = (senderVolts < SENDER_V_FAULT_LOW) || (senderVolts > SENDER_V_FAULT_HIGH);

  float filteredSenderVolts = emaFilter.update(senderVolts);
  g_lastFilteredSenderVolts = filteredSenderVolts;

  float targetAngle = calibration.voltageToAngle(filteredSenderVolts);
  float smoothedAngle = slewLimiter.update(targetAngle, millis());

  bool angleChangedEnough =
      isnan(g_lastDisplayedAngle) ||
      fabsf(smoothedAngle - g_lastDisplayedAngle) >= DISPLAY_DEADBAND_DEG;

  if (fault != g_lastFaultState) {
    hmiSendText(HMI_COMP_FAULT_TXT, fault ? "SENSOR FAULT" : "");
    g_lastFaultState = fault;
  }

  if (!fault && angleChangedEnough) {
    g_lastDisplayedAngle = smoothedAngle;

    // n0.val takes an integer; send angle*10 so the HMI can show one
    // decimal place (e.g. divide by 10 in a text-conversion, or bind a
    // Gauge/Slider component directly to the tenths-of-a-degree value).
    hmiSendNumber(HMI_COMP_ANGLE_NUM, lround(smoothedAngle * 10.0f));

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", smoothedAngle);
    hmiSendText(HMI_COMP_ANGLE_TXT, String(buf));

    DBG("[ANGLE] senderV=%.3f filtV=%.3f angle=%.2f\n",
        senderVolts, filteredSenderVolts, smoothedAngle);
  } else if (fault) {
    DBG("[FAULT] senderV=%.3f out of plausible range\n", senderVolts);
  }
}

// ---------------------------------------------------------------------------
// AIN1 float-level sampling + filtering + HMI update, one full cycle (new)
// ---------------------------------------------------------------------------
static void sampleFloatLevelAndUpdate() {
  // Re-select the gain for this channel's divider swing; see the comment in
  // sampleFilterAndUpdate() above about the shared PGA gain register.
  ads.setGain(ADS1115_GAIN_AIN1);

  float samples[LEVEL_RAW_SAMPLES_PER_BATCH];

  for (int i = 0; i < LEVEL_RAW_SAMPLES_PER_BATCH; i++) {
    int16_t raw = ads.readADC_SingleEnded(AIN1_CHANNEL);
    samples[i] = ads.computeVolts(raw);

    hmiPoll();
    pollDebugSerial();
  }

  float adcVolts = trimmedMean(samples, LEVEL_RAW_SAMPLES_PER_BATCH, LEVEL_TRIM_COUNT);
  float senderVolts = adcVolts * DIVIDER_RATIO_AIN1;
  g_lastFloatSenderVolts = senderVolts;

  bool fault = (senderVolts < LEVEL_V_FAULT_LOW) || (senderVolts > LEVEL_V_FAULT_HIGH);

  if (fault != g_lastLevelFaultState) {
    hmiSendText(HMI_COMP_LEVEL_FAULT_TXT, fault ? "LEVEL FAULT" : "");
    g_lastLevelFaultState = fault;
  }

  if (fault) {
    DBG("[LEVEL FAULT] senderV=%.3f out of plausible range\n", senderVolts);
    return;
  }

  // No EMA here on purpose: the sensor only ever sits at one of
  // FLOAT_LEVEL_COUNT discrete voltages, so nearest-match + a debounce
  // count (inside floatLevel.update) rejects noise without blurring
  // between two adjacent, legitimately different levels.
  int level = floatLevel.update(senderVolts);

  if (level != g_lastDisplayedLevel) {
    g_lastDisplayedLevel = level;
    int percent = lround(level * 100.0f / (FLOAT_LEVEL_COUNT - 1));

    hmiSendNumber(HMI_COMP_LEVEL_NUM, percent);

    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", percent);
    hmiSendText(HMI_COMP_LEVEL_TXT, String(buf));

    DBG("[LEVEL] senderV=%.3f level=%d/%d (%d%%)\n",
        senderVolts, level, FLOAT_LEVEL_COUNT - 1, percent);
  }
}

// ---------------------------------------------------------------------------
void setup() {
#if ENABLE_SERIAL_DEBUG
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(200);
  DBG("\nRudder Angle Indicator - starting up\n");
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  if (!ads.begin(ADS1115_I2C_ADDR, &Wire)) {
    DBG("[INIT] ADS1115 not found at 0x%02X - check wiring!\n", ADS1115_I2C_ADDR);
  }
  ads.setGain(ADS1115_GAIN);
  ads.setDataRate(ADS1115_DATA_RATE);

  calibration.begin();
  floatLevel.begin();
  hmiBegin();

  DBG("[INIT] ready. Serial console: MIN / CENTER / MAX / SAVE / RESET / "
      "LVL0../LVL9 / LVLSAVE / LVLRESET / STATUS\n");
}

void loop() {
  hmiPoll();
  pollDebugSerial();
  sampleFilterAndUpdate();
  sampleFloatLevelAndUpdate();
}
