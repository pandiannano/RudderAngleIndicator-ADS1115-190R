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

#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include "Config.h"
#include "Filtering.h"
#include "Calibration.h"
#include "HmiLink.h"

static Adafruit_ADS1115 ads;
static Calibration calibration;
static EmaFilter emaFilter(EMA_ALPHA);
static SlewLimiter slewLimiter(SLEW_MAX_DEG_PER_SEC);

static float g_lastFilteredSenderVolts = 0.0f;
static float g_lastDisplayedAngle = NAN;
static bool  g_lastFaultState = false;

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
      else if (line == "STATUS") {
        const CalPoints &p = calibration.points();
        DBG("[STATUS] senderV=%.3f angle=%.2f cal(min=%.3f,center=%.3f,max=%.3f)\n",
            g_lastFilteredSenderVolts, g_lastDisplayedAngle, p.vAtMin, p.vAtCenter, p.vAtMax);
      } else if (line.length() > 0) {
        DBG("[CMD] unknown: %s (try MIN/CENTER/MAX/SAVE/RESET/STATUS)\n", line.c_str());
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
  hmiBegin();

  DBG("[INIT] ready. Serial console: MIN / CENTER / MAX / SAVE / RESET / STATUS\n");
}

void loop() {
  hmiPoll();
  pollDebugSerial();
  sampleFilterAndUpdate();
}
