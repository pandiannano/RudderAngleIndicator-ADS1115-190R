// RudderAngleIndicator.ino
//
// ESP32-C3 + ADS1115 (I2C) + TJC/Nextion-protocol HMI (UART)
// AIN0 = angle sensor (0-360 deg, 0-3.3V output); AIN1 = floating level
// sender; AIN2 = tied to GND; AIN3 = tied to AVDD/3.3V (ADC self-check
// references, see the AIN2/AIN3 addition note below).
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
// only ever rests at a number of discrete resistance steps (see
// FLOAT_LEVEL_COUNT in Config.h), is now also read on AIN1 and
// decoded/filtered independently (see FloatLevel.h). This did not require
// changing the AIN0 angle pipeline above, other than re-asserting the
// ADS1115's PGA gain before each AIN0 batch, since the gain register is
// shared by all four ADS1115 inputs and the AIN1 code now also changes it.
//
// AIN2/AIN3 addition: AIN2 is wired to GND and AIN3 to AVDD (3.3V) as known
// reference levels. checkAdcReferenceRails() periodically reads both back
// and flags a fault if either is out of tolerance — a simple self-test for
// a failed ADS1115, bad I2C link, or sagging 3.3V rail. Same shared-gain
// pattern as AIN1; does not touch the AIN0/AIN1 code either.
//
// Open-wire addition: AIN0 (angle sensor) and AIN1 (float sender) fail
// differently when a wire is cut. AIN1 is a passive divider, so an open
// sender wire pulls its tap up to the supply rail — caught by comparing the
// raw reading to the measured AVDD (OPEN_CIRCUIT_MARGIN_V in Config.h).
// AIN0 is an actively-driven sensor, so an open wire floats the input
// instead of pinning it anywhere — caught instead by an abnormally large
// spread within one batch of raw samples (ANGLE_NOISE_FAULT_V, using the
// new batchSpread() helper in Filtering.h/.cpp). Both are additive checks
// layered on top of the existing range-based fault checks; see the
// Config.h comments by those two constants for the full reasoning.
//
// Unified fault indicator addition: the AIN0 and AIN1 faults above now
// share ONE blinking HMI text field (updateSharedFaultDisplay(), t5)
// instead of writing to their own separate fault texts, and drive the X2's
// onboard buzzer via TJC's documented `beep <ms>` instruction in a
// repeating on/off pattern while any fault is active (updateFaultBuzzer())
// — see the Config.h comments by HMI_COMP_FAULT_SHARED_TXT and
// FAULT_BUZZER_ON_MS/OFF_MS.
//
// Fault latch fix: AIN0's open-wire check (a noise-spread test) only fires
// while the floating input is actively noisy, and a floating input often
// quiets down on its own a second or two after the wire is cut — which was
// making the fault (and its buzzer) clear itself after one beep even
// though the sensor was still disconnected. Both AIN0 and AIN1's fault
// flags are now latched: an active fault only clears after
// FAULT_CLEAR_CONFIRM_BATCHES consecutive clean batches, not just one, so
// the underlying instant test only needs to catch the problem occasionally
// to keep the fault (and beep) going for as long as it's real.
//
// Root-cause fix for AIN0: on real hardware the floating input didn't just
// occasionally go quiet between noise bursts, it settled permanently — so
// even the latch above eventually ran out its 5-batch streak and cleared
// the fault for good. Root cause: ANGLE_NOISE_FAULT_V can only ever detect
// a floating input WHILE it's actively noisy, and this one stopped being
// noisy. Fixed by adding calRangeFault (see CAL_RANGE_FAULT_MARGIN_FRAC in
// Config.h and Calibration::isWithinCalibratedRange()): it checks the
// reading against your own captured calibration span instead of a fixed
// voltage, so it keeps working even once the floating voltage goes fully
// quiet, as long as it settles outside your narrow calibrated operating
// window — which, for a sensor whose full mechanical range is much wider
// than what you actually use (0-360 degrees vs. a 135-225 degree operating
// window here), is the expected case.

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

static unsigned long g_lastRefCheckMs = 0;
static float g_lastGndVolts  = NAN;
static float g_lastAvddVolts = NAN;
static bool  g_refCheckFault = false;
static void  checkAdcReferenceRails(bool force = false); // defined below; used by pollDebugSerial

static bool g_lastAngleOpenFault = false; // set by sampleFilterAndUpdate(), read by updateSharedFaultDisplay()
static bool g_lastFloatOpenFault = false; // set by sampleFloatLevelAndUpdate(), read by updateSharedFaultDisplay()

// Fault latches (fast-trip, slow-reset — see FAULT_CLEAR_CONFIRM_BATCHES in
// Config.h): each starts "at" its own confirm count so a clean reading at
// boot doesn't report a phantom fault.
static int  g_angleFaultClearStreak = FAULT_CLEAR_CONFIRM_BATCHES;
static int  g_levelFaultClearStreak = FAULT_CLEAR_CONFIRM_BATCHES;
static bool g_angleFaultWasOpen = false; // which reason last (re)triggered the latch
static bool g_levelFaultWasOpen = false;

// ---------------------------------------------------------------------------
// Debug helper
// ---------------------------------------------------------------------------
#if ENABLE_SERIAL_DEBUG
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
#endif

static bool isAllDigits(const String &s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (s[i] < '0' || s[i] > '9') return false;
  }
  return true;
}

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
      else if (line == "ADCCHK") checkAdcReferenceRails(true);
      else if (line.startsWith("LVL") && line.length() > 3 && isAllDigits(line.substring(3))) {
        int idx = line.substring(3).toInt();
        if (idx >= 0 && idx < FLOAT_LEVEL_COUNT) {
          floatLevel.captureLevel(idx, g_lastFloatSenderVolts);
          DBG("[LVL] slot %d set to %.3f V (LVLSAVE to persist)\n", idx, g_lastFloatSenderVolts);
        } else {
          DBG("[CMD] LVL index out of range 0..%d\n", FLOAT_LEVEL_COUNT - 1);
        }
      }
      else if (line == "STATUS") {
        const CalPoints &p = calibration.points();
        DBG("[STATUS] senderV=%.3f angle=%.2f fault=%s cal(min=%.3f,center=%.3f,max=%.3f)\n",
            g_lastFilteredSenderVolts, g_lastDisplayedAngle,
            g_lastFaultState ? "YES" : "no", p.vAtMin, p.vAtCenter, p.vAtMax);
        DBG("[STATUS] levelSenderV=%.3f level=%d/%d fault=%s table=[",
            g_lastFloatSenderVolts, g_lastDisplayedLevel, FLOAT_LEVEL_COUNT - 1,
            g_lastLevelFaultState ? "YES" : "no");
        for (int i = 0; i < FLOAT_LEVEL_COUNT; i++) {
          DBG("%.2f%s", floatLevel.levelVoltage(i), (i < FLOAT_LEVEL_COUNT - 1) ? "," : "]\n");
        }
        DBG("[STATUS] adcRef %s gnd=%.3fV (expect %.2f+-%.2f) avdd=%.3fV (expect %.2f+-%.2f)\n",
            g_refCheckFault ? "FAIL" : "OK", g_lastGndVolts,
            REFCHK_GND_EXPECTED_V, REFCHK_GND_TOLERANCE_V, g_lastAvddVolts,
            REFCHK_AVDD_EXPECTED_V, REFCHK_AVDD_TOLERANCE_V);
      } else if (line.length() > 0) {
        DBG("[CMD] unknown: %s (try MIN/CENTER/MAX/SAVE/RESET/LVL0../LVL12/LVLSAVE/LVLRESET/ADCCHK/STATUS)\n", line.c_str());
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

  // calRangeFault checks against YOUR calibrated span (135-225 degrees'
  // worth of voltage), not the sensor's full 0-360 degree range — see the
  // CAL_RANGE_FAULT_MARGIN_FRAC comment in Config.h. This is what makes an
  // open wire that settles to a quiet-but-wrong voltage a PERSISTENT fault
  // instead of a one-shot blip, since that settled voltage is very unlikely
  // to land inside your narrow operating window.
  bool calRangeFault = !calibration.isWithinCalibratedRange(senderVolts, CAL_RANGE_FAULT_MARGIN_FRAC);
  bool rangeFault = calRangeFault ||
      (senderVolts < SENDER_V_FAULT_LOW) || (senderVolts > SENDER_V_FAULT_HIGH);

  // A cut angle-sensor wire floats the ADC input rather than pinning it to
  // a rail (see the Config.h note by ANGLE_NOISE_FAULT_V), so it shows up
  // as abnormally large sample-to-sample spread within this batch instead
  // of an out-of-range voltage. This only catches the initial transient
  // (see calRangeFault above for the persistent case).
  bool openFault = batchSpread(samples, RAW_SAMPLES_PER_BATCH) > ANGLE_NOISE_FAULT_V;

  bool instantFault = rangeFault || openFault;

  // Latch: a floating input often bursts with noise right when it's cut,
  // then settles toward a quiet-but-still-wrong voltage, which would
  // otherwise make openFault flicker off after one clean-looking batch
  // even though the wire is still disconnected. Require several
  // consecutive clean batches before actually clearing the fault (and, in
  // turn, the buzzer/blink it drives) — see FAULT_CLEAR_CONFIRM_BATCHES.
  if (instantFault) {
    g_angleFaultClearStreak = 0;
    g_angleFaultWasOpen = openFault; // record the reason while it's actually happening
  } else if (g_angleFaultClearStreak < FAULT_CLEAR_CONFIRM_BATCHES) {
    g_angleFaultClearStreak++;
  }
  bool fault = g_angleFaultClearStreak < FAULT_CLEAR_CONFIRM_BATCHES;

  float filteredSenderVolts = emaFilter.update(senderVolts);
  g_lastFilteredSenderVolts = filteredSenderVolts;

  float targetAngle = calibration.voltageToAngle(filteredSenderVolts);
  float smoothedAngle = slewLimiter.update(targetAngle, millis());

  bool angleChangedEnough =
      isnan(g_lastDisplayedAngle) ||
      fabsf(smoothedAngle - g_lastDisplayedAngle) >= DISPLAY_DEADBAND_DEG;

  // Fault display now goes through the single shared fault field (see
  // updateSharedFaultDisplay(), called once per loop() after both sampling
  // functions) instead of writing HMI_COMP_FAULT_TXT directly here — that
  // field is used only for calibration-confirmation messages now. Uses the
  // latched reason (g_angleFaultWasOpen), not the instantaneous openFault,
  // so the message stays correct throughout the latch period too.
  g_lastAngleOpenFault = g_angleFaultWasOpen;
  g_lastFaultState = fault;

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
  } else if (g_angleFaultWasOpen) {
    DBG("[FAULT] angle wire open/floating (batch spread > %.2fV, latched %d/%d)\n",
        (float)ANGLE_NOISE_FAULT_V, g_angleFaultClearStreak, FAULT_CLEAR_CONFIRM_BATCHES);
  } else if (fault) {
    DBG("[FAULT] senderV=%.3f out of plausible range (cal=%d abs=%d)\n",
        senderVolts, calRangeFault, rangeFault && !calRangeFault);
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

  bool rangeFault = (senderVolts < LEVEL_V_FAULT_LOW) || (senderVolts > LEVEL_V_FAULT_HIGH);

  // If the float sender's own wire is cut, R2 drops out of the divider and
  // the tap is pulled up to essentially the supply rail (see Config.h note
  // by OPEN_CIRCUIT_MARGIN_V). Compared against the raw ADC reading (not
  // senderVolts, which is scaled by DIVIDER_RATIO_AIN1) and the actually
  // measured AVDD, so it doesn't depend on that scale factor. Skipped until
  // the AIN2/AIN3 self-check has produced a first reading.
  bool openFault = !isnan(g_lastAvddVolts) && (adcVolts >= (g_lastAvddVolts - OPEN_CIRCUIT_MARGIN_V));

  bool instantFault = rangeFault || openFault;

  // Same latch as the angle channel (see FAULT_CLEAR_CONFIRM_BATCHES in
  // Config.h) — this pinned-to-rail check doesn't strictly need it since
  // the voltage stays put once the wire is cut, but it's applied here too
  // for consistency and as cheap insurance against any single-batch blip.
  if (instantFault) {
    g_levelFaultClearStreak = 0;
    g_levelFaultWasOpen = openFault;
  } else if (g_levelFaultClearStreak < FAULT_CLEAR_CONFIRM_BATCHES) {
    g_levelFaultClearStreak++;
  }
  bool fault = g_levelFaultClearStreak < FAULT_CLEAR_CONFIRM_BATCHES;

  // Fault display now goes through the single shared fault field (see
  // updateSharedFaultDisplay()) instead of writing HMI_COMP_LEVEL_FAULT_TXT
  // directly here — that field is used only for calibration-confirmation
  // messages now.
  g_lastFloatOpenFault = g_levelFaultWasOpen;
  g_lastLevelFaultState = fault;

  if (fault) {
    if (g_levelFaultWasOpen) {
      DBG("[LEVEL FAULT] adcV=%.3f pinned near AVDD=%.3f (wire cut?, latched %d/%d)\n",
          adcVolts, g_lastAvddVolts, g_levelFaultClearStreak, FAULT_CLEAR_CONFIRM_BATCHES);
    } else {
      DBG("[LEVEL FAULT] senderV=%.3f out of plausible range\n", senderVolts);
    }
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
// AIN2/AIN3 ADC reference self-check (new). AIN2 is tied to GND and AIN3 to
// AVDD (3.3V) as known-good reference levels; reading them back and
// comparing to the expected values catches a failed/miswired ADS1115, a
// flaky I2C link, or a sagging 3.3V rail. This is a slow/static check, so
// it self-paces on REFCHK_INTERVAL_MS via millis() instead of running every
// loop() iteration like the AIN0/AIN1 channels.
// ---------------------------------------------------------------------------
static void checkAdcReferenceRails(bool force) {
  unsigned long now = millis();
  if (!force && (now - g_lastRefCheckMs) < REFCHK_INTERVAL_MS) return;
  g_lastRefCheckMs = now;

  ads.setGain(ADS1115_GAIN_REFCHK);

  float gndSamples[REFCHK_SAMPLES_PER_BATCH];
  for (int i = 0; i < REFCHK_SAMPLES_PER_BATCH; i++) {
    int16_t raw = ads.readADC_SingleEnded(AIN2_CHANNEL);
    gndSamples[i] = ads.computeVolts(raw);
    hmiPoll();
    pollDebugSerial();
  }
  g_lastGndVolts = trimmedMean(gndSamples, REFCHK_SAMPLES_PER_BATCH, REFCHK_TRIM_COUNT);

  float avddSamples[REFCHK_SAMPLES_PER_BATCH];
  for (int i = 0; i < REFCHK_SAMPLES_PER_BATCH; i++) {
    int16_t raw = ads.readADC_SingleEnded(AIN3_CHANNEL);
    avddSamples[i] = ads.computeVolts(raw);
    hmiPoll();
    pollDebugSerial();
  }
  g_lastAvddVolts = trimmedMean(avddSamples, REFCHK_SAMPLES_PER_BATCH, REFCHK_TRIM_COUNT);

  bool gndOk  = fabsf(g_lastGndVolts  - REFCHK_GND_EXPECTED_V)  <= REFCHK_GND_TOLERANCE_V;
  bool avddOk = fabsf(g_lastAvddVolts - REFCHK_AVDD_EXPECTED_V) <= REFCHK_AVDD_TOLERANCE_V;
  bool fault = !(gndOk && avddOk);

  if (fault != g_refCheckFault || force) {
    g_refCheckFault = fault;
    if (fault) {
      char msg[32];
      if (!gndOk && !avddOk) snprintf(msg, sizeof(msg), "ADC REF FAIL GND+VDD");
      else if (!gndOk)       snprintf(msg, sizeof(msg), "ADC REF FAIL GND=%.2fV", g_lastGndVolts);
      else                   snprintf(msg, sizeof(msg), "ADC REF FAIL VDD=%.2fV", g_lastAvddVolts);
      hmiSendText(HMI_COMP_REFCHK_TXT, msg);
    } else {
      hmiSendText(HMI_COMP_REFCHK_TXT, "");
    }
  }

  DBG("[ADCCHK] %s gnd=%.3fV avdd=%.3fV\n", fault ? "FAIL" : "OK",
      g_lastGndVolts, g_lastAvddVolts);
}

// ---------------------------------------------------------------------------
// Unified, blinking fault indicator (new). ONE shared HMI text field for
// both the AIN0 angle fault and the AIN1 float fault, replacing their
// previous separate fault-text writes. Priority: angle fault (steering)
// over float fault when both are active. Blinks by toggling the shared
// field's visibility with the standard `vis` instruction every
// FAULT_BLINK_INTERVAL_MS while any fault is active.
// ---------------------------------------------------------------------------
static bool   g_sharedFaultActive = false;
static String g_sharedFaultMsg = "";
static bool   g_faultBlinkVisible = true;
static unsigned long g_lastBlinkToggleMs = 0;

static void setSharedFaultVisible(bool visible) {
  hmiSendRaw(String("vis ") + HMI_COMP_FAULT_SHARED_TXT + "," + (visible ? "1" : "0"));
}

static void updateSharedFaultDisplay() {
  bool anyFault = g_lastFaultState || g_lastLevelFaultState;
  unsigned long now = millis();

  const char *msg = "";
  if (g_lastFaultState) {
    msg = g_lastAngleOpenFault ? "ANGLE OPEN FAULT" : "ANGLE SENSOR FAULT";
  } else if (g_lastLevelFaultState) {
    msg = g_lastFloatOpenFault ? "LEVEL OPEN FAULT" : "LEVEL SENSOR FAULT";
  }

  if (!anyFault) {
    if (g_sharedFaultActive) {
      hmiSendText(HMI_COMP_FAULT_SHARED_TXT, "");
      setSharedFaultVisible(true); // leave it visible-but-empty, not stuck hidden mid-blink
      g_sharedFaultActive = false;
      g_sharedFaultMsg = "";
      g_faultBlinkVisible = true;
    }
    return;
  }

  // A fault is active: (re)show immediately on activation or on a message
  // change, so switching between angle/float faults isn't delayed by the
  // blink cadence.
  if (!g_sharedFaultActive || g_sharedFaultMsg != msg) {
    hmiSendText(HMI_COMP_FAULT_SHARED_TXT, msg);
    setSharedFaultVisible(true);
    g_faultBlinkVisible = true;
    g_lastBlinkToggleMs = now;
    g_sharedFaultActive = true;
    g_sharedFaultMsg = msg;
    return;
  }

  if (now - g_lastBlinkToggleMs >= FAULT_BLINK_INTERVAL_MS) {
    g_lastBlinkToggleMs = now;
    g_faultBlinkVisible = !g_faultBlinkVisible;
    setSharedFaultVisible(g_faultBlinkVisible);
  }
}

// ---------------------------------------------------------------------------
// X2 onboard buzzer: repeating FAULT_BUZZER_ON_MS-beep /
// FAULT_BUZZER_OFF_MS-silence pattern while any fault is active (new).
// TJC's documented `beep <ms>` instruction is a self-timed, fire-and-forget
// pulse (http://wiki.tjc1688.com/commands/beep.html) — the buzzer sounds
// for exactly the given duration and stops on its own, so this only needs
// to fire one `beep` at the start of each cycle and then wait; there's no
// "stop" instruction to send, and none is needed (a beep in progress when
// the fault clears just finishes on its own, at most FAULT_BUZZER_ON_MS
// later).
// ---------------------------------------------------------------------------
static bool g_buzzerActive = false; // whether the repeating cycle is running
static unsigned long g_buzzerCycleStartMs = 0;

static void fireBeep(unsigned long ms) {
  hmiSendRaw(String("beep ") + ms);
}

static void updateFaultBuzzer() {
  bool anyFault = g_lastFaultState || g_lastLevelFaultState;
  unsigned long now = millis();

  if (!anyFault) {
    g_buzzerActive = false;
    return;
  }

  if (!g_buzzerActive) {
    g_buzzerActive = true;
    g_buzzerCycleStartMs = now;
    fireBeep(FAULT_BUZZER_ON_MS);
    return;
  }

  unsigned long cycleLen = FAULT_BUZZER_ON_MS + FAULT_BUZZER_OFF_MS;
  if (now - g_buzzerCycleStartMs >= cycleLen) {
    g_buzzerCycleStartMs = now;
    fireBeep(FAULT_BUZZER_ON_MS);
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

  checkAdcReferenceRails(true); // baseline reading + print at boot

  DBG("[INIT] ready. Serial console: MIN / CENTER / MAX / SAVE / RESET / "
      "LVL0../LVL12 / LVLSAVE / LVLRESET / ADCCHK / STATUS\n");
}

void loop() {
  hmiPoll();
  pollDebugSerial();
  sampleFilterAndUpdate();
  sampleFloatLevelAndUpdate();
  checkAdcReferenceRails(); // self-paced; only actually samples every REFCHK_INTERVAL_MS
  updateSharedFaultDisplay();
  updateFaultBuzzer();
}
