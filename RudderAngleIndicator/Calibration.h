// Calibration.h
// Stores the 3-point calibration (sender voltage at full-port, midships and
// full-starboard) in the ESP32's NVS flash (via Preferences) and converts a
// filtered sender voltage into an angle in degrees using piecewise-linear
// interpolation between those points. A 3-point curve is enough to correct
// for a mechanically off-center zero without the complexity of a full
// multi-point table, while still tracking the linear 0-190 ohm sender well.
#pragma once
#include <Arduino.h>
#include <Preferences.h>

struct CalPoints {
  float vAtMin;    // real sender volts at ANGLE_MIN_DEG
  float vAtCenter; // real sender volts at ANGLE_CENTER_DEG
  float vAtMax;    // real sender volts at ANGLE_MAX_DEG
};

class Calibration {
public:
  void begin();                 // loads from NVS, or writes factory defaults
  void loadDefaults();
  bool save();                  // persists current points to NVS
  bool resetToDefaults();       // overwrites NVS with factory defaults too

  void setMinFromVoltage(float v)    { _points.vAtMin = v; }
  void setCenterFromVoltage(float v) { _points.vAtCenter = v; }
  void setMaxFromVoltage(float v)    { _points.vAtMax = v; }

  const CalPoints &points() const { return _points; }

  // Converts a real (post divider-correction) sender voltage into degrees,
  // clamped to [ANGLE_MIN_DEG, ANGLE_MAX_DEG].
  float voltageToAngle(float senderVolts) const;

  // True if senderVolts falls within [min(vAtMin,vAtMax), max(vAtMin,vAtMax)]
  // plus a margin of marginFrac * that span on each end. Since it's derived
  // from your own captured calibration points rather than a fixed constant,
  // this works regardless of DIVIDER_RATIO or the sensor's absolute voltage
  // range — any real, in-range reading should always fall inside the
  // calibrated end-to-end span (plus a little overtravel allowance), so
  // anything outside it is implausible. Used as a fault check for a sensor
  // whose electrical range only covers a subset of the sensor's full travel
  // (e.g. a 360-degree sensor used over a much narrower operating window),
  // where a stuck/floating reading is likely to settle outside that narrow
  // window even though it's still well within the sensor's absolute limits.
  bool isWithinCalibratedRange(float senderVolts, float marginFrac) const;

private:
  Preferences _prefs;
  CalPoints _points;
};
