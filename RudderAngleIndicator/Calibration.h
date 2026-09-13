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

private:
  Preferences _prefs;
  CalPoints _points;
};
