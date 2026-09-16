// FloatLevel.h
// Decodes the AIN1 floating (float-arm) level sender: a 0-190 ohm sender
// that only ever rests at FLOAT_LEVEL_COUNT (10) discrete resistance steps,
// rather than sweeping continuously like the AIN0 angle sender.
//
// Each level's expected sender voltage is calibrated and stored in NVS.
// Decoding is nearest-match against that table, with a debounce counter so
// a level is only reported once several consecutive readings agree — this
// rejects the transient/noisy readings produced while the float arm is
// physically moving between two steps.
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "Config.h"

class FloatLevelSensor {
public:
  void begin();                     // loads from NVS, or writes factory defaults
  void loadDefaults();
  bool save();
  bool resetToDefaults();

  void captureLevel(int index, float senderVolts); // index 0..FLOAT_LEVEL_COUNT-1
  float levelVoltage(int index) const { return _levelVolts[index]; }

  // Feeds one new (already batch-averaged) sender-voltage reading through
  // the nearest-match + debounce logic and returns the current stable
  // level (0..FLOAT_LEVEL_COUNT-1).
  int update(float senderVolts);

  int stableLevel() const { return _stableLevel; }

private:
  int nearestLevel(float senderVolts) const;

  Preferences _prefs;
  float _levelVolts[FLOAT_LEVEL_COUNT];

  int _candidateLevel = 0;
  int _candidateCount = 0;
  int _stableLevel = 0;
  bool _initialized = false;
};
