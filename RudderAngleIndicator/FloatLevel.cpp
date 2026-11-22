#include <math.h>
#include "FloatLevel.h"

static const char *NVS_NAMESPACE = "floatlvl";

// Computed from the real sender circuit; see the full table/formula
// comment by its declaration in Config.h.
const float DEFAULT_LEVEL_VOLTAGES[FLOAT_LEVEL_COUNT] = {
  0.0000f, 0.0994f, 0.1929f, 0.2812f, 0.3645f, 0.4434f, 0.5182f,
  0.5891f, 0.6565f, 0.7207f, 0.7818f, 0.8401f, 0.8957f
};

void FloatLevelSensor::loadDefaults() {
  for (int i = 0; i < FLOAT_LEVEL_COUNT; i++) {
    _levelVolts[i] = DEFAULT_LEVEL_VOLTAGES[i];
  }
}

void FloatLevelSensor::begin() {
  loadDefaults();
  if (_prefs.begin(NVS_NAMESPACE, true)) {
    for (int i = 0; i < FLOAT_LEVEL_COUNT; i++) {
      char key[8];
      snprintf(key, sizeof(key), "v%d", i);
      if (_prefs.isKey(key)) {
        _levelVolts[i] = _prefs.getFloat(key, _levelVolts[i]);
      }
    }
    _prefs.end();
  }
}

bool FloatLevelSensor::save() {
  if (!_prefs.begin(NVS_NAMESPACE, false)) return false;
  for (int i = 0; i < FLOAT_LEVEL_COUNT; i++) {
    char key[8];
    snprintf(key, sizeof(key), "v%d", i);
    _prefs.putFloat(key, _levelVolts[i]);
  }
  _prefs.end();
  return true;
}

bool FloatLevelSensor::resetToDefaults() {
  loadDefaults();
  return save();
}

void FloatLevelSensor::captureLevel(int index, float senderVolts) {
  if (index < 0 || index >= FLOAT_LEVEL_COUNT) return;
  _levelVolts[index] = senderVolts;
}

int FloatLevelSensor::nearestLevel(float senderVolts) const {
  int best = 0;
  float bestDist = fabsf(senderVolts - _levelVolts[0]);
  for (int i = 1; i < FLOAT_LEVEL_COUNT; i++) {
    float dist = fabsf(senderVolts - _levelVolts[i]);
    if (dist < bestDist) {
      bestDist = dist;
      best = i;
    }
  }
  return best;
}

int FloatLevelSensor::update(float senderVolts) {
  int nearest = nearestLevel(senderVolts);

  if (!_initialized) {
    _stableLevel = nearest;
    _candidateLevel = nearest;
    _candidateCount = LEVEL_DEBOUNCE_BATCHES;
    _initialized = true;
    return _stableLevel;
  }

  if (nearest == _candidateLevel) {
    if (_candidateCount < LEVEL_DEBOUNCE_BATCHES) _candidateCount++;
  } else {
    _candidateLevel = nearest;
    _candidateCount = 1;
  }

  if (_candidateCount >= LEVEL_DEBOUNCE_BATCHES) {
    _stableLevel = _candidateLevel;
  }

  return _stableLevel;
}
