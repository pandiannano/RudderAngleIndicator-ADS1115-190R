#include "Calibration.h"
#include "Config.h"

static const char *NVS_NAMESPACE = "ruddercal";

void Calibration::loadDefaults() {
  _points.vAtMin = DEFAULT_V_AT_MIN;
  _points.vAtCenter = DEFAULT_V_AT_CENTER;
  _points.vAtMax = DEFAULT_V_AT_MAX;
}

void Calibration::begin() {
  loadDefaults();
  if (_prefs.begin(NVS_NAMESPACE, true)) { // read-only open
    if (_prefs.isKey("vMin"))    _points.vAtMin    = _prefs.getFloat("vMin", _points.vAtMin);
    if (_prefs.isKey("vCenter")) _points.vAtCenter = _prefs.getFloat("vCenter", _points.vAtCenter);
    if (_prefs.isKey("vMax"))    _points.vAtMax    = _prefs.getFloat("vMax", _points.vAtMax);
    _prefs.end();
  }
}

bool Calibration::save() {
  if (!_prefs.begin(NVS_NAMESPACE, false)) return false;
  _prefs.putFloat("vMin", _points.vAtMin);
  _prefs.putFloat("vCenter", _points.vAtCenter);
  _prefs.putFloat("vMax", _points.vAtMax);
  _prefs.end();
  return true;
}

bool Calibration::resetToDefaults() {
  loadDefaults();
  return save();
}

float Calibration::voltageToAngle(float senderVolts) const {
  float angle;

  if (senderVolts <= _points.vAtCenter) {
    // Interpolate between (vAtMin -> ANGLE_MIN_DEG) and (vAtCenter -> ANGLE_CENTER_DEG)
    float span = _points.vAtCenter - _points.vAtMin;
    float frac = (span != 0.0f) ? (senderVolts - _points.vAtMin) / span : 0.0f;
    angle = ANGLE_MIN_DEG + frac * (ANGLE_CENTER_DEG - ANGLE_MIN_DEG);
  } else {
    // Interpolate between (vAtCenter -> ANGLE_CENTER_DEG) and (vAtMax -> ANGLE_MAX_DEG)
    float span = _points.vAtMax - _points.vAtCenter;
    float frac = (span != 0.0f) ? (senderVolts - _points.vAtCenter) / span : 0.0f;
    angle = ANGLE_CENTER_DEG + frac * (ANGLE_MAX_DEG - ANGLE_CENTER_DEG);
  }

  if (angle < ANGLE_MIN_DEG) angle = ANGLE_MIN_DEG;
  if (angle > ANGLE_MAX_DEG) angle = ANGLE_MAX_DEG;

  if (INVERT_ANGLE) angle = -angle;

  if (angle < ANGLE_MIN_DEG) angle = ANGLE_MIN_DEG;
  if (angle > ANGLE_MAX_DEG) angle = ANGLE_MAX_DEG;

  return angle;
}
