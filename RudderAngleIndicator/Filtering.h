// Filtering.h
// Small, dependency-free building blocks for the noise-reduction pipeline:
//   1. trimmedMean()   - rejects impulse/spike outliers within one batch
//   2. EmaFilter        - exponential moving average (software low-pass)
//   3. SlewLimiter       - clamps the rate of change to a physically
//                          plausible value, killing anything faster than the
//                          rudder itself could ever move
#pragma once
#include <Arduino.h>

// Sorts a copy of `samples` and returns the mean of the middle
// (n - 2*trim) values, discarding the `trim` lowest and `trim` highest
// readings. This removes single-sample spikes (contact noise on the sender
// wiper, EMI) that a plain average would let through.
float trimmedMean(const float *samples, int n, int trim);

class EmaFilter {
public:
  explicit EmaFilter(float alpha) : _alpha(alpha), _value(0), _initialized(false) {}

  float update(float x) {
    if (!_initialized) {
      _value = x;
      _initialized = true;
    } else {
      _value += _alpha * (x - _value);
    }
    return _value;
  }

  float value() const { return _value; }
  void reset() { _initialized = false; }

private:
  float _alpha;
  float _value;
  bool _initialized;
};

class SlewLimiter {
public:
  explicit SlewLimiter(float maxRatePerSec)
      : _maxRatePerSec(maxRatePerSec), _value(0), _lastMs(0), _initialized(false) {}

  float update(float target, unsigned long nowMs) {
    if (!_initialized) {
      _value = target;
      _lastMs = nowMs;
      _initialized = true;
      return _value;
    }
    float dtSec = (nowMs - _lastMs) / 1000.0f;
    _lastMs = nowMs;
    if (dtSec <= 0) return _value;

    float maxStep = _maxRatePerSec * dtSec;
    float delta = target - _value;
    if (delta > maxStep) delta = maxStep;
    if (delta < -maxStep) delta = -maxStep;
    _value += delta;
    return _value;
  }

  float value() const { return _value; }
  void reset() { _initialized = false; }

private:
  float _maxRatePerSec;
  float _value;
  unsigned long _lastMs;
  bool _initialized;
};
