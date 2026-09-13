#include "Filtering.h"

float trimmedMean(const float *samples, int n, int trim) {
  if (n <= 0) return 0.0f;
  if (trim * 2 >= n) trim = 0; // safety: never trim everything away

  // Small n (RAW_SAMPLES_PER_BATCH is ~15) -> insertion sort is plenty fast
  // and avoids pulling in <algorithm>/heap allocation.
  float sorted[64];
  int count = n > 64 ? 64 : n;
  for (int i = 0; i < count; i++) sorted[i] = samples[i];

  for (int i = 1; i < count; i++) {
    float key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  float sum = 0.0f;
  int used = 0;
  for (int i = trim; i < count - trim; i++) {
    sum += sorted[i];
    used++;
  }
  return used > 0 ? sum / used : sorted[count / 2];
}
