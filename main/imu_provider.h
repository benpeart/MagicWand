#ifndef IMU_PROVIDER_H
#define IMU_PROVIDER_H

#include "tensorflow/lite/c/common.h"

TfLiteStatus GetIMUSamples(int start_ms, int duration_ms, int *audio_samples_size, int16_t **audio_samples);

#endif // IMU_PROVIDER_H
