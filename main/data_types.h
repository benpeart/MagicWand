#pragma once
#include <stdint.h>

// Normalized gesture size (fixed for inference)
#define IMU_RATE_HZ 200

#define PRE_TRIGGER_MS 500
#define PRE_TRIGGER_SAMPLES (((IMU_RATE_HZ * PRE_TRIGGER_MS) / 1000) + 1)

#define GESTURE_MS 3000
#define GESTURE_SAMPLES (((IMU_RATE_HZ * GESTURE_MS) / 1000) + 1)

#define POST_PADDING_MS 500
#define POST_PADDING_SAMPLES (((IMU_RATE_HZ * POST_PADDING_MS) / 1000) + 1)

#define INFERENCE_WINDOW_MS (PRE_TRIGGER_MS + GESTURE_MS + POST_PADDING_MS)
#define INFERENCE_WINDOW_SAMPLES (((IMU_RATE_HZ * INFERENCE_WINDOW_MS) / 1000) + 1)

// IMU sample structure (per-sample)
// This struct is the payload we send to the fusion queue.
typedef struct
{
    // Output time relative to gesture start (t0), in milliseconds.
    uint32_t timestamp_ms;

    // Linear acceleration (sensor frame), m/s^2
    float ax;
    float ay;
    float az;

    // Calibrated Gyroscope (sensor frame), rad/s
    float gx;
    float gy;
    float gz;

    // Quaternion (game rotation vector / fused orientation)
    float qw;
    float qx;
    float qy;
    float qz;
} GestureSample;

typedef enum
{
    GESTURE_NONE = 0,
    GESTURE_SPELL_1,
    GESTURE_SPELL_2,
    GESTURE_SPELL_3,
    GESTURE_SPELL_4,
    GESTURE_SPELL_5,
    // Add more as needed
} GestureType;

typedef struct
{
//    int64_t timestamp_us; // Absolute timestamp (microseconds from boot)
    GestureType gesture;
} GestureEvent;
