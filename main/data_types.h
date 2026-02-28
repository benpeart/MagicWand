#pragma once
#include <stdint.h>

// Normalized gesture size (fixed for inference)
// Set to 300 samples (3.0 s @ 100 Hz) as recommended for 1-3 s gestures.
#define INFERENCE_WINDOW_SIZE 300

// IMU sample structure (per-sample)
// This struct is the payload we send to the fusion queue.
typedef struct
{
    // Output time relative to gesture start (t0), in microseconds.
    uint32_t timestamp_us;

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
    int64_t timestamp_us; // Absolute timestamp (microseconds from boot)
    GestureType gesture;
} GestureEvent;
