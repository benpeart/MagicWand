#pragma once
#include <stdint.h>

//#define DEBUG

// Normalized gesture size (fixed for inference)
// When you resample each captured gesture to 128 points, a 1 s gesture maps to 128 Hz
// effective resolution; a 3 s gesture maps to ~43 Hz effective resolution — both are
// sufficient for IMU gesture shapes while keeping the model small and fast.
#define INFERENCE_WINDOW_SIZE 128

// IMU sample structure (per-sample)
// This struct is the payload we send to the fusion queue.
typedef struct
{
    // Output time relative to gesture start (t0), in microseconds. This allows the model to learn temporal patterns.
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
