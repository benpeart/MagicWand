#pragma once
#include <stdint.h>

//#define DEBUG
#define ENABLE_DATA_CAPTURE

// Normalized gesture size (fixed for inference)
#define INFERENCE_WINDOW_SIZE 256

// Raw gesture capture buffer size (200 Hz * 3 sec = 600 samples) 
#define MAX_RAW_GESTURE_SAMPLES 640

// 200 Hz sampling
#define IMU_RATE_HZ 200

// Raw IMU sample (from imu_task)
typedef struct
{
    int64_t timestamp_us;
    float ax, ay, az;
    float gx, gy, gz;
} RawImuSample;

// gesture sample (from fusion_task)
typedef struct {
    int64_t timestamp_us;
    float x;   // normalized x
    float y;   // normalized y
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
    int64_t timestamp_us;
    GestureType gesture;
} GestureEvent;

