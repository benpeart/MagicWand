#pragma once
#include <stdint.h>

#define ENABLE_DATA_CAPTURE

// -----------------------------
// Configuration
// -----------------------------
#define G 9.80665f
#define WINDOW_SIZE 256

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

