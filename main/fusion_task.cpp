#include "fusion_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queues.h"
#include "data_types.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>

// -----------------------------
// Task Configuration
// -----------------------------
static const char *TAG = "fusion_task";
#define FUSION_TASK_CORE 1
#define FUSION_TASK_PRIORITY 5
#define FUSION_TASK_STACK (10 * 1024)

#define DT_SEC (1.0f / IMU_RATE_HZ)

// Stillness detection
#define ACC_STILL_THRESH 3.0f
#define GYRO_STILL_THRESH 15.0f

// Movement detection
#define MOVE_ACCEL_THRESH 4.0f
#define MOVE_GYRO_THRESH 20.0f

// Gesture segmentation timing
#define STILLNESS_END_MS 300 // end gesture after 300ms stillness
#define MIN_GESTURE_MS 150   // reject gestures shorter than 150ms

// Madgwick quaternion state
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Integrated position state
static float pos_x = 0.0f;
static float pos_y = 0.0f;
static float pos_z = 0.0f;

static float vel_x = 0.0f;
static float vel_y = 0.0f;
static float vel_z = 0.0f;

// Gesture segmentation state
static bool gesture_active = false;
static uint32_t gesture_start_ts = 0;
static uint32_t last_motion_ts = 0;

static GestureSample gesture_buf[WINDOW_SIZE];
static int gesture_len = 0;

// -----------------------------
// Rotate vector by quaternion
// -----------------------------
static inline void rotate_vector(float *x, float *y, float *z)
{
    float vx = *x, vy = *y, vz = *z;

    float t2 = q0 * q1;
    float t3 = q0 * q2;
    float t4 = q0 * q3;
    float t5 = -q1 * q1;
    float t6 = q1 * q2;
    float t7 = q1 * q3;
    float t8 = -q2 * q2;
    float t9 = q2 * q3;
    float t1 = -q3 * q3;

    *x = 2.0f * ((t8 + t1) * vx + (t6 - t4) * vy + (t3 + t7) * vz) + vx;
    *y = 2.0f * ((t4 + t6) * vx + (t5 + t1) * vy + (t9 - t2) * vz) + vy;
    *z = 2.0f * ((t7 - t3) * vx + (t2 + t9) * vy + (t5 + t8) * vz) + vz;
}

// -----------------------------
// Madgwick update
// -----------------------------
static void madgwick_update(float gx, float gy, float gz,
                            float ax, float ay, float az)
{
    float recip_norm = ax * ax + ay * ay + az * az;
    if (recip_norm < 1e-6f)
        return;

    recip_norm = 1.0f / sqrtf(recip_norm);
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    gx *= 0.5f;
    gy *= 0.5f;
    gz *= 0.5f;

    float dq0 = (-q1 * gx - q2 * gy - q3 * gz);
    float dq1 = (q0 * gx + q2 * gz - q3 * gy);
    float dq2 = (q0 * gy - q1 * gz + q3 * gx);
    float dq3 = (q0 * gz + q1 * gy - q2 * gx);

    q0 += dq0 * DT_SEC;
    q1 += dq1 * DT_SEC;
    q2 += dq2 * DT_SEC;
    q3 += dq3 * DT_SEC;

    float norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

// -----------------------------
// Normalize gesture (center, scale, resample)
// -----------------------------
static int normalize_gesture(const GestureSample *in, int count, GestureSample *out)
{
    float cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < count; i++)
    {
        cx += in[i].x;
        cy += in[i].y;
    }
    cx /= count;
    cy /= count;

    float max_r = 0.0f;
    float centered_x[WINDOW_SIZE];
    float centered_y[WINDOW_SIZE];

    for (int i = 0; i < count; i++)
    {
        float x = in[i].x - cx;
        float y = in[i].y - cy;
        centered_x[i] = x;
        centered_y[i] = y;

        float r = sqrtf(x * x + y * y);
        if (r > max_r)
            max_r = r;
    }

    if (max_r < 1e-6f)
        max_r = 1e-6f;

    for (int i = 0; i < count; i++)
    {
        centered_x[i] /= max_r;
        centered_y[i] /= max_r;
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int src_idx = (i * count) / WINDOW_SIZE;
        if (src_idx >= count)
            src_idx = count - 1;

        out[i].timestamp_us = in[src_idx].timestamp_us;
        out[i].x = centered_x[src_idx];
        out[i].y = centered_y[src_idx];
    }

    return WINDOW_SIZE;
}

// -----------------------------
// Main fusion + segmentation task
// -----------------------------
static void fusion_task(void *arg)
{
    RawImuSample raw;

    ESP_LOGI(TAG, "Entering fusion_task");

    for (;;)
    {
        if (xQueueReceive(g_imu_queue, &raw, portMAX_DELAY) != pdTRUE)
            continue;

        //        ESP_LOGI(TAG, "xQueueReceive: %" PRIu64 ",%f,%f,%f,%f,%f,%f", raw.timestamp_us, raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz);
        int64_t now = raw.timestamp_us;

        // Convert gyro to rad/s
        float gx = raw.gx * (M_PI / 180.0f);
        float gy = raw.gy * (M_PI / 180.0f);
        float gz = raw.gz * (M_PI / 180.0f);

        // Update orientation
        madgwick_update(gx, gy, gz, raw.ax, raw.ay, raw.az);

        // Rotate accel into world frame
        float ax = raw.ax;
        float ay = raw.ay;
        float az = raw.az;
        rotate_vector(&ax, &ay, &az);

        // Remove gravity
        az -= G;

        // Stillness detection
        bool still =
            fabsf(ax) < ACC_STILL_THRESH &&
            fabsf(ay) < ACC_STILL_THRESH &&
            fabsf(az) < ACC_STILL_THRESH &&
            fabsf(raw.gx) < GYRO_STILL_THRESH &&
            fabsf(raw.gy) < GYRO_STILL_THRESH &&
            fabsf(raw.gz) < GYRO_STILL_THRESH;

        // Movement detection
        bool moving =
            fabsf(ax) > MOVE_ACCEL_THRESH ||
            fabsf(ay) > MOVE_ACCEL_THRESH ||
            fabsf(az) > MOVE_ACCEL_THRESH ||
            fabsf(raw.gx) > MOVE_GYRO_THRESH ||
            fabsf(raw.gy) > MOVE_GYRO_THRESH ||
            fabsf(raw.gz) > MOVE_GYRO_THRESH;

/*         ESP_LOGI(TAG, "Still: %s, Moving: %s: %" PRIu64 ",%f,%f,%f,%f,%f,%f", 
            still ? "true" : "false", moving ? "true" : "false", 
            raw.timestamp_us, ax, ay, az, raw.gx, raw.gy, raw.gz);
 */
        if (moving)
        {
            last_motion_ts = now;
        }

        // Integrate acceleration → velocity
        if (still)
        {
            vel_x = vel_y = vel_z = 0.0f;
        }
        else
        {
            vel_x += ax * DT_SEC;
            vel_y += ay * DT_SEC;
            vel_z += az * DT_SEC;
        }

        // Integrate velocity → position
        pos_x += vel_x * DT_SEC;
        pos_y += vel_y * DT_SEC;
        pos_z += vel_z * DT_SEC;

        // Project to 2D plane (X–Z)
        GestureSample out = {
            .timestamp_us = now,
            .x = pos_x,
            .y = pos_z};

        // -----------------------------
        // Gesture segmentation
        // -----------------------------
        if (!gesture_active)
        {
            if (moving)
            {
                ESP_LOGI(TAG, "Transition to gesture active");
                gesture_active = true;
                gesture_start_ts = now;
                gesture_len = 0;
            }
        }

        if (gesture_active)
        {
            if (gesture_len < WINDOW_SIZE)
            {
                gesture_buf[gesture_len++] = out;
            }

            uint32_t still_ms = (now - last_motion_ts) / 1000;

            if (still_ms > STILLNESS_END_MS)
            {
                uint32_t gesture_ms = (now - gesture_start_ts) / 1000;

                if (gesture_ms >= MIN_GESTURE_MS && gesture_len >= 32)
                {
                    GestureSample norm[WINDOW_SIZE];
                    int n = normalize_gesture(gesture_buf, gesture_len, norm);

                    for (int i = 0; i < n; i++)
                    {
                        xQueueSend(g_fusion_queue, &norm[i], 0);
                    }
                    ESP_LOGI(TAG, "Gesture captured: %d samples, %d ms", gesture_len, gesture_ms);
                }

                ESP_LOGI(TAG, "Transition to gesture inactive");
                gesture_active = false;
                gesture_len = 0;
            }
        }
    }
}

void fusion_task_start(void)
{
    xTaskCreatePinnedToCore(
        fusion_task,
        TAG,
        FUSION_TASK_STACK,
        NULL,
        FUSION_TASK_PRIORITY,
        NULL,
        FUSION_TASK_CORE);
}
