#include "fusion_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queues.h"
#include "data_types.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <inttypes.h>

// -----------------------------
// Task Configuration
// -----------------------------
static const char *TAG = "fusion_task";
#define FUSION_TASK_CORE 1
#define FUSION_TASK_PRIORITY 5
#define FUSION_TASK_STACK (16 * 1024)

// Use nominal IMU rate only as a fallback
#define DT_FALLBACK_SEC (1.0f / IMU_RATE_HZ)

// Madgwick gain
// Tune beta based on noise; start moderate
#define MADGWICK_BETA 0.18f

// How fast gravity estimate adapts when still
#define GRAVITY_LPF_ALPHA 0.03f // 1% update per still frame

// Stillness detection thresholds
#define ACC_STILL_THRESH 0.7f
#define GYRO_STILL_THRESH 4.0f

// Movement detection thresholds
#define MOVE_ACCEL_THRESH 0.9f
#define MOVE_GYRO_THRESH 15.0f

// Gesture segmentation timing
#define STILLNESS_END_MS 300 // end gesture after 300ms stillness
#define MIN_GESTURE_MS 750   // reject gestures shorter than 750ms
#define MAX_GESTURE_MS 3000  // hard cap at 3s

// Madgwick quaternion state
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Integrated position/velocity state
static float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;
static float vel_x = 0.0f, vel_y = 0.0f, vel_z = 0.0f;

// Gravity estimate in world frame
static float g_est_x = 0.0f;
static float g_est_y = 0.0f;
static float g_est_z = 9.80665f; // start with nominal gravity

// Gesture segmentation state
static bool gesture_active = false;
static uint64_t gesture_start_ts = 0;
static uint64_t last_motion_ts = 0;

static GestureSample gesture_buf[MAX_RAW_GESTURE_SAMPLES];
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
// Madgwick update (full, with accel feedback)
// gx,gy,gz in rad/s, ax,ay,az in m/s^2, dt in seconds
// -----------------------------
static void madgwick_update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt)
{
    // Normalize accelerometer
    float norm = ax * ax + ay * ay + az * az;
    if (norm < 1e-6f)
        return;
    norm = 1.0f / sqrtf(norm);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // Gradient descent algorithm corrective step
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (norm > 1e-12f)
    {
        norm = 1.0f / sqrtf(norm);
        s0 *= norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
    }

    // Apply feedback step
    float beta = MADGWICK_BETA;
    gx -= beta * s1;
    gy -= beta * s2;
    gz -= beta * s3;

    // Integrate rate of change of quaternion
    float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    // Normalize quaternion
    norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

// -----------------------------
// Normalize gesture (resample, center, scale)
// -----------------------------
static int normalize_gesture(const GestureSample *in, int count, GestureSample *out)
{
    // --------------------------------------------------------
    // 0. Degenerate gesture check
    // --------------------------------------------------------
    if (count < 2)
    {
        for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
        {
            out[i].timestamp_us = in[0].timestamp_us;
            out[i].x = 0.0f;
            out[i].y = 0.0f;
        }
        return INFERENCE_WINDOW_SIZE;
    }

    // --------------------------------------------------------
    // 1. Compute cumulative arc-length
    // --------------------------------------------------------
    float dist[count];
    dist[0] = 0.0f;

    for (int i = 1; i < count; i++)
    {
        float dx = in[i].x - in[i - 1].x;
        float dy = in[i].y - in[i - 1].y;
        float d = sqrtf(dx * dx + dy * dy);
        dist[i] = dist[i - 1] + d;
    }

    // --------------------------------------------------------
    // 2. Enforce strict monotonicity (critical!)
    // --------------------------------------------------------
    for (int i = 1; i < count; i++)
    {
        if (dist[i] < dist[i - 1])
        {
            dist[i] = dist[i - 1];
        }
    }

    float total_len = dist[count - 1];

    // If gesture is basically a point
    if (total_len < 1e-6f)
    {
        for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
        {
            out[i].timestamp_us = in[0].timestamp_us;
            out[i].x = 0.0f;
            out[i].y = 0.0f;
        }
        return INFERENCE_WINDOW_SIZE;
    }

    // --------------------------------------------------------
    // 3. Arc-length resampling
    // --------------------------------------------------------
    float step = total_len / (INFERENCE_WINDOW_SIZE - 1);
    float target = 0.0f;

    float rx[INFERENCE_WINDOW_SIZE], ry[INFERENCE_WINDOW_SIZE];
    int src = 0;

    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {

        while (src < count - 2 && dist[src + 1] < target)
        {
            src++;
        }

        float d0 = dist[src];
        float d1 = dist[src + 1];
        float span = d1 - d0;

        float alpha = 0.0f;
        if (span > 1e-6f)
        {
            alpha = (target - d0) / span;
        }

        float x0 = in[src].x;
        float y0 = in[src].y;
        float x1 = in[src + 1].x;
        float y1 = in[src + 1].y;

        rx[i] = x0 + alpha * (x1 - x0);
        ry[i] = y0 + alpha * (y1 - y0);

        target += step;
    }

    // --------------------------------------------------------
    // 4. Centering
    // --------------------------------------------------------
    float cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        cx += rx[i];
        cy += ry[i];
    }
    cx /= INFERENCE_WINDOW_SIZE;
    cy /= INFERENCE_WINDOW_SIZE;

    // --------------------------------------------------------
    // 5. Unit-circle scaling
    // --------------------------------------------------------
    float max_r = 0.0f;
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        float x = rx[i] - cx;
        float y = ry[i] - cy;
        float r = sqrtf(x * x + y * y);
        if (r > max_r)
            max_r = r;
    }

    if (max_r < 1e-6f)
        max_r = 1e-6f;

    // --------------------------------------------------------
    // 6. Output
    // --------------------------------------------------------
    uint32_t ts = in[0].timestamp_us;
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        out[i].timestamp_us = ts;
        out[i].x = (rx[i] - cx) / max_r;
        out[i].y = (ry[i] - cy) / max_r;
    }

#ifdef DEBUG
    ESP_LOGI(TAG, "normalize_gesture: raw=%d, out=%d, total_len=%.6f, centroid=(%.6f, %.6f)",
             count, INFERENCE_WINDOW_SIZE, total_len, cx, cy);
#endif

    return INFERENCE_WINDOW_SIZE;
}

// ---------------------------------------------
// Main fusion, segmentation, normalization task
// ---------------------------------------------
static void fusion_task(void *arg)
{
    RawImuSample raw;
    ESP_LOGI(TAG, "Entering fusion_task");

    uint64_t last_ts = 0;

    for (;;)
    {
        if (xQueueReceive(g_imu_queue, &raw, portMAX_DELAY) != pdTRUE)
            continue;

        int64_t now = raw.timestamp_us;

        // Compute dt from timestamps (seconds)
        float dt = DT_FALLBACK_SEC;
        if (last_ts != 0 && now > last_ts)
        {
            dt = (float)(now - last_ts) / 1e6f;
        }
        last_ts = now;

        // Convert gyro to rad/s
        float gx = raw.gx * (M_PI / 180.0f);
        float gy = raw.gy * (M_PI / 180.0f);
        float gz = raw.gz * (M_PI / 180.0f);

        // Update orientation with real Madgwick
        madgwick_update(gx, gy, gz, raw.ax, raw.ay, raw.az, dt);

        // Rotate accel into world frame (raw)
        float ax_w = raw.ax;
        float ay_w = raw.ay;
        float az_w = raw.az;
        rotate_vector(&ax_w, &ay_w, &az_w);

        // Remove estimated gravity (world frame → linear accel)
        float ax = ax_w - g_est_x;
        float ay = ay_w - g_est_y;
        float az = az_w - g_est_z;

        // Stillness detection (raw accel + raw gyro)
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
#ifdef DEBUG
        static int counter = 0;
        if (++counter % 50 == 0)
        {
            ESP_LOGI(TAG, "xQueueReceive: %" PRIu64 ",%f,%f,%f,%f,%f,%f",
                     raw.timestamp_us, raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz);
            ESP_LOGI(TAG, "Still: %s, Moving: %s: %" PRIu64 ",%f,%f,%f,%f,%f,%f",
                     still ? "true" : "false", moving ? "true" : "false",
                     raw.timestamp_us, ax, ay, az, raw.gx, raw.gy, raw.gz);
        }
#endif // DEBUG
        if (moving)
        {
            last_motion_ts = now;
        }

        // Gravity estimation (only when still)
        if (still)
        {
            g_est_x = (1.0f - GRAVITY_LPF_ALPHA) * g_est_x + GRAVITY_LPF_ALPHA * ax_w;
            g_est_y = (1.0f - GRAVITY_LPF_ALPHA) * g_est_y + GRAVITY_LPF_ALPHA * ay_w;
            g_est_z = (1.0f - GRAVITY_LPF_ALPHA) * g_est_z + GRAVITY_LPF_ALPHA * az_w;
        }

        // Integrate acceleration → velocity → position
        if (still && !gesture_active)
        {
            // Aggressive reset when still: forget history (but NOT during gesture)
            vel_x = vel_y = vel_z = 0.0f;
            pos_x = pos_y = pos_z = 0.0f;
        }
        else
        {
            vel_x += ax * dt;
            vel_y += ay * dt;
            vel_z += az * dt;

            pos_x += vel_x * dt;
            pos_y += vel_y * dt;
            pos_z += vel_z * dt;
        }

        // Project to 2D plane (X–Z) for gesture
        GestureSample out = {
            .timestamp_us = now,
            .x = pos_x,
            .y = pos_z};

        // -----------------------------
        // Gesture segmentation
        // -----------------------------
        if (!gesture_active && moving)
        {
            ESP_LOGI(TAG, "Transition to gesture active");
            gesture_active = true;
            gesture_start_ts = now;
            gesture_len = 0;
        }

        if (gesture_active)
        {
            if (gesture_len < sizeof(gesture_buf) / sizeof(gesture_buf[0]))
            {
                gesture_buf[gesture_len++] = out;
            }
            else
            {
                ESP_LOGW(TAG, "Raw gesture buffer overflow — gesture truncated at %d samples", gesture_len);
            }

            uint32_t still_ms = (uint32_t)((now - last_motion_ts) / 1000);
            uint32_t gesture_ms = (uint32_t)((now - gesture_start_ts) / 1000);

            bool time_exceeded = (gesture_ms > MAX_GESTURE_MS);

            if (still_ms > STILLNESS_END_MS || time_exceeded)
            {
                if (gesture_ms >= MIN_GESTURE_MS)
                {
#if 1
                    ESP_LOGI(TAG, "Pre normalized count[%d]\n", gesture_len);
                    for (int i = 0; i < gesture_len; i++)
                    {
                        printf("%" PRIu64 ",%f,%f\n", gesture_buf[i].timestamp_us, gesture_buf[i].x, gesture_buf[i].y);
                    }
#endif
                    // normalize the gesture samples
                    GestureSample norm[INFERENCE_WINDOW_SIZE];
                    int n = normalize_gesture(gesture_buf, gesture_len, norm);
#if 1
                    ESP_LOGI(TAG, "Normalized count[%d]\n", n);
                    for (int i = 0; i < n; i++)
                    {
                        printf("%" PRIu64 ",%f,%f\n", norm[i].timestamp_us, norm[i].x, norm[i].y);
                    }
#endif // DEBUG

                    // Send start-of-gesture marker
                    GestureSample marker = {
                        .timestamp_us = 0,
                        .x = NAN,
                        .y = NAN};
                    xQueueSend(g_fusion_queue, &marker, 0);

                    // Send normalized gesture samples
                    for (int i = 0; i < n; i++)
                    {
                        if (xQueueSend(g_fusion_queue, &norm[i], 0) != pdTRUE)
                        {
                            ESP_LOGW(TAG, "fusion_queue full, dropping sample %d of %d", i, n);
                        }
                    }
                    ESP_LOGI(TAG, "Gesture captured: %d samples, %d ms%s",
                             gesture_len, gesture_ms,
                             time_exceeded ? " (time-capped)" : "");
                }
                else
                {
                    ESP_LOGI(TAG, "Gesture rejected: %d samples, %d ms (too short)",
                             gesture_len, gesture_ms);
                }

                ESP_LOGI(TAG, "Transition to gesture inactive");
                gesture_active = false;
                gesture_len = 0;
                // Also reset state so next gesture starts fresh
                vel_x = vel_y = vel_z = 0.0f;
                pos_x = pos_y = pos_z = 0.0f;
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
