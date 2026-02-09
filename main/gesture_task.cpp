#include "gesture_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <inttypes.h>
#include "mpu6050.h"
#include "queues.h"
#include "data_types.h"

// -----------------------------
// Task Configuration
// -----------------------------
static const char *TAG = "gesture_task";
#define GESTURE_TASK_CORE 1
#define GESTURE_TASK_PRIORITY 8
#define GESTURE_TASK_STACK (16 * 1024)

// -----------------------------
// I2C pins for ESP32-S3
// -----------------------------
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN GPIO_NUM_8
#define I2C_SCL_PIN GPIO_NUM_9

static mpu6050_dev_t mpu;

// -----------------------------
// IMU configuration
// -----------------------------

// Use nominal IMU rate only as a fallback
#define DT_FALLBACK_SEC (1.0f / IMU_RATE_HZ)

// Madgwick gain
// Tune beta based on noise; start moderate
#define MADGWICK_BETA 0.18f

// How fast gravity estimate adapts when still
#define GRAVITY_LPF_ALPHA 0.03f // 1% update per still frame

// Madgwick quaternion state
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Integrated position/velocity state
static float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;
static float vel_x = 0.0f, vel_y = 0.0f, vel_z = 0.0f;

// Standard gravity in m/s²
#define GRAVITY 9.80665f

// Gravity estimate in world frame
static float g_est_x = 0.0f;
static float g_est_y = 0.0f;
static float g_est_z = GRAVITY; // start with nominal gravity

// -----------------------------
// Gesture configuration
// -----------------------------

// Thresholds tuned from captured data
const float STILL_ENERGY_MAX = 1.05f; // energy near 1g
const float STILL_GYRO_MAX = 0.05f;   // rad/s, very low rotation
const float MOVING_START_THRESH = 1.8f;
const float MOVING_STOP_THRESH = 1.2f;

// Gesture segmentation timing
#define STILLNESS_END_MS 300 // end gesture after 300ms stillness
#define MIN_GESTURE_MS 750   // reject gestures shorter than 750ms
#define MAX_GESTURE_MS 3000  // hard cap at 3s

// Gesture segmentation state
static bool gesture_active = false;
static uint64_t gesture_start_ts = 0;
static uint64_t last_motion_ts = 0;

static GestureSample gesture_buf[MAX_RAW_GESTURE_SAMPLES];
static int gesture_len = 0;

static inline float clamp01(float x)
{
    return fmaxf(0.0f, fminf(1.0f, x));
}

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
static void gesture_task(void *arg)
{
    ESP_LOGI(TAG, "Entering gesture_task");
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_ts = 0;

    for (;;)
    {
        // Maintain fixed IMU_RATE_HZ schedule using FreeRTOS
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / IMU_RATE_HZ));

        // ------------------------------------------------
        // Compute Linear Acceleration In World Coordinates
        // ------------------------------------------------

        // grab next IMU sample
        mpu6050_raw_acceleration_t raw_acc;
        mpu6050_raw_rotation_t raw_rot;

        if (mpu6050_get_raw_acceleration(&mpu, &raw_acc) != ESP_OK ||
            mpu6050_get_raw_rotation(&mpu, &raw_rot) != ESP_OK)
        {
            ESP_LOGW(TAG, "IMU read failed");
            continue;
        }
        int64_t now = esp_timer_get_time();

        // Compute dt (seconds)
        float dt = DT_FALLBACK_SEC;
        if (last_ts != 0 && now > last_ts)
        {
            dt = (float)(now - last_ts) / 1e6f;
        }
        // Validate dt for 200Hz sampling (0.005s nominal, allow ±100% variance)
        if (dt <= 0.0f || dt > 0.01f)
        {
            ESP_LOGW(TAG, "Anomalous dt: %.6f s (expected ~0.005s at 200Hz)", dt);
        }
        last_ts = now;

        // Convert from raw accel (LSB) to m/s²
        const float accel_scale = 1.0f / 8192.0f; // ±4g
        float ax = (float)raw_acc.x * accel_scale * GRAVITY;
        float ay = (float)raw_acc.y * accel_scale * GRAVITY;
        float az = (float)raw_acc.z * accel_scale * GRAVITY;

        // Conversion from raw gyro (LSB) to rad/s
        const float gyro_scale_rad = (1.0f / 65.5f) * (M_PI / 180.0f); // for ±500°/s
        float gx = (float)raw_rot.x * gyro_scale_rad;
        float gy = (float)raw_rot.y * gyro_scale_rad;
        float gz = (float)raw_rot.z * gyro_scale_rad;

        // Update orientation with Madgwick
        madgwick_update(gx, gy, gz, ax, ay, az, dt);

        // Rotate accel into world frame
        rotate_vector(&ax, &ay, &az);

        // After rotate_vector, before subtracting gravity:
        float a_world_x = ax;
        float a_world_y = ay;
        float a_world_z = az;

        // Remove estimated gravity (world frame → linear accel)
        ax -= g_est_x;
        ay -= g_est_y;
        az -= g_est_z;

        // Compute motion energy for stillness/movement detection
        float a_mag = sqrtf(ax * ax + ay * ay + az * az);
        float g_mag = sqrtf(gx * gx + gy * gy + gz * gz);
        float motion_energy = a_mag + 0.15f * g_mag;

        // Smooth
        static float smoothed_motion_energy = 0.0f;
        smoothed_motion_energy = 0.85f * smoothed_motion_energy + 0.15f * motion_energy;

        // ------------------------------------------
        // Remove acceleration due to gravity adaptively
        // ------------------------------------------
#if 1
        // Motion energy already computed: smoothed_motion_energy

        // Map motion energy to a [0,1] "stillness factor"
        float still_factor = 1.0f - clamp01((smoothed_motion_energy - 1.0f) / (MOVING_START_THRESH - 1.0f));

        // still_factor ≈1 near rest, →0 as you approach strong motion
        float alpha = GRAVITY_LPF_ALPHA * still_factor;
        g_est_x = (1.0f - alpha) * g_est_x + alpha * a_world_x;
        g_est_y = (1.0f - alpha) * g_est_y + alpha * a_world_y;
        g_est_z = (1.0f - alpha) * g_est_z + alpha * a_world_z;
#endif
#ifdef GRAVITY_DURING_STILLNESS
        // Instantaneous stillness
        // Bad and removes too much acceleration-based motion
        bool inst_still = (smoothed_motion_energy < STILL_ENERGY_MAX) && (g_mag < STILL_GYRO_MAX);

        // Require N consecutive still frames to avoid flicker
        static int still_count = 0;
        if (inst_still)
        {
            still_count++;
        }
        else
        {
            still_count = 0;
        }
        bool still = (still_count >= 20); // ~100 ms at 200 Hz

        // Gravity estimation (only when still)
        if (still)
        {
            g_est_x = (1.0f - GRAVITY_LPF_ALPHA) * g_est_x + GRAVITY_LPF_ALPHA * ax;
            g_est_y = (1.0f - GRAVITY_LPF_ALPHA) * g_est_y + GRAVITY_LPF_ALPHA * ay;
            g_est_z = (1.0f - GRAVITY_LPF_ALPHA) * g_est_z + GRAVITY_LPF_ALPHA * az;
        }
#endif // GRAVITY_DURING_STILLNESS
#ifdef GRAVITY_FROM_QUATERNION
        // Compute gravity from the quaternion every frame
        // Terrible as it removes all acceleration based motion
        g_est_x = (2 * (q1 * q3 - q0 * q2)) * GRAVITY;
        g_est_y = (2 * (q0 * q1 + q2 * q3)) * GRAVITY;
        g_est_z = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * GRAVITY;
#endif // GRAVITY_FROM_QUATERNION

#ifdef DEBUG
        // Output one clean CSV-style line per sample to use to tune thresholds.
        ESP_LOGI("GESTURE_DBG",
                 "RAW_ACC: %d,%d,%d  "
                 "RAW_GYRO: %d,%d,%d  "
                 "LIN_ACC: %.3f,%.3f,%.3f  "
                 "GYRO_RAD: %.3f,%.3f,%.3f  "
                 "A_MAG: %.5f  G_MAG: %.5f  E: %.5f  ACTIVE:%d",
                 raw_acc.x, raw_acc.y, raw_acc.z,
                 raw_rot.x, raw_rot.y, raw_rot.z,
                 ax, ay, az,
                 gx, gy, gz,
                 a_mag, g_mag, motion_energy,
                 gesture_active ? 1 : 0);
#endif
        // ------------------------------------------
        // Detect when we start to cast a spell
        // ------------------------------------------

        bool moving = smoothed_motion_energy > MOVING_START_THRESH;
#if 0
        static int counter = 0;
        if (++counter % 50 == 0)
        {
            ESP_LOGI(TAG, "Still: %s, Moving: %s: %" PRIu64 ",%f",
                     still ? "true" : "false", moving ? "true" : "false",
                     now, smoothed_motion_energy);
        }
#endif // DEBUG
        if (moving)
        {
            last_motion_ts = now;
        }

        // Detect motion and transition to gesture capture
        if (!gesture_active && moving)
        {
            ESP_LOGI(TAG, "Transition to gesture active");
            gesture_active = true;
            gesture_start_ts = now;
            gesture_len = 0;

            // Reset state for fresh gesture tracking (before first integration)
            vel_x = vel_y = vel_z = 0.0f;
            pos_x = pos_y = pos_z = 0.0f;
        }

        // ---------------------------------------------
        // Capture the Gesture and detect end of gesture
        // ---------------------------------------------
        if (gesture_active)
        {
            // compute and save the gesture sample in the gesture buffer
            if (gesture_len < MAX_RAW_GESTURE_SAMPLES)
            {
                // Integrate acceleration → velocity → position
                vel_x += ax * dt;
                vel_y += ay * dt;
                vel_z += az * dt;

                pos_x += vel_x * dt;
                pos_y += vel_y * dt;
                pos_z += vel_z * dt;

                // Project to 2D plane (X–Z) for gesture
                GestureSample out = {
                    .timestamp_us = now,
                    .x = pos_x,
                    .y = pos_z};

                // store the position in our gesture buffer
                gesture_buf[gesture_len++] = out;
            }
#ifdef DEBUG
            else
            {
                ESP_LOGW(TAG, "Raw gesture buffer overflow — gesture truncated at %d samples", gesture_len);
            }
#endif // DEBUG

            // detect the end of gesture
            uint32_t gesture_ms = (uint32_t)((now - gesture_start_ts) / 1000);
            bool time_exceeded = (gesture_ms > MAX_GESTURE_MS);

            // End gesture
            if (smoothed_motion_energy < MOVING_STOP_THRESH || time_exceeded)
            {
                // remove all the trailing stillness samples by only keeping samples with timestamp <= last_motion_ts
                while (gesture_len > 0 && gesture_buf[gesture_len - 1].timestamp_us > last_motion_ts)
                {
                    gesture_len--;
                }

                // reject too-short gestures
                if (gesture_ms >= MIN_GESTURE_MS)
                {
                    // normalize the gesture samples
                    GestureSample norm[INFERENCE_WINDOW_SIZE];
                    int n = normalize_gesture(gesture_buf, gesture_len, norm);

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
                    ESP_LOGI(TAG, "Gesture captured: %d samples, %d ms%s", gesture_len, gesture_ms, time_exceeded ? " (time-capped)" : "");
                }
                else
                {
                    ESP_LOGI(TAG, "Gesture rejected: %d samples, %d ms (too short)", gesture_len, gesture_ms);
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

void gesture_task_start(void)
{
    ESP_ERROR_CHECK(i2cdev_init());

    // Create device descriptor
    ESP_ERROR_CHECK(mpu6050_init_desc(
        &mpu,
        MPU6050_I2C_ADDRESS_LOW,
        I2C_PORT,
        I2C_SDA_PIN,
        I2C_SCL_PIN));

    // Initialize chip
    ESP_ERROR_CHECK(mpu6050_init(&mpu));

    // Configure full-scale ranges
    ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&mpu, MPU6050_ACCEL_RANGE_4));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&mpu, MPU6050_GYRO_RANGE_500));
    ESP_LOGI(TAG, "MPU6050 initialized (esp-idf-lib)");

    xTaskCreatePinnedToCore(
        gesture_task,
        TAG,
        GESTURE_TASK_STACK,
        NULL,
        GESTURE_TASK_PRIORITY,
        NULL,
        GESTURE_TASK_CORE);
}
