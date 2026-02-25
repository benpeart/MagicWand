//
// Rewritten gesture capture task to output linear accel + gyro + quaternion per sample
// Sends fixed-length, time-resampled, per-channel normalized IMU windows to g_fusion_queue.
//
// Notes:
// - This file assumes g_fusion_queue is configured to accept GestureSample objects.
// - We capture sensor-frame linear acceleration (gravity removed by BNO08x), raw gyroscope,
//   and the fused game rotation quaternion for each sample.
// - We perform time-based resampling to INFERENCE_WINDOW_SIZE and per-channel normalization
//   before sending the window to the fusion queue.

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <math.h>
#include <inttypes.h>
#include <float.h>
#include <cassert>
#include "BNO08x.hpp"
#include "queues.h"
#include "data_types.h"
#include "gesture_task.h"

// =========================================================================
// Task Configuration
// =========================================================================
static const char *TAG = "gesture_task";
#define GESTURE_TASK_CORE 1
#define GESTURE_TASK_PRIORITY 4  // Must be lower than BNO08x driver tasks (5,6,7) to avoid starving them
#define GESTURE_TASK_STACK (16 * 1024)

// =========================================================================
// BNO08x Driver Instance
// =========================================================================
static BNO08x imu;

// =========================================================================
// Gesture sampling rates
// =========================================================================
#define IMU_RATE_HZ 200
#define BNO08X_RATE_GAMERV (1000000 / IMU_RATE_HZ)           // reporting interval in microseconds
#define BNO08X_RATE_LINACC (1000000 / IMU_RATE_HZ)           // reporting interval in microseconds
#define BNO08X_RATE_GYRO (1000000 / IMU_RATE_HZ)             // reporting interval in microseconds
#define BNO08X_RATE_STABILITY (4000000 / IMU_RATE_HZ)        // reporting interval in microseconds, 1/4 sampling rate (50 Hz when IMU_RATE_HZ=200)

// =========================================================================
// Gesture Configuration
// =========================================================================
// Thresholds for motion detection based on linear acceleration magnitude (sensor frame)
const float MOVING_START_THRESH = 2.0f; // m/s²: start gesture
const float MOVING_STOP_THRESH = 1.0f;  // m/s²: stop gesture

// Gesture segmentation timing
#define STILLNESS_END_MS 300 // end gesture after 300ms stillness
#define MIN_GESTURE_MS 750   // reject gestures shorter than 750ms
#define MAX_GESTURE_MS 3000  // hard cap at 3s

// Min and max raw samples for buffer sizing (with some margin for timing jitter)
#define MAX_RAW_GESTURE_SAMPLES (MAX_GESTURE_MS * IMU_RATE_HZ / 1000 * 115 / 100) // 115% of ideal max samples to allow for timing jitter and safety margin
#define MIN_RAW_GESTURE_SAMPLES (MIN_GESTURE_MS * IMU_RATE_HZ / 1000)

// Sanity check: ensure normalize_gesture buffer doesn't exceed stack or cause overflow
_Static_assert(sizeof(GestureSample) * INFERENCE_WINDOW_SIZE <= GESTURE_TASK_STACK / 2, "normalize_gesture buffer may exceed safe stack usage");

// Sanity check: ensure the tick rate is configured to 1000 Hz for accurate timing with vTaskDelayUntil
_Static_assert(configTICK_RATE_HZ == 1000, "CONFIG_FREERTOS_HZ must be 1000 Hz for accurate vTaskDelayUntil timing");

// =========================================================================
// Gesture State Machine
// =========================================================================
static bool gesture_active = false;
static uint64_t gesture_start_ts = 0;
static uint64_t last_motion_ts = 0;

static GestureSample gesture_buf[MAX_RAW_GESTURE_SAMPLES];
static int gesture_len = 0;

// Smoothed motion energy for stillness detection (sensor-frame linear accel magnitude)
static float smoothed_motion_energy = 0.0f;

// =========================================================================
// Normalize and resample IMU gesture to fixed-length window
// - Time-resample each channel to INFERENCE_WINDOW_SIZE samples (uniform time grid)
// - Per-channel zero-mean, unit-variance normalization (using training-set stats later is ideal;
//   here we normalize per-window to stabilize scale across users)
// =========================================================================
static int normalize_gesture(const GestureSample *in, int count, GestureSample *out)
{
    assert(count > 0 && count <= MAX_RAW_GESTURE_SAMPLES);

    // Determine time range
    uint64_t t0 = in[0].timestamp_us;
    uint64_t tN = in[count - 1].timestamp_us;
    assert(tN > t0);

    // Precompute arrays of timestamps for input
    // We'll perform linear interpolation on each channel independently.
    // For quaternion interpolation we use simple normalized linear interpolation (nlerp) for speed.
    // Output timestamps are relative to gesture start (t0)
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        // target timestamp (relative to gesture start)
        float alpha = (float)i / (float)(INFERENCE_WINDOW_SIZE - 1);
        uint64_t abs_target_ts = (uint64_t)lroundf((1.0f - alpha) * (float)t0 + alpha * (float)tN);
        out[i].timestamp_us = (uint32_t)(abs_target_ts - t0);

        // Find source interval [s, s+1] such that in[s].timestamp <= target_ts <= in[s+1].timestamp
        // Safety: ensure s+1 stays within bounds
        int s = 0;
        while (s < count - 1 && s + 1 < count && in[s + 1].timestamp_us < abs_target_ts)
            s++;

        uint64_t ts_s = in[s].timestamp_us;
        uint64_t ts_s1 = in[s + 1].timestamp_us;
        float span = (float)(ts_s1 - ts_s);
        float t = 0.0f;
        if (span > 0.0f)
            t = ((float)abs_target_ts - (float)ts_s) / span;

        // Linear accel
        out[i].ax = lerp(in[s].ax, in[s + 1].ax, t);
        out[i].ay = lerp(in[s].ay, in[s + 1].ay, t);
        out[i].az = lerp(in[s].az, in[s + 1].az, t);

        // Gyro
        out[i].gx = lerp(in[s].gx, in[s + 1].gx, t);
        out[i].gy = lerp(in[s].gy, in[s + 1].gy, t);
        out[i].gz = lerp(in[s].gz, in[s + 1].gz, t);

        // Quaternion: nlerp (normalize after lerp)
        float qw = lerp(in[s].qw, in[s + 1].qw, t);
        float qx = lerp(in[s].qx, in[s + 1].qx, t);
        float qy = lerp(in[s].qy, in[s + 1].qy, t);
        float qz = lerp(in[s].qz, in[s + 1].qz, t);
        float qnorm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
        if (qnorm < 1e-9f)
            qnorm = 1.0f;
        out[i].qw = qw / qnorm;
        out[i].qx = qx / qnorm;
        out[i].qy = qy / qnorm;
        out[i].qz = qz / qnorm;
    }

    // Per-channel normalization: compute mean and std per channel across the window
    // Channels: ax,ay,az,gx,gy,gz. For quaternion we keep normalized quaternions (unit length).
    float mean_ax = 0, mean_ay = 0, mean_az = 0;
    float mean_gx = 0, mean_gy = 0, mean_gz = 0;

    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        mean_ax += out[i].ax;
        mean_ay += out[i].ay;
        mean_az += out[i].az;
        mean_gx += out[i].gx;
        mean_gy += out[i].gy;
        mean_gz += out[i].gz;
    }
    mean_ax /= INFERENCE_WINDOW_SIZE;
    mean_ay /= INFERENCE_WINDOW_SIZE;
    mean_az /= INFERENCE_WINDOW_SIZE;
    mean_gx /= INFERENCE_WINDOW_SIZE;
    mean_gy /= INFERENCE_WINDOW_SIZE;
    mean_gz /= INFERENCE_WINDOW_SIZE;

    float var_ax = 0, var_ay = 0, var_az = 0;
    float var_gx = 0, var_gy = 0, var_gz = 0;
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        float dax = out[i].ax - mean_ax;
        float day = out[i].ay - mean_ay;
        float daz = out[i].az - mean_az;
        float dgx = out[i].gx - mean_gx;
        float dgy = out[i].gy - mean_gy;
        float dgz = out[i].gz - mean_gz;
        var_ax += dax * dax;
        var_ay += day * day;
        var_az += daz * daz;
        var_gx += dgx * dgx;
        var_gy += dgy * dgy;
        var_gz += dgz * dgz;
    }
    var_ax = sqrtf(var_ax / INFERENCE_WINDOW_SIZE);
    var_ay = sqrtf(var_ay / INFERENCE_WINDOW_SIZE);
    var_az = sqrtf(var_az / INFERENCE_WINDOW_SIZE);
    var_gx = sqrtf(var_gx / INFERENCE_WINDOW_SIZE);
    var_gy = sqrtf(var_gy / INFERENCE_WINDOW_SIZE);
    var_gz = sqrtf(var_gz / INFERENCE_WINDOW_SIZE);

    // Avoid division by zero: enforce minimum std
    const float MIN_STD = 1e-3f;
    if (var_ax < MIN_STD)
        var_ax = MIN_STD;
    if (var_ay < MIN_STD)
        var_ay = MIN_STD;
    if (var_az < MIN_STD)
        var_az = MIN_STD;
    if (var_gx < MIN_STD)
        var_gx = MIN_STD;
    if (var_gy < MIN_STD)
        var_gy = MIN_STD;
    if (var_gz < MIN_STD)
        var_gz = MIN_STD;

    // Normalize channels in-place
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        out[i].ax = (out[i].ax - mean_ax) / var_ax;
        out[i].ay = (out[i].ay - mean_ay) / var_ay;
        out[i].az = (out[i].az - mean_az) / var_az;
        out[i].gx = (out[i].gx - mean_gx) / var_gx;
        out[i].gy = (out[i].gy - mean_gy) / var_gy;
        out[i].gz = (out[i].gz - mean_gz) / var_gz;
        // quaternions remain unit-length and are left as-is (they encode orientation)
    }

    return INFERENCE_WINDOW_SIZE;
}

// =========================================================================
// Main Gesture Processing Task
// =========================================================================
static void gesture_task(void *arg)
{
    ESP_LOGI(TAG, "Entering gesture_task");
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_ts = 0;

    for (;;)
    {
        // Yield CPU to allow callbacks to run
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / IMU_RATE_HZ));

        // Compute dt (seconds)
        int64_t now = esp_timer_get_time();
        float dt = 1.0f / IMU_RATE_HZ;
        if (last_ts != 0 && now > last_ts)
        {
            dt = (float)(now - last_ts) / 1e6f;
        }
        if (dt <= 0.0f || dt > 0.01f)
        {
            ESP_LOGW(TAG, "Anomalous dt: %.6f s (expected ~0.005s at 200Hz)", dt);
        }
        last_ts = now;
        
        //
        // Read fused quaternion, linear acceleration (sensor frame), and gyroscope
        //
        bno08x_quat_t game_rv = imu.rpt.rv_game.get_quat();
        bno08x_accel_t lin_accel = imu.rpt.linear_accelerometer.get();
        bno08x_gyro_t gyro = imu.rpt.cal_gyro.get();
        bno08x_stability_classifier_t stability = imu.rpt.stability_classifier.get();

        // Use sensor-frame linear acceleration (BNO08x already provides linear accel with gravity removed)
        float ax = lin_accel.x;
        float ay = lin_accel.y;
        float az = lin_accel.z;

        // Gyroscope (rad/s)
        float gx = gyro.x;
        float gy = gyro.y;
        float gz = gyro.z;

        // Quaternion (game rotation vector)
        float qw = game_rv.real;
        float qx = game_rv.i;
        float qy = game_rv.j;
        float qz = game_rv.k;

        // log stability classifier changes (optional)
        static BNO08xStability last_stability = BNO08xStability::UNDEFINED;
        if (stability.stability != last_stability)
        {
            const char *stability_str;
            switch (stability.stability)
            {
            case BNO08xStability::UNKNOWN:
                stability_str = "UNKNOWN";
                break;
            case BNO08xStability::ON_TABLE:
                stability_str = "ON_TABLE";
                break;
            case BNO08xStability::STATIONARY:
                stability_str = "STATIONARY";
                break;
            case BNO08xStability::STABLE:
                stability_str = "STABLE";
                break;
            case BNO08xStability::MOTION:
                stability_str = "MOTION";
                break;
            case BNO08xStability::RESERVED:
                stability_str = "RESERVED";
                break;
            default:
                stability_str = "UNDEFINED";
            }
            ESP_LOGI(TAG, "Stability changed to: %s", stability_str);
            last_stability = stability.stability;
        }

        // ------------------------------------------
        // Detect when we start to cast a spell
        // ------------------------------------------

        // Compute motion energy from linear acceleration magnitude (sensor frame)
        float accel_mag = sqrtf(ax * ax + ay * ay + az * az);

        // Low-pass filter on motion energy for stability
        smoothed_motion_energy = 0.85f * smoothed_motion_energy + 0.15f * accel_mag;

        bool moving = smoothed_motion_energy > MOVING_START_THRESH;
        if (moving)
        {
            last_motion_ts = now;
        }

        // Detect motion and transition to gesture capture
        if (!gesture_active && moving)
        {
            ESP_LOGI(TAG, "Transition to gesture active (motion detected: %.3f m/s²)", smoothed_motion_energy);
            gesture_active = true;
            gesture_start_ts = now;
            gesture_len = 0;
        }

        // ---------------------
        // Capture the Gesture and detect end of gesture
        // ---------------------
        if (gesture_active)
        {
            // Capture sample: store sensor-frame accel, gyro, quaternion
            if (gesture_len < MAX_RAW_GESTURE_SAMPLES)
            {
                GestureSample sample;
                // Store absolute timestamp (microseconds from boot)
                sample.timestamp_us = now;
                sample.ax = ax;
                sample.ay = ay;
                sample.az = az;
                sample.gx = gx;
                sample.gy = gy;
                sample.gz = gz;
                sample.qw = qw;
                sample.qx = qx;
                sample.qy = qy;
                sample.qz = qz;
                gesture_buf[gesture_len++] = sample;
            }
#ifdef DEBUG
            else
            {
                ESP_LOGW(TAG, "Raw gesture buffer overflow at %d samples", gesture_len);
            }
#endif

            // --------
            // End Gesture Detection
            // --------
            uint32_t gesture_ms = (uint32_t)((now - gesture_start_ts) / 1000);
            bool time_exceeded = (gesture_ms > MAX_GESTURE_MS);
            bool motion_stopped = (smoothed_motion_energy < MOVING_STOP_THRESH) && ((now - last_motion_ts) > STILLNESS_END_MS * 1000);

            // End gesture when motion energy drops or time cap hit
            if (motion_stopped || time_exceeded)
            {
                // Remove trailing stillness samples (by timestamp)
                uint64_t motion_cutoff_us = last_motion_ts + (STILLNESS_END_MS * 1000);
                while (gesture_len > 0 && gesture_buf[gesture_len - 1].timestamp_us > motion_cutoff_us)
                {
                    gesture_len--;
                }

                // Reject too-short gestures
                if (gesture_ms >= MIN_GESTURE_MS && gesture_len >= MIN_RAW_GESTURE_SAMPLES)
                {
                    // Normalize and resample the gesture samples to fixed window
                    GestureSample norm[INFERENCE_WINDOW_SIZE];
                    int n = normalize_gesture(gesture_buf, gesture_len, norm);

                    // Send start-of-gesture marker (timestamp_us = 0, ax = NAN)
                    GestureSample marker;
                    marker.timestamp_us = 0;
                    marker.ax = NAN;
                    marker.ay = NAN;
                    marker.az = NAN;
                    marker.gx = NAN;
                    marker.gy = NAN;
                    marker.gz = NAN;
                    marker.qw = NAN;
                    marker.qx = NAN;
                    marker.qy = NAN;
                    marker.qz = NAN;
                    xQueueSend(g_fusion_queue, &marker, 0);

                    // Send normalized gesture samples
                    for (int i = 0; i < n; i++)
                    {
                        if (xQueueSend(g_fusion_queue, &norm[i], 0) != pdTRUE)
                        {
                            ESP_LOGW(TAG, "fusion_queue full, dropping sample %d of %d", i, n);
                        }
                    }

                    ESP_LOGI(TAG, "Gesture accepted: %d raw → %d normalized samples, %d ms%s",
                             gesture_len, n, gesture_ms, time_exceeded ? " (time-capped)" : "");
                }
                else
                {
                    ESP_LOGI(TAG, "Gesture rejected: %d samples, %d ms (too short or empty)",
                             gesture_len, gesture_ms);
                }

                ESP_LOGI(TAG, "Transition to gesture inactive");
                gesture_active = false;
            }
        }
    }
}

// =========================================================================
// Task Initialization
// =========================================================================
void gesture_task_start(void)
{
    ESP_LOGI(TAG, "Initializing BNO08x IMU...");
    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "BNO08x initialization failed!");
        return;
    }

    // Disable all reports first then re-enable desired reports with correct rates.
    imu.disable_all_reports();

    if (!imu.rpt.rv_game.enable(BNO08X_RATE_GAMERV))
        ESP_LOGE(TAG, "Game Rotation Vector: FAILED");
    
    if (!imu.rpt.linear_accelerometer.enable(BNO08X_RATE_LINACC))
        ESP_LOGE(TAG, "Linear Accelerometer: FAILED");
    
    if (!imu.rpt.cal_gyro.enable(BNO08X_RATE_GYRO))
        ESP_LOGE(TAG, "Calibrated Gyroscope: FAILED");
    
    if (!imu.rpt.stability_classifier.enable(BNO08X_RATE_STABILITY))
        ESP_LOGE(TAG, "Stability Classifier: FAILED");

    // Start gesture processing task
    xTaskCreatePinnedToCore(
        gesture_task,
        TAG,
        GESTURE_TASK_STACK,
        NULL,
        GESTURE_TASK_PRIORITY,
        NULL,
        GESTURE_TASK_CORE);

    ESP_LOGI(TAG, "gesture_task created and running (IMU output)");
}
