#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <math.h>
#include <inttypes.h>
#include "BNO08x.hpp"
#include "queues.h"
#include "data_types.h"
#include "gesture_task.h"

// =========================================================================
// Task Configuration
// =========================================================================
static const char *TAG = "gesture_task";
#define GESTURE_TASK_CORE 1
#define GESTURE_TASK_PRIORITY 8
#define GESTURE_TASK_STACK (16 * 1024)

// =========================================================================
// BNO08x Driver Instance
// =========================================================================
static BNO08x imu;

// =========================================================================
// Report Rates (microseconds)
// =========================================================================
#define BNO08X_RATE_GAMERV 5000UL // 200 Hz
#define BNO08X_RATE_LINACC 5000UL // 200 Hz
#ifdef GRAVITY_DEBUG
#define BNO08X_RATE_GRAVITY 10000UL   // 100 Hz
#endif                                // GRAVITY_DEBUG
#define BNO08X_RATE_STABILITY 20000UL // 50 Hz

// Gesture sampling loop rate (match main IMU rate)
#define IMU_RATE_HZ 200

// =========================================================================
// Gesture Configuration
// =========================================================================
// Thresholds for motion detection based on linear acceleration magnitude
const float STILL_ACCEL_MAX = 0.5f;     // m/s²: very still
const float MOVING_START_THRESH = 2.0f; // m/s²: start gesture
const float MOVING_STOP_THRESH = 1.0f;  // m/s²: stop gesture

// Gesture segmentation timing
#define STILLNESS_END_MS 300 // end gesture after 300ms stillness
#define MIN_GESTURE_MS 750   // reject gestures shorter than 750ms
#define MAX_GESTURE_MS 3000  // hard cap at 3s

// =========================================================================
// Gesture State Machine
// =========================================================================
static bool gesture_active = false;
static uint64_t gesture_start_ts = 0;
static uint64_t last_motion_ts = 0;

static GestureSample gesture_buf[MAX_RAW_GESTURE_SAMPLES];
static int gesture_len = 0;

// Smoothed motion energy for stillness detection
static float smoothed_motion_energy = 0.0f;

// Integration state for position tracking
static float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;
static float vel_x = 0.0f, vel_y = 0.0f, vel_z = 0.0f;

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

// -------------------------
// Rotate vector using quaternion
// -------------------------
static void rotate_vector_by_quaternion(const bno08x_quat_t &q, float vx, float vy, float vz, float &out_x, float &out_y, float &out_z)
{
    // Rotate vector (vx, vy, vz) from sensor frame to world frame using quaternion q
    // Quaternion format: q = real + i*i_hat + j*j_hat + k*k_hat (real is scalar, i/j/k are vector components)
    // Formula: v' = v + 2*qreal*(q_vec × v) + 2*(q_vec × (q_vec × v))

    float qi = q.i, qj = q.j, qk = q.k, qreal = q.real;

    // First cross product: q_vec × v
    float cross1_x = qj * vz - qk * vy;
    float cross1_y = qk * vx - qi * vz;
    float cross1_z = qi * vy - qj * vx;

    // Second cross product: q_vec × (q_vec × v)
    float cross2_x = qj * cross1_z - qk * cross1_y;
    float cross2_y = qk * cross1_x - qi * cross1_z;
    float cross2_z = qi * cross1_y - qj * cross1_x;

    // v' = v + 2*qreal*(q_vec × v) + 2*(q_vec × (q_vec × v))
    out_x = vx + 2.0f * qreal * cross1_x + 2.0f * cross2_x;
    out_y = vy + 2.0f * qreal * cross1_y + 2.0f * cross2_y;
    out_z = vz + 2.0f * qreal * cross1_z + 2.0f * cross2_z;
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
        // Get game rotation vector and use it to rotate linear acceleration to world frame
        //
        bno08x_quat_t game_rv = imu.rpt.rv_game.get_quat();

        // Get linear acceleration for gesture capture and motion detection
        bno08x_accel_t lin_accel = imu.rpt.linear_accelerometer.get();

        // Rotate linear acceleration from sensor frame to world frame
        float ax, ay, az;
        rotate_vector_by_quaternion(game_rv, lin_accel.x, lin_accel.y, lin_accel.z, ax, ay, az);
#ifdef DEBUG
        ESP_LOGV(TAG, "LinAccel: x=%.3f y=%.3f z=%.3f m/s² (mag=%.3f, smooth=%.3f)",
                 ax, ay, az, accel_mag, smoothed_motion_energy);
#endif

        // Compute motion energy from linear acceleration magnitude
        float accel_mag = sqrtf(ax * ax + ay * ay + az * az);

        // Low-pass filter on motion energy for stability
        smoothed_motion_energy = 0.85f * smoothed_motion_energy + 0.15f * accel_mag;

#ifdef GRAVITY_DEBUG
        //
        // get gravity vector for world-frame conversion (not currently used)
        //
        bno08x_accel_t gravity = imu.rpt.gravity.get();
#endif // GRAVITY_DEBUG

        //
        // log stability classifier changes
        //
        static BNO08xStability last_stability = BNO08xStability::UNDEFINED;
        bno08x_stability_classifier_t stability = imu.rpt.stability_classifier.get();
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
        //
        // TODO: Compare this logic to using the stability_classifier report from the IMU,
        // which may already do some of this work for us. For example, we could require
        // stability_class == MOTION or similar.
        //
        // ------------------------------------------
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

            // Reset integration state for fresh gesture
            vel_x = vel_y = vel_z = 0.0f;
            pos_x = pos_y = pos_z = 0.0f;
        }

        // ---------------------
        // Capture the Gesture and detect end of gesture
        // ---------------------
        if (gesture_active)
        {
            // Capture sample: project 3D motion onto X-Z plane
            if (gesture_len < MAX_RAW_GESTURE_SAMPLES)
            {
                // Integrate linear acceleration to get position in world frame
                vel_x += ax * dt;
                vel_y += ay * dt;
                vel_z += az * dt;

                pos_x += vel_x * dt;
                pos_y += vel_y * dt;
                pos_z += vel_z * dt;

                GestureSample sample = {
                    .timestamp_us = now,
                    .x = pos_x,
                    .y = pos_z}; // Use Z (depth) as Y component for 2D gesture
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
            bool motion_stopped = (smoothed_motion_energy < MOVING_STOP_THRESH) &&
                                  ((now - last_motion_ts) > STILLNESS_END_MS * 1000);

            // End gesture when motion energy drops or time cap hit
            if (motion_stopped || time_exceeded)
            {
                // Remove trailing stillness samples
                while (gesture_len > 0 && gesture_buf[gesture_len - 1].timestamp_us > last_motion_ts)
                {
                    gesture_len--;
                }

                // Reject too-short gestures
                if (gesture_ms >= MIN_GESTURE_MS && gesture_len > 0)
                {
                    // Normalize the gesture samples
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
                gesture_len = 0;

                // Reset integration state
                vel_x = vel_y = vel_z = 0.0f;
                pos_x = pos_y = pos_z = 0.0f;
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

    // Initialize IMU
    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "BNO08x initialization failed!");
        return;
    }

    ESP_LOGI(TAG, "BNO08x initialized successfully");
    imu.print_product_ids();

    // Disable all reports first then re-enable desired reports with correct rates.
    imu.disable_all_reports();

    if (!imu.rpt.rv_game.enable(BNO08X_RATE_GAMERV))
    {
        ESP_LOGE(TAG, "Failed to enable Game Rotation Vector");
    }

    if (!imu.rpt.linear_accelerometer.enable(BNO08X_RATE_LINACC))
    {
        ESP_LOGE(TAG, "Failed to enable Linear Acceleration");
    }

#ifdef GRAVITY_DEBUG
    if (!imu.rpt.gravity.enable(BNO08X_RATE_GRAVITY))
    {
        ESP_LOGE(TAG, "Failed to enable Gravity");
    }
#endif // GRAVITY_DEBUG

    if (!imu.rpt.stability_classifier.enable(BNO08X_RATE_STABILITY))
    {
        ESP_LOGE(TAG, "Failed to enable Stability Classifier");
    }

    // Start gesture processing task
    xTaskCreatePinnedToCore(
        gesture_task,
        TAG,
        GESTURE_TASK_STACK,
        NULL,
        GESTURE_TASK_PRIORITY,
        NULL,
        GESTURE_TASK_CORE);

    ESP_LOGI(TAG, "gesture_task created and running");
}
