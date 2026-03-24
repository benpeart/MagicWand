//
// gesture_task.cpp
//
// Event-triggered gesture capture with rolling ring buffer, pre-trigger + post-padding,
// fixed-length resampling to INFERENCE_WINDOW_SAMPLES, per-window normalization, and
// relative-quaternion computation for orientation invariance.
//
// Assumptions:
//  - g_fusion_queue is configured to accept GestureSample objects.
//  - BNO08x driver instance provides calibrated linear accel, gyro, and fused quaternion.
//  - FreeRTOS tick rate is 1000 Hz (configTICK_RATE_HZ == 1000).
//

#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <math.h>
#include <inttypes.h>
#include <float.h>
#include <assert.h>
#include "BNO08x.hpp"
#include "queues.h"
#include "data_types.h"
#include "gesture_task.h"

// Set to 1 to enable raw sensor data logging and validation checks for debugging purposes. Disable for best performance.
// #define DEBUG_SENSOR_DATA 1
#define IMU_RECOVERY 1

// =========================================================================
// Task Configuration
// =========================================================================
static const char *TAG = "gesture_task";
#define GESTURE_TASK_CORE 1
#define GESTURE_TASK_PRIORITY 8 // BNO08x driver tasks are pri (5,6,7)
#define GESTURE_TASK_STACK (12 * 1024)

//_Static_assert(sizeof(GestureSample) * INFERENCE_WINDOW_SAMPLES <= GESTURE_TASK_STACK / 2, "normalize_gesture buffers may exceed safe stack usage");
_Static_assert(configTICK_RATE_HZ == 1000, "CONFIG_FREERTOS_HZ must be 1000 Hz for accurate vTaskDelayUntil timing");

// =========================================================================
// IMU Configuration and motion detection thresholds
// =========================================================================
#define SAMPLE_PERIOD_US (1000000 / IMU_RATE_HZ)

#define MOVING_START_THRESH 2.0f // m/s^2
#define MOVING_STOP_THRESH 1.0f  // m/s^2
#define SMOOTH_ALPHA 0.15f
#define STILLNESS_END_MS 300
#define MIN_GESTURE_MS 750
#define MAX_GESTURE_MS GESTURE_MS

// -------------------------------------------------------------------------
// BNO08x instance
// -------------------------------------------------------------------------
static BNO08x imu;

// -------------------------------------------------------------------------
// Ring buffer types and globals
// -------------------------------------------------------------------------
typedef struct
{
    int64_t timestamp_us; // absolute timestamp from esp_timer_get_time()
    float ax, ay, az;
    float gx, gy, gz;
    float qw, qx, qy, qz;
} RawSample;

#define RING_CAPACITY (INFERENCE_WINDOW_SAMPLES + 200)
static RawSample ring_buffer[RING_CAPACITY];
static int ring_head = 0;  // next write index
static int ring_count = 0; // number of valid samples in buffer

// -------------------------------------------------------------------------
// Ring buffer helpers
// -------------------------------------------------------------------------
static inline void ring_push_sample(const RawSample *s)
{
    ring_buffer[ring_head] = *s;
    ring_head = (ring_head + 1) % RING_CAPACITY;
    if (ring_count < RING_CAPACITY)
        ring_count++;
}

static inline RawSample ring_get_sample_at(int idx)
{
    idx %= RING_CAPACITY;
    if (idx < 0)
        idx += RING_CAPACITY;
    return ring_buffer[idx];
}

static inline int ring_index_of_last_sample()
{
    int idx = ring_head - 1;
    if (idx < 0)
        idx += RING_CAPACITY;
    return idx;
}

// -------------------------------------------------------------------------
// Quaternion helpers
// -------------------------------------------------------------------------
static inline float quat_dot(float a_w, float a_x, float a_y, float a_z,
                             float b_w, float b_x, float b_y, float b_z)
{
    return a_w * b_w + a_x * b_x + a_y * b_y + a_z * b_z;
}

static inline void quat_normalize_inplace(float *w, float *x, float *y, float *z)
{
    float n = sqrtf((*w) * (*w) + (*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (n < 1e-9f)
        n = 1.0f;
    *w /= n;
    *x /= n;
    *y /= n;
    *z /= n;
}

// NLERP (lerp then normalize). Assumes sign flip handled by caller if needed.
static inline void quat_nlerp(float a_w, float a_x, float a_y, float a_z,
                              float b_w, float b_x, float b_y, float b_z,
                              float t,
                              float *out_w, float *out_x, float *out_y, float *out_z)
{
    *out_w = lerp(a_w, b_w, t);
    *out_x = lerp(a_x, b_x, t);
    *out_y = lerp(a_y, b_y, t);
    *out_z = lerp(a_z, b_z, t);
    quat_normalize_inplace(out_w, out_x, out_y, out_z);
}

// SLERP with numeric guards; falls back to NLERP for very small angles or near-parallel quaternions.
static inline void quat_slerp(float a_w, float a_x, float a_y, float a_z,
                              float b_w, float b_x, float b_y, float b_z,
                              float t,
                              float *out_w, float *out_x, float *out_y, float *out_z)
{
    float dot = quat_dot(a_w, a_x, a_y, a_z, b_w, b_x, b_y, b_z);
    if (dot > 1.0f)
        dot = 1.0f;
    if (dot < -1.0f)
        dot = -1.0f;

    const float DOT_THRESHOLD = 0.9995f;
    if (fabsf(dot) > DOT_THRESHOLD)
    {
        quat_nlerp(a_w, a_x, a_y, a_z, b_w, b_x, b_y, b_z, t, out_w, out_x, out_y, out_z);
        return;
    }

    float theta_0 = acosf(dot);
    float theta = theta_0 * t;
    float sin_theta = sinf(theta);
    float sin_theta_0 = sinf(theta_0);

    if (fabsf(sin_theta_0) < 1e-9f)
    {
        quat_nlerp(a_w, a_x, a_y, a_z, b_w, b_x, b_y, b_z, t, out_w, out_x, out_y, out_z);
        return;
    }

    float s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    *out_w = (s0 * a_w) + (s1 * b_w);
    *out_x = (s0 * a_x) + (s1 * b_x);
    *out_y = (s0 * a_y) + (s1 * b_y);
    *out_z = (s0 * a_z) + (s1 * b_z);
    quat_normalize_inplace(out_w, out_x, out_y, out_z);
}

// Rotate vector v by quaternion q (q must be unit-length). v_out = q * v * q^{-1}
static inline void quat_rotate_vector(float q_w, float q_x, float q_y, float q_z,
                                      float vx, float vy, float vz,
                                      float *out_x, float *out_y, float *out_z)
{
    // Quaternion-vector multiplication: t = 2 * cross(q_vec, v)
    float tx = 2.0f * (q_y * vz - q_z * vy);
    float ty = 2.0f * (q_z * vx - q_x * vz);
    float tz = 2.0f * (q_x * vy - q_y * vx);

    // v' = v + q_w * t + cross(q_vec, t)
    float vpx = vx + q_w * tx + (q_y * tz - q_z * ty);
    float vpy = vy + q_w * ty + (q_z * tx - q_x * tz);
    float vpz = vz + q_w * tz + (q_x * ty - q_y * tx);

    *out_x = vpx;
    *out_y = vpy;
    *out_z = vpz;
}

// =========================================================================
// BNO08x Error Detection and Recovery (Helper Functions)
// =========================================================================

/**
 * @brief Configures IMU sensor reports with appropriate data rates.
 *
 * Re-enables the required sensors:
 * - Game Rotation Vector, Linear Accelerometer, Calibrated Gyroscope at SAMPLE_PERIOD_US
 * - Stability Classifier at SAMPLE_PERIOD_US * 4
 */
void set_reports()
{
    /*
        https://cdn.sparkfun.com/assets/2/b/9/0/6/DS-14686-BNO080.pdf

        The maximum available data rates that can be configured per sensor are:

        Composite Sensor            |     Maximum Data rates (Hz)
        Gyro rotation Vector        |     1000
        Rotation Vector             |     400
        Gaming Rotation Vector      |     400
        Geomagnetic Rotation Vector |     90
        Gravity                     |     400
        Linear Acceleration         |     400
        Accelerometer               |     500
        Gyroscope                   |     400
        Magnetometer                |     100
    */
    if (!imu.rpt.rv_game.enable(SAMPLE_PERIOD_US))
        ESP_LOGE(TAG, "Game Rotation Vector: FAILED");

    if (!imu.rpt.linear_accelerometer.enable(SAMPLE_PERIOD_US))
        ESP_LOGE(TAG, "Linear Accelerometer: FAILED");

    if (!imu.rpt.cal_gyro.enable(SAMPLE_PERIOD_US))
        ESP_LOGE(TAG, "Calibrated Gyroscope: FAILED");

    if (!imu.rpt.stability_classifier.enable(SAMPLE_PERIOD_US * 4))
        ESP_LOGE(TAG, "Stability Classifier: FAILED");
}

// =========================================================================
// IMU Recovery State Machine (persists across calls)
// =========================================================================
#ifdef IMU_RECOVERY

/**
 * @brief Recovery stage enumeration for cascading reset strategy.
 */
enum class RecoveryStage
{
    SOFT_RESET, // Stage 1: software reset (3 attempts)
    HARD_RESET, // Stage 2: hardware reset via GPIO (3 attempts)
    EXHAUSTED   // All recovery attempts failed
};

static RecoveryStage recovery_stage = RecoveryStage::SOFT_RESET;
static int recovery_attempts_in_stage = 0;
static constexpr int RECOVERY_MAX_ATTEMPTS_PER_STAGE = 3;

/**
 * @brief Reset IMU recovery state machine to initial state.
 * Called when communication resumes to prepare state machine for fresh attempts if needed.
 */
static void reset_imu_recovery_state()
{
    recovery_stage = RecoveryStage::SOFT_RESET;
    recovery_attempts_in_stage = 0;
    ESP_LOGI(TAG, "Recovery state machine reset to SOFT_RESET (data flow resumed)");
}

/**
 * @brief Attempt IMU recovery using cascading reset strategy state machine.
 *
 * Each stage gets up to 3 attempts before escalating to the next stage.
 * Cascade: SOFT_RESET (3x) → HARD_RESET (3x) → EXHAUSTED
 *
 * NOTE: This function does NOT claim success. The return value only indicates
 * whether we're still attempting recovery or have exhausted all attempts.
 * ACTUAL success is determined by checking if data flow resumes.
 * The recovery state machine will be reset when data is detected.
 *
 * @return true if recovery attempts exhausted (complete failure), false if still attempting
 */
static bool attempt_imu_recovery()
{
    // Nothing I've tried helps (soft reset, hard reset, reconfiguring reports, etc).
    // Just restart to attempt recovery from a clean slate. Even this only works sometimes.
    ESP_LOGE(TAG, "Recovery already exhausted. Restarting system...");
    esp_restart();

#ifdef NEVER
        // Log prior reset reason for diagnostics (first attempt only per recovery session)
    if (recovery_attempts_in_stage == 0)
    {
        BNO08xResetReason reason = imu.get_reset_reason();
        const char *reason_str = "UNKNOWN";
        switch (reason)
        {
        case BNO08xResetReason::UNDEFINED:
            reason_str = "UNDEFINED";
            break;
        case BNO08xResetReason::POR:
            reason_str = "POR (Power On Reset)";
            break;
        case BNO08xResetReason::INT_RST:
            reason_str = "INT_RST (Internal Reset)";
            break;
        case BNO08xResetReason::WTD:
            reason_str = "WTD (Watchdog Timer)";
            break;
        case BNO08xResetReason::EXT_RST:
            reason_str = "EXT_RST (External Reset)";
            break;
        case BNO08xResetReason::OTHER:
            reason_str = "OTHER";
            break;
        }
        ESP_LOGW(TAG, "Recovery initiated - Prior reset reason: %s", reason_str);
    }

    // Execute current recovery stage
    switch (recovery_stage)
    {
    // -----------------------------------------------
    // Stage 1: Soft reset (fastest recovery method)
    // -----------------------------------------------
    case RecoveryStage::SOFT_RESET:
    {
        recovery_attempts_in_stage++;
        ESP_LOGI(TAG, "Attempting soft reset (%d/%d)...", recovery_attempts_in_stage, RECOVERY_MAX_ATTEMPTS_PER_STAGE);

        // Attempt soft reset, but DO NOT trust the return value as true success.
        // Only the resumption of actual data flow proves this worked.
        imu.soft_reset();
        set_reports();
        // Log attempt but don't assume success
        ESP_LOGW(TAG, "Soft reset attempt sent (true recovery verified only on data resumption)");

        // Check if we've exhausted attempts in this stage
        if (recovery_attempts_in_stage >= RECOVERY_MAX_ATTEMPTS_PER_STAGE)
        {
            ESP_LOGW(TAG, "Soft reset exhausted (%d attempts). Escalating to hard reset...", RECOVERY_MAX_ATTEMPTS_PER_STAGE);
            recovery_stage = RecoveryStage::HARD_RESET;
            recovery_attempts_in_stage = 0;
        }
        return false; // Recovery attempt made, keep monitoring for data
    }

    // -----------------------------------------------
    // Stage 2: Hard reset via GPIO
    // -----------------------------------------------
    case RecoveryStage::HARD_RESET:
    {
        recovery_attempts_in_stage++;
        ESP_LOGI(TAG, "Attempting hard reset (%d/%d)...", recovery_attempts_in_stage, RECOVERY_MAX_ATTEMPTS_PER_STAGE);

        // Attempt hard reset, but DO NOT trust the return value as true success.
        imu.hard_reset();
        set_reports();
        ESP_LOGW(TAG, "Hard reset attempt sent (true recovery verified only on data resumption)");

        // Check if we've exhausted attempts in this stage
        if (recovery_attempts_in_stage >= RECOVERY_MAX_ATTEMPTS_PER_STAGE)
        {
            ESP_LOGE(TAG, "All recovery stages exhausted! (%d soft + %d hard attempts). Recovery completely failed.",
                     RECOVERY_MAX_ATTEMPTS_PER_STAGE,
                     RECOVERY_MAX_ATTEMPTS_PER_STAGE);
            recovery_stage = RecoveryStage::EXHAUSTED;
            recovery_attempts_in_stage = 0;
            return true; // Signal: all recovery attempts exhausted
        }
        return false; // Recovery attempt made, keep monitoring for data
    }

    // -----------------------------------------------
    // Stage 3: Exhausted - all recovery attempts failed
    // -----------------------------------------------
    case RecoveryStage::EXHAUSTED:
    {
        // We are already in exhausted state, so just restart to attempt recovery from a clean slate.
        ESP_LOGE(TAG, "Recovery already exhausted. Restarting system...");
        esp_restart();
        return true; // Signal: all recovery attempts exhausted
    }
    }
#endif // NEVER
    return false; // Should not reach here
}

/**
 * @brief Detect HINT/communication loss by checking if has_new_data() returns false.
 * Monitors for complete communication stall across all reports over consecutive windows.
 * Attempts recovery if stall is detected. Resets recovery state when data resumes.
 * @return true if any report has new data, false otherwise
 */
static bool stall_detection_and_recovery()
{
    // Track consecutive windows with zero data from any report
    static int consecutive_no_data = 0;
    static constexpr int NO_DATA_THRESHOLD = 2; // Require 2+ windows to confirm stall

    // Check if ANY report has new data this cycle
    bool any_data = imu.rpt.rv_game.has_new_data() ||
                    imu.rpt.linear_accelerometer.has_new_data() ||
                    imu.rpt.cal_gyro.has_new_data();

    if (any_data)
    {
        // At least one report has new data - communication OK
        if (consecutive_no_data > 0)
        {
            // Data has resumed after stall - reset recovery state machine
            reset_imu_recovery_state();
        }
        consecutive_no_data = 0;
        return true;
    }

    // No new data from any report - HINT may not be asserting
    consecutive_no_data++;

    if (consecutive_no_data < NO_DATA_THRESHOLD)
        return false; // Wait for confirmation across multiple windows

    // Confirmed: No new data for multiple consecutive windows
    ESP_LOGE(TAG, "HINT communication loss detected! No new data for %d consecutive windows", consecutive_no_data);
    attempt_imu_recovery();

    // Do NOT reset consecutive_no_data after exhaustion so we don't keep re-triggering recovery
    if (recovery_stage != RecoveryStage::EXHAUSTED)
    {
        consecutive_no_data = 0;
    }

    return false;
}
#endif // IMU_RECOVERY

/**
 * @brief Check calibration status and automatically enable/disable dynamic calibration.
 * @param game_rv Game rotation vector report containing accuracy information
 */
static void check_and_update_calibration(const bno08x_quat_t &game_rv)
{
    // Orientation confidence (what you'd use to decide gesture quality)
    float orientation_uncertainty_radians = game_rv.rad_accuracy;
    bool confident_reading = (game_rv.rad_accuracy < 0.05f); // < ~3 degrees

    // Calibration status (what you need for "should I calibrate?")
    // Flag as needing calibration if either accuracy enum is low OR orientation uncertainty is high
    bool needs_calibration = (game_rv.accuracy == BNO08xAccuracy::LOW ||
                              game_rv.accuracy == BNO08xAccuracy::UNRELIABLE ||
                              !confident_reading);

    // Auto-enable/disable dynamic calibration based on current needs
    static bool calibration_active = false;
    if (needs_calibration && !calibration_active)
    {
        ESP_LOGW(TAG, "Calibration needed: %s, uncertainty: %.3f rad - ENABLING automatic calibration",
                 BNO08xAccuracy_to_str(game_rv.accuracy), orientation_uncertainty_radians);
        imu.dynamic_calibration_enable(BNO08xCalSel::all);
        calibration_active = true;
    }
    else if (!needs_calibration && calibration_active)
    {
        ESP_LOGI(TAG, "Calibration complete: %s, uncertainty: %.3f rad - DISABLING automatic calibration",
                 BNO08xAccuracy_to_str(game_rv.accuracy), orientation_uncertainty_radians);
        imu.dynamic_calibration_disable(BNO08xCalSel::all);
        calibration_active = false;
    }
}

// -------------------------------------------------------------------------
// normalize_gesture: read samples from ring buffer, resample to INFERENCE_WINDOW_SAMPLES,
// compute relative quaternion, rotate accel/gyro into start frame (orientation-agnostic),
// and per-channel normalization.
// Input: start_idx = ring buffer start index, count = number of samples to read.
// Output: 'out' array of INFERENCE_WINDOW_SAMPLES GestureSample entries.
// Returns INFERENCE_WINDOW_SAMPLES on success, 0 on failure.
// -------------------------------------------------------------------------
static int normalize_gesture(int start_idx, int count, GestureSample *out)
{
    // Require at least two samples to interpolate
    if (!(count >= 2 && count <= RING_CAPACITY))
    {
#ifdef DEBUG_SENSOR_DATA
        ESP_LOGW(TAG, "normalize_gesture FAILED: invalid sample count=%d (need 2-%d)", count, RING_CAPACITY);
#endif
        return 0;
    }

    // Validate we have enough samples in the ring buffer
    if (count > ring_count)
    {
#ifdef DEBUG_SENSOR_DATA
        ESP_LOGW(TAG, "normalize_gesture FAILED: insufficient samples in ring (have %d, requested %d)", ring_count, count);
#endif
        return 0;
    }

    // Get first and last samples from ring buffer
    RawSample first_sample = ring_get_sample_at(start_idx);
    RawSample last_sample = ring_get_sample_at(start_idx + count - 1);

    int64_t t0 = first_sample.timestamp_us;
    int64_t tN = last_sample.timestamp_us;
    if (tN <= t0)
    {
#ifdef DEBUG_SENSOR_DATA
        ESP_LOGW(TAG, "normalize_gesture FAILED: invalid timestamps t0=%" PRId64 " tN=%" PRId64, t0, tN);
#endif
        return 0;
    }

    int64_t duration = (int64_t)INFERENCE_WINDOW_MS * 1000; // Use fixed window duration instead of actual captured span
    if (duration == 0)
    {
#ifdef DEBUG_SENSOR_DATA
        ESP_LOGW(TAG, "normalize_gesture FAILED: zero duration (t0=tN=%" PRId64 ")", t0);
#endif
        return 0;
    }

    // Calculate captured duration to map fixed window into captured time range
    // This ensures all 800 output samples interpolate within actual captured data
    int64_t captured_duration = tN - t0;

    // Resample to INFERENCE_WINDOW_SAMPLES using linear interpolation for accel/gyro
    // and quaternion interpolation (shortest-path sign flip + NLERP/SLERP).
    int s = 0; // running source index pointer (improves complexity to O(M+N))
    for (int i = 0; i < INFERENCE_WINDOW_SAMPLES; i++)
    {
        double alpha = (double)i / (double)(INFERENCE_WINDOW_SAMPLES - 1);
        // Map fixed output window back into captured time range (unstretch)
        int64_t captured_target_ts = t0 + (int64_t)llround(alpha * (double)captured_duration);
        int64_t rel_ts = (int64_t)llround(alpha * (double)duration);
        out[i].timestamp_ms = (uint32_t)(rel_ts / 1000); // Output timestamp relative to gesture start, in milliseconds

        // Find the bracketing samples in captured time
        while (s < count - 2)
        {
            RawSample s_next = ring_get_sample_at(start_idx + s + 1);
            if (s_next.timestamp_us < captured_target_ts)
                s++;
            else
                break;
        }
        if (s >= count - 1)
            s = count - 2;

        // Get the two bracketing samples
        RawSample s_sample = ring_get_sample_at(start_idx + s);
        RawSample s1_sample = ring_get_sample_at(start_idx + s + 1);

        int64_t ts_s = s_sample.timestamp_us;
        int64_t ts_s1 = s1_sample.timestamp_us;
        double span = (double)(ts_s1 - ts_s);
        double t = 0.0;
        if (span > 0.0)
            t = ((double)captured_target_ts - (double)ts_s) / span;
        if (t < 0.0)
            t = 0.0;
        if (t > 1.0)
            t = 1.0;

        // Linear accel interpolation
        out[i].ax = lerp(s_sample.ax, s1_sample.ax, (float)t);
        out[i].ay = lerp(s_sample.ay, s1_sample.ay, (float)t);
        out[i].az = lerp(s_sample.az, s1_sample.az, (float)t);

        // Linear gyro interpolation
        out[i].gx = lerp(s_sample.gx, s1_sample.gx, (float)t);
        out[i].gy = lerp(s_sample.gy, s1_sample.gy, (float)t);
        out[i].gz = lerp(s_sample.gz, s1_sample.gz, (float)t);

        // Quaternion interpolation with shortest-path sign correction
        float a_w = s_sample.qw, a_x = s_sample.qx, a_y = s_sample.qy, a_z = s_sample.qz;
        float b_w = s1_sample.qw, b_x = s1_sample.qx, b_y = s1_sample.qy, b_z = s1_sample.qz;

        float dot = quat_dot(a_w, a_x, a_y, a_z, b_w, b_x, b_y, b_z);
        if (dot < 0.0f)
        {
            b_w = -b_w;
            b_x = -b_x;
            b_y = -b_y;
            b_z = -b_z;
            dot = -dot;
        }

        const float SLERP_THRESHOLD = 0.999f;
        float out_w, out_x, out_y, out_z;
        if (dot < SLERP_THRESHOLD)
        {
            quat_slerp(a_w, a_x, a_y, a_z, b_w, b_x, b_y, b_z, (float)t, &out_w, &out_x, &out_y, &out_z);
        }
        else
        {
            quat_nlerp(a_w, a_x, a_y, a_z, b_w, b_x, b_y, b_z, (float)t, &out_w, &out_x, &out_y, &out_z);
        }

        out[i].qw = out_w;
        out[i].qx = out_x;
        out[i].qy = out_y;
        out[i].qz = out_z;
    }

    // Compute relative quaternions q_rel = q * q0^{-1} to remove absolute orientation
    float iw = out[0].qw, ix = -out[0].qx, iy = -out[0].qy, iz = -out[0].qz;
    for (int i = 0; i < INFERENCE_WINDOW_SAMPLES; i++)
    {
        float qw = out[i].qw, qx = out[i].qx, qy = out[i].qy, qz = out[i].qz;
        float rw = qw * iw - qx * ix - qy * iy - qz * iz;
        float rx = qw * ix + qx * iw + qy * iz - qz * iy;
        float ry = qw * iy - qx * iz + qy * iw + qz * ix;
        float rz = qw * iz + qx * iy - qy * ix + qz * iw;
        float rnorm = sqrtf(rw * rw + rx * rx + ry * ry + rz * rz);
        if (rnorm < 1e-9f)
            rnorm = 1.0f;
        out[i].qw = rw / rnorm;
        out[i].qx = rx / rnorm;
        out[i].qy = ry / rnorm;
        out[i].qz = rz / rnorm;
    }

    // --- NEW: rotate accel and gyro into the start (reference) frame using q_rel ---
    // After this step, accel/gyro are expressed in the same canonical frame (start frame),
    // making the linear/angular signals orientation-agnostic across different initial device poses.
    for (int i = 0; i < INFERENCE_WINDOW_SAMPLES; i++)
    {
        float qrw = out[i].qw, qrx = out[i].qx, qry = out[i].qy, qrz = out[i].qz;

        // Rotate accel into start frame
        float ax_s, ay_s, az_s;
        quat_rotate_vector(qrw, qrx, qry, qrz, out[i].ax, out[i].ay, out[i].az, &ax_s, &ay_s, &az_s);

        // Rotate gyro into start frame
        float gx_s, gy_s, gz_s;
        quat_rotate_vector(qrw, qrx, qry, qrz, out[i].gx, out[i].gy, out[i].gz, &gx_s, &gy_s, &gz_s);

        // Replace sensor-frame values with start-frame values
        out[i].ax = ax_s;
        out[i].ay = ay_s;
        out[i].az = az_s;
        out[i].gx = gx_s;
        out[i].gy = gy_s;
        out[i].gz = gz_s;
    }

    // Per-window normalization for accel and gyro (zero-mean, unit-variance)
    // NOTE: This preserves the original event-triggered behavior. If you want to
    // preserve gravity or use global normalization, change this to use stored
    // training mean/std constants and ensure training uses the same pipeline.
    float mean_ax = 0, mean_ay = 0, mean_az = 0;
    float mean_gx = 0, mean_gy = 0, mean_gz = 0;
    for (int i = 0; i < INFERENCE_WINDOW_SAMPLES; i++)
    {
        mean_ax += out[i].ax;
        mean_ay += out[i].ay;
        mean_az += out[i].az;
        mean_gx += out[i].gx;
        mean_gy += out[i].gy;
        mean_gz += out[i].gz;
    }
    mean_ax /= (float)INFERENCE_WINDOW_SAMPLES;
    mean_ay /= (float)INFERENCE_WINDOW_SAMPLES;
    mean_az /= (float)INFERENCE_WINDOW_SAMPLES;
    mean_gx /= (float)INFERENCE_WINDOW_SAMPLES;
    mean_gy /= (float)INFERENCE_WINDOW_SAMPLES;
    mean_gz /= (float)INFERENCE_WINDOW_SAMPLES;

    float var_ax = 0, var_ay = 0, var_az = 0;
    float var_gx = 0, var_gy = 0, var_gz = 0;
    for (int i = 0; i < INFERENCE_WINDOW_SAMPLES; i++)
    {
        float dax = out[i].ax - mean_ax;
        var_ax += dax * dax;
        float day = out[i].ay - mean_ay;
        var_ay += day * day;
        float daz = out[i].az - mean_az;
        var_az += daz * daz;
        float dgx = out[i].gx - mean_gx;
        var_gx += dgx * dgx;
        float dgy = out[i].gy - mean_gy;
        var_gy += dgy * dgy;
        float dgz = out[i].gz - mean_gz;
        var_gz += dgz * dgz;
    }

    // Population standard deviation (consistent with original code)
    var_ax = sqrtf(var_ax / (float)INFERENCE_WINDOW_SAMPLES);
    var_ay = sqrtf(var_ay / (float)INFERENCE_WINDOW_SAMPLES);
    var_az = sqrtf(var_az / (float)INFERENCE_WINDOW_SAMPLES);
    var_gx = sqrtf(var_gx / (float)INFERENCE_WINDOW_SAMPLES);
    var_gy = sqrtf(var_gy / (float)INFERENCE_WINDOW_SAMPLES);
    var_gz = sqrtf(var_gz / (float)INFERENCE_WINDOW_SAMPLES);

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

    for (int i = 0; i < INFERENCE_WINDOW_SAMPLES; i++)
    {
        out[i].ax = (out[i].ax - mean_ax) / var_ax;
        out[i].ay = (out[i].ay - mean_ay) / var_ay;
        out[i].az = (out[i].az - mean_az) / var_az;
        out[i].gx = (out[i].gx - mean_gx) / var_gx;
        out[i].gy = (out[i].gy - mean_gy) / var_gy;
        out[i].gz = (out[i].gz - mean_gz) / var_gz;
        // quaternions remain unit-length and are left as-is (they encode relative orientation)
    }

#ifdef DEBUG_SENSOR_DATA
    ESP_LOGD(TAG, "normalize_gesture SUCCESS: processed %d samples over duration %" PRId64 " µs", count, duration);
#endif

    return INFERENCE_WINDOW_SAMPLES;
}

// -------------------------------------------------------------------------
// Gesture state machine
// -------------------------------------------------------------------------
enum class GestureState
{
    IDLE,      // waiting for motion to start
    ACTIVE,    // motion detected, collecting samples
    COMPLETING // motion stopped, collecting final samples before processing
};

// -------------------------------------------------------------------------
// Main gesture processing task
// - Always push samples into ring buffer.
// - Use detector to mark trigger index (pre-trigger).
// - On gesture end, wait up to POST_PADDING_MS for post gesture samples, then assemble
//   a normalized gesture window starting at trigger_index - PRE_TRIGGER_SAMPLES.
// -------------------------------------------------------------------------
static void gesture_task(void *arg)
{
    ESP_LOGI(TAG, "Entering gesture_task (window=%d milliseconds, %d ms at %d Hz)", INFERENCE_WINDOW_SAMPLES, INFERENCE_WINDOW_MS, IMU_RATE_HZ);

    GestureState state = GestureState::IDLE;
    int64_t gesture_start_ts = 0;
    int64_t gesture_complete_deadline = 0;
    int64_t last_motion_ts = 0;
    float smoothed_motion_energy = 0.0f;

    int trigger_ring_index = -1; // absolute ring index of trigger sample
    int samples_since_trigger = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / IMU_RATE_HZ));
        int64_t now = esp_timer_get_time();

#if DEBUG_SENSOR_DATA
        // Track actual update frequency for each report type
        static int64_t freq_window_start = 0;
        static int rv_update_count = 0;
        static int la_update_count = 0;
        static int cg_update_count = 0;
        static int sc_update_count = 0;
        static int sample_count = 0;

        if (freq_window_start == 0)
            freq_window_start = now;

        // Count updates when new data arrives
        if (imu.rpt.rv_game.has_new_data())
            rv_update_count++;
        if (imu.rpt.linear_accelerometer.has_new_data())
            la_update_count++;
        if (imu.rpt.cal_gyro.has_new_data())
            cg_update_count++;
        if (imu.rpt.stability_classifier.has_new_data())
            sc_update_count++;

        sample_count++;

        // Log frequency every ~1 second
        int64_t window_elapsed = now - freq_window_start;
        if (window_elapsed >= 1000000) // ~1 second in microseconds
        {
            int rv_freq = (rv_update_count * 1000000) / window_elapsed;
            int la_freq = (la_update_count * 1000000) / window_elapsed;
            int cg_freq = (cg_update_count * 1000000) / window_elapsed;
            int sc_freq = (sc_update_count * 1000000) / window_elapsed;
            ESP_LOGI(TAG, "Actual update frequency (%.2f s) - RV: %d Hz | LA: %d Hz | CG: %d Hz | SC: %d Hz",
                     window_elapsed / 1000000.0f, rv_freq, la_freq, cg_freq, sc_freq);

            // Reset counters
            freq_window_start = now;
            rv_update_count = 0;
            la_update_count = 0;
            cg_update_count = 0;
            sc_update_count = 0;
            sample_count = 0;
        }
#else
#ifdef IMU_RECOVERY
        // Check for communication loss via has_new_data() stall detection and attempt recovery if needed
        // Only do this when we are not tracking frequency tracking as the call to
        // has_new_data() resets the internal "new data" flag.
        stall_detection_and_recovery();
#endif // IMU_RECOVERY
#endif // DEBUG_SENSOR_DATA

        // Always read reports, even if some are stale
        // Sample rate consistency matters more than perfect freshness
        bno08x_quat_t game_rv = imu.rpt.rv_game.get_quat();
        bno08x_accel_t lin_accel = imu.rpt.linear_accelerometer.get();
        bno08x_gyro_t gyro = imu.rpt.cal_gyro.get();

        RawSample s;
        s.timestamp_us = now;
        s.ax = lin_accel.x;
        s.ay = lin_accel.y;
        s.az = lin_accel.z;
        s.gx = gyro.x;
        s.gy = gyro.y;
        s.gz = gyro.z;
        s.qw = game_rv.real;
        s.qx = game_rv.i;
        s.qy = game_rv.j;
        s.qz = game_rv.k;

        // Push into ring buffer
        ring_push_sample(&s);

        // Check and update calibration status
        check_and_update_calibration(game_rv);

#ifdef DEBUG_SENSOR_DATA
        // Validate data bounds to catch sensor errors/glitches
        const float ACCEL_MAX = 30.0f; // m/s² (typical air gesture ~5-30 m/s², impacts >50 m/s²)
        const float GYRO_MAX = 500.0f; // deg/s
        const float QUAT_MIN = 0.9f;   // Unit quaternion magnitude should be ~1.0
        const float QUAT_MAX = 1.1f;

        float quat_mag = sqrtf(game_rv.real * game_rv.real + game_rv.i * game_rv.i +
                               game_rv.j * game_rv.j + game_rv.k * game_rv.k);

        bool accel_valid = (fabsf(lin_accel.x) < ACCEL_MAX) &&
                           (fabsf(lin_accel.y) < ACCEL_MAX) &&
                           (fabsf(lin_accel.z) < ACCEL_MAX);

        bool gyro_valid = (fabsf(gyro.x) < GYRO_MAX) &&
                          (fabsf(gyro.y) < GYRO_MAX) &&
                          (fabsf(gyro.z) < GYRO_MAX);

        bool quat_valid = (quat_mag >= QUAT_MIN) && (quat_mag <= QUAT_MAX);

        if (!accel_valid || !gyro_valid || !quat_valid)
        {
            ESP_LOGW(TAG, "Invalid sensor data: accel_ok=%s gyro_ok=%s quat_ok=%s (mag=%.4f)",
                     accel_valid ? "true" : "false",
                     gyro_valid ? "true" : "false",
                     quat_valid ? "true" : "false",
                     quat_mag);
        }
#endif // DEBUG_SENSOR_DATA

        // log stability classifier changes (I intend to use this to do power management in the future, but for now it’s just informational)
#ifndef POWER_MANAGEMENT
        static BNO08xStability last_stability = BNO08xStability::UNDEFINED;
        bno08x_stability_classifier_t stability = imu.rpt.stability_classifier.get();
        if (stability.stability != last_stability)
        {
            ESP_LOGI(TAG, "Stability changed to: %s", BNO08xStability_to_str(stability.stability));
            last_stability = stability.stability;
        }
#endif // POWER_MANAGEMENT

        // ------------------------------------------
        // Detect when we start to cast a spell
        // ------------------------------------------

        // Compute motion energy from linear acceleration magnitude (sensor frame)
        float accel_mag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
#ifdef DEBUG_SENSOR_DATA
        if (accel_mag > 50.0)
        {
            ESP_LOGW(TAG, "High accel magnitude: %.3f m/s²", accel_mag);
        }
#endif // DEBUG_SENSOR_DATA

        // Low-pass filter on motion energy for stability
        smoothed_motion_energy = (1.0f - SMOOTH_ALPHA) * smoothed_motion_energy + SMOOTH_ALPHA * accel_mag;

        bool moving = smoothed_motion_energy > MOVING_START_THRESH;
        if (moving)
            last_motion_ts = now;

        // =====================================================================
        // Gesture state machine
        // =====================================================================
        switch (state)
        {
        // -----------------------------------------------
        // IDLE: waiting for motion to start
        // -----------------------------------------------
        case GestureState::IDLE:
        {
            if (moving)
            {
                // Ensure we have enough pre-trigger samples in the ring buffer
                if (ring_count >= PRE_TRIGGER_SAMPLES)
                {
                    ESP_LOGI(TAG, "Transition to ACTIVE (motion detected: %.3f m/s²)", smoothed_motion_energy);
                    state = GestureState::ACTIVE;
                    gesture_start_ts = now;
                    trigger_ring_index = ring_index_of_last_sample();
                    samples_since_trigger = 0;
                }
                else
                {
                    ESP_LOGD(TAG, "Motion detected but insufficient pre-trigger samples: have %d, need %d", ring_count, PRE_TRIGGER_SAMPLES);
                }
            }
            break;
        }

        // -----------------------------------------------
        // ACTIVE: motion detected, collecting samples
        // -----------------------------------------------
        case GestureState::ACTIVE:
        {
            samples_since_trigger++;

            uint32_t gesture_ms = (uint32_t)((now - gesture_start_ts) / 1000);
            bool gesture_long_enough = (gesture_ms >= MIN_GESTURE_MS);
            bool time_exceeded = (gesture_ms > MAX_GESTURE_MS);
            bool motion_stopped = (smoothed_motion_energy < MOVING_STOP_THRESH);
            bool stillness_duration_exceeded = motion_stopped ? ((now - last_motion_ts) > (STILLNESS_END_MS * 1000)) : false;

            if ((stillness_duration_exceeded || time_exceeded) && gesture_long_enough)
            {
                ESP_LOGI(TAG, "Transition to COMPLETING (motion stopped after %d ms)", gesture_ms);
                state = GestureState::COMPLETING;
                gesture_complete_deadline = now + (POST_PADDING_MS * 1000);
            }
            else if (stillness_duration_exceeded && !gesture_long_enough)
            {
                ESP_LOGI(TAG, "Transition to IDLE (Gesture rejected: too short (%d ms, need %d ms))", gesture_ms, MIN_GESTURE_MS);
                state = GestureState::IDLE;
            }
            break;
        }

        // -----------------------------------------------
        // COMPLETING: motion stopped or exceeded max duration, collecting post-padding
        // -----------------------------------------------
        case GestureState::COMPLETING:
        {
            samples_since_trigger++;

            // if we’ve reached the deadline for post-trigger samples, attempt to process the gesture
            if (now >= gesture_complete_deadline)
            {
                // Compute start index = trigger_ring_index - PRE_TRIGGER_SAMPLES
                int start_idx = trigger_ring_index - PRE_TRIGGER_SAMPLES;
                start_idx %= RING_CAPACITY;
                if (start_idx < 0)
                    start_idx += RING_CAPACITY;

                // cap our sample count to the max that normalize_gesture can handle
                // as we may get a few more since our logic is time based not sample-based
                int total_samples = PRE_TRIGGER_SAMPLES + samples_since_trigger;
                if (total_samples > INFERENCE_WINDOW_SAMPLES)
                    total_samples = INFERENCE_WINDOW_SAMPLES;

                // static so we don't exceed stack usage
                static GestureSample norm[INFERENCE_WINDOW_SAMPLES];
                int n = normalize_gesture(start_idx, total_samples, norm);
                if (n == INFERENCE_WINDOW_SAMPLES)
                {
                    // Send normalized gesture samples
                    for (int i = 0; i < n; i++)
                    {
                        if (xQueueSend(g_fusion_queue, &norm[i], 0) != pdTRUE)
                        {
                            vTaskDelay(pdMS_TO_TICKS(10));
                            if (xQueueSend(g_fusion_queue, &norm[i], 0) != pdTRUE)
                            {
                                ESP_LOGW(TAG, "fusion_queue full, dropping sample %d of %d", i, n);
                            }
                        }
                    }
                    uint32_t final_gesture_ms = (uint32_t)((now - gesture_start_ts) / 1000);
                    ESP_LOGI(TAG, "Gesture accepted: %d-sample window (duration=%d ms)", n, final_gesture_ms);
                }
                else
                {
                    ESP_LOGW(TAG, "Normalization failed, dropping gesture");
                }

                // Transition back to IDLE
                ESP_LOGI(TAG, "Transition to IDLE");
                state = GestureState::IDLE;
                smoothed_motion_energy = 0.0f; // reset filter to avoid immediately retriggering on residual motion
            }
            break;
        }
        } // end switch(state)
    }
}

// -------------------------------------------------------------------------
// Task Initialization
// -------------------------------------------------------------------------
void gesture_task_start(void)
{
    ESP_LOGI(TAG, "Initializing BNO08x IMU...");

    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "BNO08x initialization failed!");
        return;
    }

    // Enable dynamic calibration autosave - calibration persists in device's internal flash
    imu.dynamic_calibration_autosave_enable();

    // set desired reports and rates
    imu.disable_all_reports();
    set_reports();

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
