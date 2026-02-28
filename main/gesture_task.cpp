//
// gesture_task.cpp
//
// Event-triggered gesture capture with rolling ring buffer, pre-trigger + post-padding,
// fixed-length resampling to INFERENCE_WINDOW_SIZE, per-window normalization, and
// relative-quaternion computation for orientation invariance.
//
// Assumptions:
//  - g_fusion_queue is configured to accept GestureSample objects.
//  - BNO08x driver instance provides calibrated linear accel, gyro, and fused quaternion.
//  - FreeRTOS tick rate is 1000 Hz (configTICK_RATE_HZ == 1000).
//

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
#define DEBUG_SENSOR_DATA 1

// =========================================================================
// Task Configuration
// =========================================================================
static const char *TAG = "gesture_task";
#define GESTURE_TASK_CORE 1
#define GESTURE_TASK_PRIORITY 8 // BNO08x driver tasks are pri (5,6,7)
#define GESTURE_TASK_STACK (12 * 1024)

// =========================================================================
// IMU Sampling Configuration
// =========================================================================
#define IMU_RATE_HZ 100
#define SAMPLE_PERIOD_US (1000000 / IMU_RATE_HZ)

#define CLASSIFIER_SAMPLES INFERENCE_WINDOW_SIZE
#define CLASSIFIER_WINDOW_S ((float)CLASSIFIER_SAMPLES / (float)IMU_RATE_HZ)

#define PRE_TRIGGER_MS 500
#define PRE_TRIGGER_SAMPLES ((IMU_RATE_HZ * PRE_TRIGGER_MS) / 1000)
#define POST_PADDING_MS 500
#define POST_PADDING_SAMPLES ((IMU_RATE_HZ * POST_PADDING_MS) / 1000)

#define RING_CAPACITY (CLASSIFIER_SAMPLES + PRE_TRIGGER_SAMPLES + POST_PADDING_SAMPLES + 200)

static const float MOVING_START_THRESH = 2.0f; // m/s^2
static const float MOVING_STOP_THRESH = 1.0f;  // m/s^2
#define STILLNESS_END_MS 300
#define MIN_GESTURE_MS 750
#define MAX_GESTURE_MS 3000

#define SMOOTH_ALPHA 0.15f

//_Static_assert(sizeof(GestureSample) * CLASSIFIER_SAMPLES <= GESTURE_TASK_STACK / 2, "normalize_gesture buffers may exceed safe stack usage");
_Static_assert(configTICK_RATE_HZ == 1000, "CONFIG_FREERTOS_HZ must be 1000 Hz for accurate vTaskDelayUntil timing");

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

static int ring_get_samples(RawSample *out, int start_idx, int n)
{
    if (n > RING_CAPACITY)
        return -1;
    if (n > ring_count)
    {
        return -1;
    }
    int idx = start_idx;
    for (int i = 0; i < n; ++i)
    {
        out[i] = ring_buffer[idx];
        idx++;
        if (idx >= RING_CAPACITY)
            idx = 0;
    }

    return 0;
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

// -------------------------------------------------------------------------
// normalize_gesture: resample to INFERENCE_WINDOW_SIZE, compute relative quaternion,
// rotate accel/gyro into start frame (orientation-agnostic), and per-channel normalization.
// Input: 'in' array of count samples (timestamps must be increasing).
// Output: 'out' array of INFERENCE_WINDOW_SIZE GestureSample entries.
// Returns INFERENCE_WINDOW_SIZE on success, 0 on failure.
// -------------------------------------------------------------------------
static int normalize_gesture(const RawSample *in, int count, GestureSample *out)
{
    // Require at least two samples to interpolate
    if (!(count >= 2 && count <= RING_CAPACITY))
        return 0;

    int64_t t0 = in[0].timestamp_us;
    int64_t tN = in[count - 1].timestamp_us;
    if (tN <= t0)
        return 0;

    int64_t duration = tN - t0;
    if (duration == 0)
        return 0;

    // Resample to INFERENCE_WINDOW_SIZE using linear interpolation for accel/gyro
    // and quaternion interpolation (shortest-path sign flip + NLERP/SLERP).
    int s = 0; // running source index pointer (improves complexity to O(M+N))
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        double alpha = (double)i / (double)(INFERENCE_WINDOW_SIZE - 1);
        int64_t abs_target_ts = t0 + (int64_t)llround(alpha * (double)duration);
        int64_t rel_ts = abs_target_ts - t0;
        out[i].timestamp_us = (uint32_t)rel_ts;

        while (s < count - 2 && in[s + 1].timestamp_us < abs_target_ts)
            s++;
        if (s >= count - 1)
            s = count - 2;

        int64_t ts_s = in[s].timestamp_us;
        int64_t ts_s1 = in[s + 1].timestamp_us;
        double span = (double)(ts_s1 - ts_s);
        double t = 0.0;
        if (span > 0.0)
            t = ((double)abs_target_ts - (double)ts_s) / span;
        if (t < 0.0)
            t = 0.0;
        if (t > 1.0)
            t = 1.0;

        // Linear accel interpolation
        out[i].ax = lerp(in[s].ax, in[s + 1].ax, (float)t);
        out[i].ay = lerp(in[s].ay, in[s + 1].ay, (float)t);
        out[i].az = lerp(in[s].az, in[s + 1].az, (float)t);

        // Linear gyro interpolation
        out[i].gx = lerp(in[s].gx, in[s + 1].gx, (float)t);
        out[i].gy = lerp(in[s].gy, in[s + 1].gy, (float)t);
        out[i].gz = lerp(in[s].gz, in[s + 1].gz, (float)t);

        // Quaternion interpolation with shortest-path sign correction
        float a_w = in[s].qw, a_x = in[s].qx, a_y = in[s].qy, a_z = in[s].qz;
        float b_w = in[s + 1].qw, b_x = in[s + 1].qx, b_y = in[s + 1].qy, b_z = in[s + 1].qz;

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
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
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
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
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
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        mean_ax += out[i].ax;
        mean_ay += out[i].ay;
        mean_az += out[i].az;
        mean_gx += out[i].gx;
        mean_gy += out[i].gy;
        mean_gz += out[i].gz;
    }
    mean_ax /= (float)INFERENCE_WINDOW_SIZE;
    mean_ay /= (float)INFERENCE_WINDOW_SIZE;
    mean_az /= (float)INFERENCE_WINDOW_SIZE;
    mean_gx /= (float)INFERENCE_WINDOW_SIZE;
    mean_gy /= (float)INFERENCE_WINDOW_SIZE;
    mean_gz /= (float)INFERENCE_WINDOW_SIZE;

    float var_ax = 0, var_ay = 0, var_az = 0;
    float var_gx = 0, var_gy = 0, var_gz = 0;
    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
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
    var_ax = sqrtf(var_ax / (float)INFERENCE_WINDOW_SIZE);
    var_ay = sqrtf(var_ay / (float)INFERENCE_WINDOW_SIZE);
    var_az = sqrtf(var_az / (float)INFERENCE_WINDOW_SIZE);
    var_gx = sqrtf(var_gx / (float)INFERENCE_WINDOW_SIZE);
    var_gy = sqrtf(var_gy / (float)INFERENCE_WINDOW_SIZE);
    var_gz = sqrtf(var_gz / (float)INFERENCE_WINDOW_SIZE);

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

    for (int i = 0; i < INFERENCE_WINDOW_SIZE; i++)
    {
        out[i].ax = (out[i].ax - mean_ax) / var_ax;
        out[i].ay = (out[i].ay - mean_ay) / var_ay;
        out[i].az = (out[i].az - mean_az) / var_az;
        out[i].gx = (out[i].gx - mean_gx) / var_gx;
        out[i].gy = (out[i].gy - mean_gy) / var_gy;
        out[i].gz = (out[i].gz - mean_gz) / var_gz;
        // quaternions remain unit-length and are left as-is (they encode relative orientation)
    }

    return INFERENCE_WINDOW_SIZE;
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
// - On gesture end, wait up to POST_PADDING_MS for forward samples, then assemble
//   a CLASSIFIER_SAMPLES window starting at trigger_index - PRE_TRIGGER_SAMPLES.
// -------------------------------------------------------------------------
static void gesture_task(void *arg)
{
    ESP_LOGI(TAG, "Entering gesture_task (window=%d samples, %.3f s at %d Hz)", CLASSIFIER_SAMPLES, CLASSIFIER_WINDOW_S, IMU_RATE_HZ);

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

        sample_count++;

        // Log frequency every ~1 second
        int64_t window_elapsed = now - freq_window_start;
        if (window_elapsed >= 1000000) // ~1 second in microseconds
        {
            float elapsed_s = window_elapsed / 1000000.0f;
            float rv_freq = rv_update_count / elapsed_s;
            float la_freq = la_update_count / elapsed_s;
            float cg_freq = cg_update_count / elapsed_s;
            ESP_LOGI(TAG, "Actual update frequency (%.2f s) - RV: %.1f Hz | LA: %.1f Hz | CG: %.1f Hz",
                     elapsed_s, rv_freq, la_freq, cg_freq);

            // Reset counters
            freq_window_start = now;
            rv_update_count = 0;
            la_update_count = 0;
            cg_update_count = 0;
            sample_count = 0;
        }
#endif // DEBUG_SENSOR_DATA

        // Always read reports, even if some are stale
        // The 200Hz consistency matters more than perfect freshness
        bno08x_quat_t game_rv = imu.rpt.rv_game.get_quat();
        bno08x_accel_t lin_accel = imu.rpt.linear_accelerometer.get();
        bno08x_gyro_t gyro = imu.rpt.cal_gyro.get();
        bno08x_stability_classifier_t stability = imu.rpt.stability_classifier.get();

        // Validate data bounds to catch sensor errors/glitches
#ifdef DEBUG_SENSOR_DATA
        const float ACCEL_MAX = 50.0f; // m/s²
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
            ESP_LOGW(TAG, "Invalid sensor data: accel_ok=%d gyro_ok=%d quat_ok=%d (mag=%.4f)",
                     accel_valid, gyro_valid, quat_valid, quat_mag);
        }
#endif // DEBUG_SENSOR_DATA

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
        float accel_mag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
#ifdef DEBUG_SENSOR_DATA
        if (accel_mag > 30.0)
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
                ESP_LOGI(TAG, "Transition to ACTIVE (motion detected: %.3f m/s²)", smoothed_motion_energy);
                state = GestureState::ACTIVE;
                gesture_start_ts = now;
                trigger_ring_index = ring_index_of_last_sample();
                samples_since_trigger = 0;
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
            bool time_exceeded = (gesture_ms > MAX_GESTURE_MS);
            bool motion_stopped = (smoothed_motion_energy < MOVING_STOP_THRESH);
            bool stillness_duration_exceeded = motion_stopped ? ((now - last_motion_ts) > (STILLNESS_END_MS * 1000)) : false;

            if (stillness_duration_exceeded || time_exceeded)
            {
                ESP_LOGI(TAG, "Transition to COMPLETING (motion stopped after %d ms)", gesture_ms);
                state = GestureState::COMPLETING;
                gesture_complete_deadline = now + (POST_PADDING_MS * 1000);
            }
            break;
        }

        // -----------------------------------------------
        // COMPLETING: motion stopped, collecting post-padding
        // -----------------------------------------------
        case GestureState::COMPLETING:
        {
            samples_since_trigger++;

            int samples_available = samples_since_trigger;
            int required_forward = CLASSIFIER_SAMPLES - PRE_TRIGGER_SAMPLES;

            bool deadline_reached = (now >= gesture_complete_deadline);
            bool enough_samples = (samples_available >= required_forward);

            if (deadline_reached || enough_samples)
            {
                uint32_t final_gesture_ms = (uint32_t)((now - gesture_start_ts) / 1000);

                // Attempt to process gesture if it meets minimum duration criteria
                if (final_gesture_ms >= MIN_GESTURE_MS)
                {
                    // Compute start index = trigger_ring_index - PRE_TRIGGER_SAMPLES
                    int start_idx = trigger_ring_index - PRE_TRIGGER_SAMPLES;
                    start_idx %= RING_CAPACITY;
                    if (start_idx < 0)
                        start_idx += RING_CAPACITY;

                    if (ring_count >= CLASSIFIER_SAMPLES)
                    {
                        // static so we doesn’t exceed stack usage
                        static RawSample window_raw[CLASSIFIER_SAMPLES];
                        if (ring_get_samples(window_raw, start_idx, CLASSIFIER_SAMPLES) == 0)
                        {
                            // static so we doesn’t exceed stack usage
                            static GestureSample norm[INFERENCE_WINDOW_SIZE];
                            int n = normalize_gesture(window_raw, CLASSIFIER_SAMPLES, norm);
                            if (n == INFERENCE_WINDOW_SIZE)
                            {
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
                                if (xQueueSend(g_fusion_queue, &marker, 0) != pdTRUE)
                                {
                                    vTaskDelay(pdMS_TO_TICKS(10));
                                    if (xQueueSend(g_fusion_queue, &marker, 0) != pdTRUE)
                                    {
                                        ESP_LOGW(TAG, "fusion_queue full, dropping gesture");
                                        ESP_LOGI(TAG, "Transition to IDLE");
                                        state = GestureState::IDLE;
                                        continue;
                                    }
                                }

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
                                ESP_LOGI(TAG, "Gesture accepted: %d-sample window (duration=%d ms, post_samples=%d)", n, final_gesture_ms, samples_available);
                            }
                            else
                            {
                                ESP_LOGW(TAG, "Normalization failed, dropping gesture");
                            }
                        }
                        else
                        {
                            ESP_LOGW(TAG, "ring_get_samples failed when assembling window");
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Not enough samples in ring (have %d need %d)", ring_count, CLASSIFIER_SAMPLES);
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Gesture rejected: too short (%d ms) or insufficient samples (%d)", final_gesture_ms, samples_available);
                }

                // Transition back to IDLE
                ESP_LOGI(TAG, "Transition to IDLE");
                state = GestureState::IDLE;
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

    // Disable all reports first then re-enable desired reports with correct rates.
    imu.disable_all_reports();

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

    if (!imu.rpt.stability_classifier.enable(SAMPLE_PERIOD_US / 4))
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
