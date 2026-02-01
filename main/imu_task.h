#pragma once

/* --------------------------------------------------------------------------
 * imu_task_start()
 *
 * PURPOSE:
 *   The IMU task is the hardware‑interface and data‑acquisition layer for the
 *   entire gesture‑recognition pipeline. It is responsible for reading raw
 *   accelerometer and gyroscope samples from the IMU at a fixed, deterministic
 *   rate and delivering those samples to the fusion_task() with minimal jitter.
 *
 * WHAT THIS TASK DOES:
 *   1. Initializes and configures the IMU hardware.
 *      Sets sample rate, full‑scale ranges, filters, and interrupt behavior.
 *      Ensures the IMU is producing stable, time‑aligned accel/gyro data.
 *
 *   2. Reads IMU samples at a fixed update interval.
 *      This task runs at the highest timing priority needed to guarantee
 *      consistent sampling. It prevents timing drift, aliasing, and gaps in
 *      the motion data stream.
 *
 *   3. Performs basic sanity checks on raw sensor data.
 *      Detects invalid readings, saturation, communication errors, or
 *      out‑of‑range values. Faults are logged and optionally filtered.
 *
 *   4. Packages each IMU sample into a timestamped structure.
 *      The timestamp is critical for downstream fusion and integration.
 *      All samples are delivered in chronological order.
 *
 *   5. Pushes samples into g_imu_queue to avoid blocking and ensure real‑time 
 *      behavior.
 *
 * WHY THIS DESIGN IS ESSENTIAL:
 *   - IMU data is only useful if sampled consistently. Irregular timing
 *     produces drift, unstable fusion, and distorted gesture paths.
 *   - Separating IMU acquisition from fusion ensures that sensor reads are
 *     never delayed by CPU‑heavy operations like filtering or ML inference.
 *   - Clean, timestamped raw data is the foundation for accurate orientation
 *     estimation, motion integration, and gesture segmentation.
 *
 * RESULT:
 *   A stable, high‑integrity stream of timestamped IMU samples feeding the
 *   fusion_task(), forming the foundation of the entire gesture‑recognition
 *   pipeline.
 * -------------------------------------------------------------------------- */
void imu_task_start(void);
