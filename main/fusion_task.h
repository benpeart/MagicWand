#pragma once

/* --------------------------------------------------------------------------
 * fusion_task_start()
 *
 * PURPOSE:
 *   The fusion task is the real‑time sensor‑processing engine for the wand.
 *   It ingests raw IMU data, performs orientation and motion fusion, detects
 *   gesture start/stop transitions, integrates motion into a 2D path, and
 *   streams clean, time‑aligned samples into the gesture buffer and once
 *   the gesture is complete, it is normalized and added to the g_fusion_queue
 *   for the ML inference for gesture recognition or for the data capture task
 *   to capture the training data for ML.
 *
 * WHAT THIS TASK DOES:
 *   1. Reads IMU samples at a fixed, high‑rate update interval.
 *      Ensures consistent timing and prevents aliasing or jitter in the
 *      downstream gesture pipeline.
 *
 *   2. Runs sensor fusion (accel + gyro) to produce stable orientation and
 *      motion estimates. This removes drift, reduces noise, and provides
 *      a coherent frame of reference for integration.
 *
 *   3. Performs motion segmentation:
 *        - Detects when the wand transitions from still → moving
 *        - Detects when the wand transitions from moving → still
 *      These transitions define the boundaries of a gesture.
 *
 *   4. Integrates fused motion into a continuous gesture path.
 *      Converts IMU‑space motion into a 2D/3D trajectory suitable for
 *      shape‑based recognition. Handles velocity thresholds, drift
 *      suppression, and integration resets at gesture boundaries.
 *
 *   5. Buffers raw gesture samples until the gesture ends.
 *      The buffer contains timestamped (x, y, z) points representing the
 *      user’s actual motion. This buffer is later passed to
 *      normalize_gesture() for arc‑length resampling.
 *
 * WHY THIS DESIGN IS ESSENTIAL:
 *   - IMU data is noisy, high‑frequency, and orientation‑dependent.
 *   - Gesture recognition requires clean, drift‑controlled, time‑aligned
 *     motion data with well‑defined start/stop boundaries.
 *   - The ML model expects a stable geometric path, not raw accelerometer
 *     noise or inconsistent sample counts.
 *   - By isolating fusion, segmentation, and integration into a dedicated
 *     real‑time task, the system guarantees deterministic behavior and
 *     consistent gesture capture regardless of CPU load or timing jitter.
 *
 * RESULT:
 *   A robust, real‑time motion‑capture pipeline that transforms raw IMU
 *   readings into clean, segmented gesture paths ready for ML inference.
 * -------------------------------------------------------------------------- */
void fusion_task_start(void);
