#pragma once

/* --------------------------------------------------------------------------
 * capture_task()
 *
 * PURPOSE:
 *   The data capture task completed gestures from fusion_task(), and outputs
 *   the samples to the real‑time visualizer and/or the training‑data logger.
 *
 * WHAT THIS TASK DOES:
 *   1. Waits for a completed, normalized gesture from fusion_task() and then
 *      outputs that normalized gesture to the visualizer. This provides 
 *      real‑time debugging that is intuitive and trustworthy.
 *
 *   2. This same output can also be captured to CSV for training data or
 *      fed directly into a training pipeline. 
 *
 * WHY THIS DESIGN IS ESSENTIAL:
 *   - The visualizer and training pipeline both depend on receiving the same
 *     normalized representation that the model uses.
 *
 * RESULT:
 *   A deterministic, clean, fixed‑length, normalized gesture delivered to both
 *   the visualizer and the inference engine, ensuring consistency across
 *   debugging, training, and real‑time recognition.
 * -------------------------------------------------------------------------- */
void capture_task_start(void);
