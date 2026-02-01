#pragma once

/* --------------------------------------------------------------------------
 * inference_task()
 *
 * PURPOSE:
 *   The inference task is the final stage of the gesture‑recognition pipeline.
 *   It receives a fully normalized, fixed‑length gesture from the fusion and
 *   normalization stages, runs the TensorFlow Lite Micro model, interprets the
 *   output probabilities, applies confidence logic, and emits a clean,
 *   debounced classification event to the rest of the system.
 *
 * WHAT THIS TASK DOES:
 *   1. Waits for a “gesture ready” event from the fusion/normalization pipeline.
 *      The gesture buffer at this point contains a 256‑sample, arc‑length
 *      resampled, centered, and scaled trajectory suitable for ML inference.
 *
 *   2. Copies the normalized gesture into the model’s input tensor.
 *      Ensures strict shape compliance (1 × 256 × feature_dim) and prevents
 *      race conditions by isolating all model access to this task.
 *
 *   3. Invokes the TensorFlow Lite Micro interpreter.
 *      Runs the model using the MicroMutableOpResolver and the statically
 *      allocated arena. This task owns the interpreter to guarantee that
 *      inference is deterministic and never preempted mid‑execution.
 *
 *   4. Reads the output probability vector.
 *      The model produces a softmax distribution over all gesture classes.
 *      The task extracts the top‑1 class and its associated confidence.
 *
 *   5. Applies post‑processing and confidence gating:
 *        - Minimum confidence threshold
 *        - Optional “unknown” class routing
 *        - Optional smoothing or hysteresis
 *      This prevents spurious classifications and ensures that only deliberate,
 *      well‑formed gestures trigger system actions.
 *
 *   6. Emits a final classification event.
 *      The event includes:
 *        - gesture_id (top‑1 class)
 *        - confidence
 *        - timestamp
 *      This event is consumed by the home assistant task.
 *
 * WHY THIS DESIGN IS ESSENTIAL:
 *   - The ML model expects clean, fixed‑length, normalized input. Only the
 *     inference task should touch the interpreter to avoid concurrency issues.
 *   - Running inference in its own task isolates CPU load and ensures that
 *     gesture capture, fusion, and normalization remain real‑time safe.
 *   - Confidence gating prevents accidental activations caused by noise,
 *     partial gestures, or ambiguous shapes.
 *   - By decoupling inference from gesture capture, the system remains stable
 *     even under heavy load or rapid gesture sequences.
 *
 * RESULT:
 *   A deterministic, thread‑safe inference pipeline that transforms a
 *   normalized gesture into a clean, high‑confidence classification event
 *   ready for application‑level logic.
 * -------------------------------------------------------------------------- */
void inference_task_start(void);
