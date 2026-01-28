# Magic Wand Firmware Architecture

This document describes the architecture of the Magic Wand firmware, the responsibilities of each module, and how the system components work together to produce stable, real‑time gesture recognition on the ESP32‑S3.

The design emphasizes modularity, predictable timing, and a clean data pipeline from raw IMU readings all the way to gesture classification and Home Assistant event emission.

## High‑Level System Overview

The firmware is organized into **four FreeRTOS tasks**, each pinned to a specific core and assigned a priority based on its real‑time requirements:

| Task | Core | Priority | Purpose |
|------|------|----------|---------|
| **IMU Task** | Core 1 | Medium‑High | Samples raw accelerometer + gyro data at 200 Hz |
| **Fusion Task** | Core 1 | High | Sensor fusion, tip tracking, gesture segmentation, normalization |
| **Inference Task** | Core 1 | Medium | Runs ML model on normalized gesture samples |
| **Home Assistant Task** | Core 0 | Low | Publishes recognized gestures over Wi‑Fi |

The tasks communicate through **lock‑free FreeRTOS queues**, ensuring timing isolation and preventing slow tasks from blocking fast ones.

The overall data flow:

```
IMU → Fusion → Inference → Home Assistant
```

Each stage transforms the data into a more meaningful representation:

1. **Raw IMU samples** (accelerometer + gyro)
2. **Orientation + integrated tip trajectory**
3. **Normalized gesture samples** (centered, scaled, resampled)
4. **Gesture classification** (ML model output)
5. **Event emission** (MQTT / Home Assistant)

## Project Structure

```
main/
│
├── imu_task.cpp
├── fusion_task.cpp
├── inference_task.cpp
├── ha_task.cpp
├── data_capture_task.cpp
│
├── data_types.h
├── queues.h
├── queues.cpp
│
├── model.cpp
│
└── CMakeLists.txt
```

Each module is intentionally focused and self‑contained.

## Module‑by‑Module Breakdown

### 1. **imu_task.cpp** — Raw Sensor Acquisition (200 Hz)

**Purpose:**  
Reads raw accelerometer and gyroscope data from the IMU using the esp‑idf‑lib driver.

**Responsibilities:**

- Initialize I²C and configure the IMU  
- Sample raw accel/gyro at a fixed 200 Hz  
- Convert raw values to physical units (m/s², deg/s)  
- Timestamp each sample  
- Push `RawImuSample` into `g_imu_queue`  

**Why it runs on Core 1:**  
Consistent timing is essential for stable sensor fusion.

### 2. **fusion_task.cpp** — Orientation, Tip Tracking, Segmentation, Normalization

This is the heart of the wand’s real‑time motion pipeline.

**Purpose:**  
Consumes raw IMU samples and produces **fully normalized gesture samples** ready for inference.

**Responsibilities:**

1. **Orientation Fusion**  
   - Runs the Madgwick AHRS algorithm  
   - Maintains a quaternion representing wand orientation  

2. **Tip Tracking**  
   - Rotates acceleration into world frame  
   - Removes gravity  
   - Integrates acceleration → velocity → position  
   - Computes wand tip position in 3D  
   - Projects tip position onto the **XZ plane**  

3. **Gesture Segmentation**  
   - Detects movement start  
   - Buffers tip trajectory samples  
   - Detects stillness to mark gesture end  
   - Rejects gestures that are too short  

4. **Gesture Normalization**  
   - Centering
   - Scaling
   - Resampling to `WINDOW_SIZE`  
   - Produces a clean, consistent `(x, y)` trajectory  

5. **Output**  
   - Emits `GestureSample` (normalized) into `g_fusion_queue`  

**Why it runs on Core 1:**  
Must keep up with incoming IMU data in real time so it can properly segment gestures.

### 3. **inference_task.cpp** — ML Model Execution

**Purpose:**  
Consumes normalized gesture samples and runs the TensorFlow Lite Micro model.

**Responsibilities:**

- Read `WINDOW_SIZE` normalized `GestureSample`s  
- Copy them directly into the model input tensor  
- Run inference  
- Decode the output into a `GestureType`  
- Apply confidence thresholding  
- Emit `GestureEvent` into `g_gesture_queue`  

**Why it runs on Core 1:**  
Inference is lightweight and benefits from being colocated with fusion to minimize queue latency.

### 4. **ha_task.cpp** — Home Assistant / MQTT Event Output

**Purpose:**  
Publishes recognized gestures to Home Assistant or other consumers.

**Responsibilities:**

- Connect to Wi‑Fi  
- Publish gesture events (MQTT, WebSocket, or REST)  
- Handle reconnection and backoff  
- Run at low priority so it never interferes with real‑time tasks  

**Why it runs on Core 0:**  
Network activity is isolated from the real‑time motion pipeline.

## Inter‑Task Communication

The system uses **FreeRTOS queues** defined in `queues.h`:

- `g_imu_queue` — raw IMU samples  
- `g_fusion_queue` — normalized gesture samples  
- `g_gesture_queue` — recognized gesture events  

Queues provide:

- Thread‑safe communication  
- Back‑pressure isolation  
- Deterministic memory usage  

## Model Integration (`model.cc`)

The ML model is embedded as a byte array:

- `g_model[]` — the TFLite model data  
- `g_model_len` — length in bytes  

The inference task loads this into a TFLM interpreter with:

- Static tensor arena  
- Int8 quantized ops  
- Minimal memory footprint  

## Data Types (`data_types.h`)

Defines the core structs passed between tasks:

- `RawImuSample`  
- `GestureSample` (unified structure for both raw tip and normalized tip)  
- `GestureEvent`  

Each struct is timestamped for consistent timing across the pipeline.

## Testing & Debugging Hooks

The architecture supports:

- Logging at each stage  
- Optional streaming of IMU or fusion data over UART  
- Model confidence debugging  
- Gesture visualization tools (Python notebooks)  

##  Future Improvements

- Switch to a higher‑quality IMU (ICM‑42688‑P)  
- Implement FIFO‑based IMU sampling  
- Add calibration routines  
- Improve gesture segmentation  
