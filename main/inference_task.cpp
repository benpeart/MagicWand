#include "inference_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queues.h"
#include "data_types.h"
#include "model.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// -----------------------------
// Task Configuration
// -----------------------------
static const char *TAG = "inference_task";
#define INFER_TASK_CORE 1
#define INFER_TASK_PRIORITY 7
#define INFER_TASK_STACK (12 * 1024)

#define FEATURE_COUNT 2 // tip_x, tip_y

#define CONFIDENCE_THRESHOLD 0.70f // tune as needed

static const int kTensorArenaSize = 60 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

static const tflite::Model *model = NULL;
static tflite::MicroInterpreter *interpreter = NULL;
static TfLiteTensor *input_tensor = NULL;

// ---------------------------------------------------------
// TFLM init
// ---------------------------------------------------------
static void tflm_init(void)
{
    model = tflite::GetModel(g_model);
    configASSERT(model->version() == TFLITE_SCHEMA_VERSION);

    static tflite::MicroMutableOpResolver<8> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddFullyConnected();
    resolver.AddReshape();
    resolver.AddMaxPool2D();
    resolver.AddUnidirectionalSequenceLSTM();
    resolver.AddSoftmax();
    resolver.AddQuantize();

    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);

    interpreter = &static_interpreter;

    TfLiteStatus alloc_status = interpreter->AllocateTensors();
    configASSERT(alloc_status == kTfLiteOk);

    input_tensor = interpreter->input(0);

    configASSERT(input_tensor->dims->size == 3);
    configASSERT(input_tensor->dims->data[1] == INFERENCE_WINDOW_SIZE);
    configASSERT(input_tensor->dims->data[2] == FEATURE_COUNT);

    ESP_LOGI(TAG, "TFLM ready: input [%d x %d]", INFERENCE_WINDOW_SIZE, FEATURE_COUNT);
}

// ---------------------------------------------------------
// Decode model output → GestureType
// ---------------------------------------------------------
static GestureType decode_output(float *out_confidence)
{
    TfLiteTensor *out = interpreter->output(0);
    int num_classes = out->dims->data[out->dims->size - 1];

    int best = 0;
    float best_val = out->data.f[0];

    for (int i = 1; i < num_classes; i++)
    {
        float v = out->data.f[i];
        if (v > best_val)
        {
            best_val = v;
            best = i;
        }
    }

    *out_confidence = best_val;

    // Apply confidence threshold
    if (best_val < CONFIDENCE_THRESHOLD)
    {
        return GESTURE_NONE;
    }

    return (GestureType)best;
}

// ---------------------------------------------------------
// Main inference task
// ---------------------------------------------------------
static void inference_task(void *arg)
{
    GestureSample gesture[INFERENCE_WINDOW_SIZE];
    int count = 0;

    ESP_LOGI(TAG, "Entering inference_task");

    tflm_init();

    for (;;)
    {
        // Wait for start of gesture marker indicating a new segmented gesture
        if (xQueueReceive(g_fusion_queue, &gesture[0], portMAX_DELAY) != pdTRUE)
            continue;
        count = 0;

        // Now collect the gesture
        while (count < INFERENCE_WINDOW_SIZE)
        {
            if (xQueueReceive(g_fusion_queue, &gesture[count], 0) != pdTRUE)
                break;
            count++;
        }

        if (count < 32)
        {
            ESP_LOGW(TAG, "Gesture too small (%d samples), ignored", count);
            continue;
        }

        // Run inference
        if (interpreter->Invoke() != kTfLiteOk)
        {
            ESP_LOGW(TAG, "Invoke failed");
            continue;
        }

        float confidence = 0.0f;
        GestureType g = decode_output(&confidence);

        if (g != GESTURE_NONE)
        {
            GestureEvent ev = {
                .timestamp_us = gesture[0].timestamp_us,
                .gesture = g};
            xQueueSend(g_gesture_queue, &ev, 0);

            ESP_LOGI(TAG, "Gesture recognized: %d (conf=%.2f)", g, confidence);
        }
        else
        {
            ESP_LOGI(TAG, "Gesture rejected (conf=%.2f)", confidence);
        }
    }
}

void inference_task_start(void)
{
    xTaskCreatePinnedToCore(
        inference_task,
        TAG,
        INFER_TASK_STACK,
        NULL,
        INFER_TASK_PRIORITY,
        NULL,
        INFER_TASK_CORE);
}
