#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t g_imu_queue;     // ImuSample -> Fusion
extern QueueHandle_t g_fusion_queue;  // FusionSample -> Inference
extern QueueHandle_t g_gesture_queue; // GestureEvent -> Home Assistant

void queues_init(void);
