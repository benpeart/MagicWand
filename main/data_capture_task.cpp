#include "data_capture_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queues.h"
#include "data_types.h"
#include "esp_log.h"

/*
    To run the data capture task, call capture_task_start() from main(),
    then build and flash the project as usual.

    idf.py monitor | tee gesture_raw.csv
*/

// -----------------------------
// Task Configuration
// -----------------------------
static const char *TAG = "data_capture_task";
#define CAPTURE_TASK_CORE 1
#define CAPTURE_TASK_PRIORITY 4
#define CAPTURE_TASK_STACK (4 * 1024)

static void capture_task(void *arg)
{
    GestureSample g;

    ESP_LOGI(TAG, "Entering capture_task");

    for (;;)
    {
        if (xQueueReceive(g_fusion_queue, &g, portMAX_DELAY) == pdTRUE)
        {
            printf("%" PRIu32 ",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                   g.timestamp_us,          // timestamp is relative to gesture start (first sample has timestamp 0)
                   g.ax, g.ay, g.az,        // ax, ay, az are linear acceleration (sensor frame, gravity removed by BNO08x), m/s^2
                   g.gx, g.gy, g.gz,        // gx, gy, gz are gyroscope (sensor frame, calibrated by BNO08x), rad/s
                   g.qw, g.qx, g.qy, g.qz); // qw, qx, qy, qz are the fused game rotation vector quaternion components (game rotation vector / fused orientation)
        }
    }
}

void capture_task_start(void)
{
    xTaskCreatePinnedToCore(
        capture_task,
        TAG,
        CAPTURE_TASK_STACK,
        NULL,
        CAPTURE_TASK_PRIORITY,
        NULL,
        CAPTURE_TASK_CORE);
}
