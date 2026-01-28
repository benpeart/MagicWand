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
#define CAPTURE_TASK_PRIORITY 5
#define CAPTURE_TASK_STACK (4 * 1024)

static void capture_task(void *arg)
{
    GestureSample g;

    ESP_LOGI(TAG, "Entering capture_task");

    for (;;)
    {
        if (xQueueReceive(g_fusion_queue, &g, portMAX_DELAY) == pdTRUE)
        {
//            ESP_LOGI(TAG, "xQueueReceive: %" PRIu64 ",%f,%f", g.timestamp_us, g.x, g.y);
            printf("%" PRIu64 ",%f,%f\n", g.timestamp_us, g.x, g.y);
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
