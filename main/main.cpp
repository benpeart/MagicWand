#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "queues.h"
#include "data_types.h"
#include "imu_task.h"
#include "fusion_task.h"
#include "inference_task.h"
#include "ha_task.h"
#include "data_capture_task.h"

static const char *TAG = "main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Entering app_main");

    queues_init();

    imu_task_start();
    fusion_task_start();
#ifdef ENABLE_DATA_CAPTURE
    capture_task_start();
#else
    inference_task_start();
    ha_task_start();
#endif

    // Kill the default main task; everything now runs in our tasks
    ESP_LOGI(TAG, "Exiting app_main");
    vTaskDelete(NULL);
}
