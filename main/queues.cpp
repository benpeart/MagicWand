#include "queues.h"
#include "data_types.h"

QueueHandle_t g_imu_queue = NULL;
QueueHandle_t g_fusion_queue = NULL;
QueueHandle_t g_gesture_queue = NULL;

void queues_init(void)
{
    // Tune depths as needed
    g_imu_queue = xQueueCreate(64, sizeof(RawImuSample));
    g_fusion_queue = xQueueCreate(INFERENCE_WINDOW_SIZE + 1, sizeof(GestureSample));  // ensure enough room for one full gesture + marker
    g_gesture_queue = xQueueCreate(16, sizeof(GestureEvent));

    // Optionally assert
    configASSERT(g_imu_queue != NULL);
    configASSERT(g_fusion_queue != NULL);
    configASSERT(g_gesture_queue != NULL);
}
