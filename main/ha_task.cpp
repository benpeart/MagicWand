#include "ha_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queues.h"
#include "data_types.h"
#include <inttypes.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_system.h"

// -----------------------------
// Task Configuration
// -----------------------------
static const char *TAG = "ha_task";
#define HA_TASK_CORE 0
#define HA_TASK_PRIORITY 5
#define HA_TASK_STACK (6 * 1024)

// -------------------------------
// MQTT CONFIGURATION
// -------------------------------
#define MQTT_URI "mqtt://homeassistant.local"
#define MQTT_TOPIC_GESTURE "wand/gesture"
#define MQTT_TOPIC_BATTERY "wand/battery"
#define MQTT_TOPIC_RSSI "wand/rssi"

// MQTT Discovery topics
#define DISCOVERY_GESTURE "homeassistant/sensor/wand/last_gesture/config"
#define DISCOVERY_BATTERY "homeassistant/sensor/wand/battery/config"
#define DISCOVERY_RSSI "homeassistant/sensor/wand/rssi/config"

// Device metadata
#define DEVICE_ID "magic_wand_001"
#define DEVICE_NAME "Magic Wand"
#define DEVICE_MANUF "Ben"
#define DEVICE_MODEL "Gesture Wand v1"
#define DEVICE_SW "1.0.0"

// MQTT client handle
static esp_mqtt_client_handle_t mqtt_client = NULL;

// -------------------------------
// Gesture → string mapping
// -------------------------------
static const char *gesture_to_string(GestureType g)
{
    switch (g)
    {
    case GESTURE_SPELL_1:
        return "spell_1";
    case GESTURE_SPELL_2:
        return "spell_2";
    case GESTURE_SPELL_3:
        return "spell_3";
    case GESTURE_SPELL_4:
        return "spell_4";
    case GESTURE_SPELL_5:
        return "spell_5";
    default:
        return "none";
    }
}

// -------------------------------
// MQTT event handler
// -------------------------------
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    default:
        break;
    }
}

// -------------------------------
// Wi‑Fi init
// -------------------------------
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = "IOT",
            .password = "",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

// -------------------------------
// MQTT init
// -------------------------------
static void mqtt_init(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address = {
                .uri = MQTT_URI,
            },
        },
        .session = {
            .keepalive = 30,
        },
        /*
                .credentials = {
                    .username = "user",
                    .authentication = {
                        .password = "pass",
                    },
                },
        */
    };

    mqtt_client = esp_mqtt_client_init(&cfg);
    configASSERT(mqtt_client != NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        mqtt_client,
        MQTT_EVENT_ANY,
        mqtt_event_handler,
        NULL));

    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// -------------------------------
// Publish MQTT Discovery messages
// -------------------------------
static void publish_discovery()
{
    char payload[512];

    // Gesture sensor
    snprintf(payload, sizeof(payload),
             "{"
             "\"name\":\"Wand Last Gesture\","
             "\"state_topic\":\"%s\","
             "\"value_template\":\"{{ value_json.gesture }}\","
             "\"unique_id\":\"wand_last_gesture_001\","
             "\"device\":{"
             "\"identifiers\":[\"%s\"],"
             "\"name\":\"%s\","
             "\"manufacturer\":\"%s\","
             "\"model\":\"%s\","
             "\"sw_version\":\"%s\""
             "}"
             "}",
             MQTT_TOPIC_GESTURE,
             DEVICE_ID, DEVICE_NAME, DEVICE_MANUF, DEVICE_MODEL, DEVICE_SW);
    esp_mqtt_client_publish(mqtt_client, DISCOVERY_GESTURE, payload, 0, 1, true);

    // Battery sensor
    snprintf(payload, sizeof(payload),
             "{"
             "\"name\":\"Wand Battery\","
             "\"state_topic\":\"%s\","
             "\"unit_of_measurement\":\"%%\","
             "\"device_class\":\"battery\","
             "\"unique_id\":\"wand_battery_001\","
             "\"device\":{"
             "\"identifiers\":[\"%s\"],"
             "\"name\":\"%s\","
             "\"manufacturer\":\"%s\","
             "\"model\":\"%s\","
             "\"sw_version\":\"%s\""
             "}"
             "}",
             MQTT_TOPIC_BATTERY,
             DEVICE_ID, DEVICE_NAME, DEVICE_MANUF, DEVICE_MODEL, DEVICE_SW);
    esp_mqtt_client_publish(mqtt_client, DISCOVERY_BATTERY, payload, 0, 1, true);

    // RSSI sensor
    snprintf(payload, sizeof(payload),
             "{"
             "\"name\":\"Wand WiFi RSSI\","
             "\"state_topic\":\"%s\","
             "\"unit_of_measurement\":\"dBm\","
             "\"device_class\":\"signal_strength\","
             "\"unique_id\":\"wand_rssi_001\","
             "\"device\":{"
             "\"identifiers\":[\"%s\"],"
             "\"name\":\"%s\","
             "\"manufacturer\":\"%s\","
             "\"model\":\"%s\","
             "\"sw_version\":\"%s\""
             "}"
             "}",
             MQTT_TOPIC_RSSI,
             DEVICE_ID, DEVICE_NAME, DEVICE_MANUF, DEVICE_MODEL, DEVICE_SW);
    esp_mqtt_client_publish(mqtt_client, DISCOVERY_RSSI, payload, 0, 1, true);
}

// -------------------------------
// Publish gesture event
// -------------------------------
static void ha_send_gesture(const GestureEvent *ev)
{
    const char *name = gesture_to_string(ev->gesture);

    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"gesture\":\"%s\",\"timestamp\":%" PRIu64 "}",
             name, ev->timestamp_us);

    esp_mqtt_client_publish(
        mqtt_client,
        MQTT_TOPIC_GESTURE,
        payload,
        0,
        1,
        false);
}

// -------------------------------
// Publish battery + RSSI
// -------------------------------
static void publish_status()
{
    // Battery: replace with your ADC/fuel gauge reading
    int battery_percent = 87;

    // Wi‑Fi RSSI
    wifi_ap_record_t info;
    int rssi = 0;
    if (esp_wifi_sta_get_ap_info(&info) == ESP_OK)
    {
        rssi = info.rssi;
    }

    char buf[32];

    snprintf(buf, sizeof(buf), "%d", battery_percent);
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_BATTERY, buf, 0, 1, false);

    snprintf(buf, sizeof(buf), "%d", rssi);
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_RSSI, buf, 0, 1, false);
}

// -------------------------------
// Main HA task
// -------------------------------
static void ha_task(void *arg)
{
    ESP_LOGI(TAG, "Entering ha_task");

    wifi_init();
    mqtt_init();

    // Wait a moment for MQTT to connect
    vTaskDelay(pdMS_TO_TICKS(1500));

    publish_discovery();

    GestureEvent ev;
    TickType_t last_status = xTaskGetTickCount();

    for (;;)
    {
        // Publish battery + RSSI every 10 seconds
        if (xTaskGetTickCount() - last_status > pdMS_TO_TICKS(10000))
        {
            publish_status();
            last_status = xTaskGetTickCount();
        }

        // Handle gesture events
        if (xQueueReceive(g_gesture_queue, &ev, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            ha_send_gesture(&ev);
        }
    }
}

void ha_task_start(void)
{
    xTaskCreatePinnedToCore(
        ha_task,
        TAG,
        HA_TASK_STACK,
        NULL,
        HA_TASK_PRIORITY,
        NULL,
        HA_TASK_CORE);
}
