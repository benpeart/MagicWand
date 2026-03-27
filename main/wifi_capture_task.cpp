#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#include "wifi_capture_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "queues.h"
#include "data_types.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <inttypes.h>
#include "wifi_info.h"

// WiFi Event Group bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Task Configuration
static const char *TAG = "wifi_capture_task";
#define CAPTURE_TASK_CORE 1
#define CAPTURE_TASK_PRIORITY 4
#define CAPTURE_TASK_STACK (8 * 1024) // Larger stack for WiFi operations

// Global state
static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;
static int s_socket_fd = -1;
static const int MAXIMUM_RETRY = 5;

// Buffer for sending data
#define SEND_BUFFER_SIZE 512

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    ESP_LOGI(TAG, "WiFi init sta called");

    // Initialize NVS as the WiFi stack uses it to store data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static int connect_to_server(void)
{
    ESP_LOGI(TAG, "Connecting to server %s:%d", SERVER_HOST, SERVER_PORT);

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr = {
            .s_addr = inet_addr(SERVER_HOST),
        },
    };

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0)
    {
        ESP_LOGE(TAG, "Socket connect failed: errno %d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "Successfully connected to server");

    return sock;
}

static void send_gesture_data(int sock, const GestureSample *g)
{
    static char send_buffer[SEND_BUFFER_SIZE];

    if (sock < 0)
        return;

    int len = snprintf(send_buffer, SEND_BUFFER_SIZE,
                       "%" PRIu32 ",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                       g->timestamp_ms,
                       g->ax, g->ay, g->az,
                       g->gx, g->gy, g->gz,
                       g->qw, g->qx, g->qy, g->qz);

    if (len > 0 && len < SEND_BUFFER_SIZE)
    {
        int bytes_sent = send(sock, send_buffer, len, 0);
        if (bytes_sent < 0)
        {
            ESP_LOGE(TAG, "Failed to send data: errno %d, attempted %d bytes", errno, len);
            // Connection lost, will reconnect
            close(sock);
            s_socket_fd = -1;
        }
        else if (bytes_sent != len)
        {
            ESP_LOGW(TAG, "Partial send: sent %d of %d bytes", bytes_sent, len);
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to format gesture data (len=%d)", len);
    }
}

static void wifi_capture_task(void *arg)
{
    GestureSample g;

    ESP_LOGI(TAG, "Entering wifi_capture_task");

    // Wait for WiFi connection
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "WiFi connected");

    // Main loop: receive data from queue and send over socket
    int sample_count = 0;
    for (;;)
    {
        if (xQueueReceive(g_fusion_queue, &g, portMAX_DELAY) == pdTRUE)
        {
            if (g.timestamp_ms == 0)
            {
                // New gesture started - connect to server and send header
                sample_count = 0;
                ESP_LOGI(TAG, "New gesture capture started");

                // If we still have an old connection, close it
                if (s_socket_fd >= 0)
                {
                    close(s_socket_fd);
                    s_socket_fd = -1;
                }

                // Connect for new gesture
                s_socket_fd = connect_to_server();
                if (s_socket_fd < 0)
                {
                    ESP_LOGE(TAG, "Failed to connect for new gesture");
                    continue;
                }

                // Send CSV header
                const char *header = "timestamp_ms,ax,ay,az,gx,gy,gz,qw,qx,qy,qz\n";
                if (send(s_socket_fd, header, strlen(header), 0) < 0)
                {
                    ESP_LOGE(TAG, "Failed to send header: errno %d", errno);
                    close(s_socket_fd);
                    s_socket_fd = -1;
                    continue;
                }
            }

            send_gesture_data(s_socket_fd, &g);
            sample_count++;

            // After INFERENCE_WINDOW_SAMPLES, close connection to signal gesture complete
            // This allows server to process without waiting for timeout
            if (sample_count >= INFERENCE_WINDOW_SAMPLES)
            {
                ESP_LOGI(TAG, "Gesture complete: %d samples sent, closing connection", INFERENCE_WINDOW_SAMPLES);
                if (s_socket_fd >= 0)
                {
                    close(s_socket_fd);
                    s_socket_fd = -1;
                }
                sample_count = 0;
            }

            // Try to reconnect if disconnected
            if (s_socket_fd < 0 && sample_count > 0)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                s_socket_fd = connect_to_server();
                if (s_socket_fd < 0)
                {
                    ESP_LOGE(TAG, "Failed to reconnect, waiting for next gesture");
                }
            }
        }
    }
}

void wifi_capture_task_start(void)
{
    // Initialize WiFi
    wifi_init_sta();

    // Create the capture task
    xTaskCreatePinnedToCore(
        wifi_capture_task,
        TAG,
        CAPTURE_TASK_STACK,
        NULL,
        CAPTURE_TASK_PRIORITY,
        NULL,
        CAPTURE_TASK_CORE);
}
