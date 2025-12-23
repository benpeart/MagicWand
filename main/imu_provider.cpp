#include "imu_provider.h"

#include <cstdlib>
#include <cstring>

// FreeRTOS.h must be included before some of the following dependencies.
// Solves b/150260343.
// clang-format off
#include "freertos/FreeRTOS.h"
// clang-format on

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "i2cdev.h"
#include "ringbuf.h"
#include "micro_model_settings.h"

using namespace std;

static const char *TAG = "TF_LITE_IMU_PROVIDER";

/* MPU6050 I2C configuration */
#define IMU_I2C_PORT I2C_NUM_0
#define IMU_I2C_SDA_GPIO GPIO_NUM_8
#define IMU_I2C_SCL_GPIO GPIO_NUM_9
#define IMU_I2C_ADDR MPU6050_I2C_ADDRESS_LOW

/* ringbuffer to hold the incoming IMU data */
ringbuf_t *g_imu_capture_buffer;
volatile int32_t g_latest_imu_timestamp = 0;

/* model requires 20ms new data and 10ms old data each time */
constexpr int32_t history_samples_to_keep = ((kFeatureDurationMs - kFeatureStrideMs) * (kAudioSampleFrequency / 1000));
constexpr int32_t new_samples_to_get = (kFeatureStrideMs * (kAudioSampleFrequency / 1000));

const int32_t kIMUCaptureBufferSize = 40000;

namespace
{
    int16_t g_imu_output_buffer[kMaxAudioSampleSize * 32];
    bool g_is_imu_initialized = false;
    int16_t g_history_buffer[history_samples_to_keep];
    mpu6050_dev_t mpu6050_dev;
} // namespace

static void CaptureIMUSamples(void *arg)
{
    esp_err_t ret = ESP_OK;
    mpu6050_raw_acceleration_t raw_accel;
    mpu6050_raw_rotation_t raw_gyro;
    uint8_t imu_data_buffer[12]; // 6 bytes accel + 6 bytes gyro

    ESP_LOGI(TAG, "Starting IMU capture task");

    while (1)
    {
        /* Read accelerometer and gyroscope data */
        ret = mpu6050_get_raw_acceleration(&mpu6050_dev, &raw_accel);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading accelerometer: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ret = mpu6050_get_raw_rotation(&mpu6050_dev, &raw_gyro);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading gyroscope: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Pack raw data into buffer: accel(x,y,z) + gyro(x,y,z) */
        memcpy(&imu_data_buffer[0], &raw_accel.x, 2);
        memcpy(&imu_data_buffer[2], &raw_accel.y, 2);
        memcpy(&imu_data_buffer[4], &raw_accel.z, 2);
        memcpy(&imu_data_buffer[6], &raw_gyro.x, 2);
        memcpy(&imu_data_buffer[8], &raw_gyro.y, 2);
        memcpy(&imu_data_buffer[10], &raw_gyro.z, 2);

        /* Write to ring buffer */
        int bytes_written = rb_write(g_imu_capture_buffer,
                                     (uint8_t *)imu_data_buffer,
                                     sizeof(imu_data_buffer),
                                     pdMS_TO_TICKS(100));

        if (bytes_written == sizeof(imu_data_buffer))
        {
            /* Update timestamp */
            g_latest_imu_timestamp += 10; // Approximate 10ms per sample
        }
        else if (bytes_written > 0)
        {
            ESP_LOGW(TAG, "Partial write: %d bytes of %d", bytes_written, (int)sizeof(imu_data_buffer));
        }
        else
        {
            ESP_LOGE(TAG, "Ring buffer write failed");
        }

        /* Sample at approximately 100Hz (10ms intervals) */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

TfLiteStatus InitIMUSensor()
{
    esp_err_t ret = ESP_OK;

    /* Initialize I2C interface */
    ret = i2cdev_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2cdev_init failed: %s", esp_err_to_name(ret));
        return kTfLiteError;
    }

    /* Initialize MPU6050 device descriptor */
    ret = mpu6050_init_desc(&mpu6050_dev, IMU_I2C_ADDR, IMU_I2C_PORT, IMU_I2C_SDA_GPIO, IMU_I2C_SCL_GPIO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050_init_desc failed: %s", esp_err_to_name(ret));
        return kTfLiteError;
    }

    /* Initialize MPU6050 sensor */
    ret = mpu6050_init(&mpu6050_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050_init failed: %s", esp_err_to_name(ret));
        mpu6050_free_desc(&mpu6050_dev);
        return kTfLiteError;
    }

    /* Create ring buffer for IMU data */
    g_imu_capture_buffer = rb_init("imu_ringbuffer", kIMUCaptureBufferSize);
    if (!g_imu_capture_buffer)
    {
        ESP_LOGE(TAG, "Error creating ring buffer");
        mpu6050_free_desc(&mpu6050_dev);
        return kTfLiteError;
    }

    /* Create IMU capture task */
    xTaskCreate(CaptureIMUSamples, "CaptureIMUSamples", 1024 * 4, NULL, 10, NULL);

    /* Wait for initial data */
    while (g_latest_imu_timestamp == 0)
    {
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "IMU Sensor initialized and capturing data");
    return kTfLiteOk;
}

TfLiteStatus GetIMUSamples(int start_ms, int duration_ms, int *imu_samples_size, int16_t **imu_samples)
{
    if (!g_is_imu_initialized)
    {
        TfLiteStatus init_status = InitIMUSensor();
        if (init_status != kTfLiteOk)
        {
            return init_status;
        }
        g_is_imu_initialized = true;
    }

    /* Copy history buffer to output buffer */
    memcpy((void *)(g_imu_output_buffer), (void *)(g_history_buffer), history_samples_to_keep * sizeof(int16_t));

    /* Read new data from ring buffer */
    int bytes_read = rb_read(g_imu_capture_buffer, ((uint8_t *)(g_imu_output_buffer + history_samples_to_keep)), new_samples_to_get * sizeof(int16_t), pdMS_TO_TICKS(200));
    if (bytes_read < 0)
    {
        ESP_LOGE(TAG, "Could not read data from Ring Buffer");
    }
    else if (bytes_read < new_samples_to_get * sizeof(int16_t))
    {
        ESP_LOGD(TAG, "Ring buffer filled: %d bytes", rb_filled(g_imu_capture_buffer));
        ESP_LOGD(TAG, "Partial Read of Data by Model");
        ESP_LOGV(TAG, "Could only read %d bytes when required %d bytes", bytes_read, (int)(new_samples_to_get * sizeof(int16_t)));
    }

    /* Update history buffer */
    memcpy((void *)(g_history_buffer),
           (void *)(g_imu_output_buffer + new_samples_to_get),
           history_samples_to_keep * sizeof(int16_t));

    *imu_samples_size = kMaxAudioSampleSize;
    *imu_samples = g_imu_output_buffer;
    return kTfLiteOk;
}

int32_t LatestIMUTimestamp() { return g_latest_imu_timestamp; }
