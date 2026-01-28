#include "imu_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "mpu6050.h" // from esp-idf-lib
#include "queues.h"
#include "data_types.h"

// -----------------------------
// Task Configuration
// -----------------------------
static const char *TAG = "imu_task";
#define IMU_TASK_CORE 0
#define IMU_TASK_PRIORITY 8
#define IMU_TASK_STACK (4 * 1024)

// I2C pins for ESP32-S3 (TinyS3 / OMG-S3)
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN GPIO_NUM_8
#define I2C_SCL_PIN GPIO_NUM_9
#define I2C_FREQ_HZ 400000

#define IMU_PERIOD_US (1000000 / IMU_RATE_HZ)

static mpu6050_dev_t mpu;

// ---------------------------------------------------------
// I2C initialization
// ---------------------------------------------------------
static esp_err_t i2c_init(void)
{
    // Init i2cdev (this will configure + install the legacy I2C driver)
    ESP_ERROR_CHECK(i2cdev_init());

    return ESP_OK;
}

// ---------------------------------------------------------
// IMU initialization
// ---------------------------------------------------------
static esp_err_t imu_init(void)
{
    // Create device descriptor
    ESP_ERROR_CHECK(mpu6050_init_desc(
        &mpu,
        MPU6050_I2C_ADDRESS_LOW,
        I2C_PORT,
        I2C_SDA_PIN,
        I2C_SCL_PIN));

    // Initialize chip
    ESP_ERROR_CHECK(mpu6050_init(&mpu));

    // Configure full-scale ranges
    ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&mpu, MPU6050_ACCEL_RANGE_4));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&mpu, MPU6050_GYRO_RANGE_500));

    ESP_LOGI(TAG, "MPU6050 initialized (esp-idf-lib)");
    return ESP_OK;
}

// ---------------------------------------------------------
// Main IMU task
// ---------------------------------------------------------
static void imu_task(void *arg)
{
    RawImuSample sample;
    int64_t next_ts = esp_timer_get_time();

    ESP_LOGI(TAG, "Entering imu_task");
    ESP_LOGI(TAG, "g_imu_queue=%p", (void *)g_imu_queue);

    for (;;)
    {
        mpu6050_raw_acceleration_t raw_acc;
        mpu6050_raw_rotation_t raw_rot;

        if (mpu6050_get_raw_acceleration(&mpu, &raw_acc) != ESP_OK ||
            mpu6050_get_raw_rotation(&mpu, &raw_rot) != ESP_OK)
        {
            ESP_LOGW(TAG, "IMU read failed");
            continue;
        }

        sample.timestamp_us = esp_timer_get_time();

        // Convert raw accel to g → m/s²
        const float accel_scale = 1.0f / 8192.0f; // for ±4g
        sample.ax = raw_acc.x * accel_scale * 9.80665f;
        sample.ay = raw_acc.y * accel_scale * 9.80665f;
        sample.az = raw_acc.z * accel_scale * 9.80665f;

        // Convert raw gyro to deg/s
        const float gyro_scale = 1.0f / 65.5f; // for ±500°/s
        sample.gx = raw_rot.x * gyro_scale;
        sample.gy = raw_rot.y * gyro_scale;
        sample.gz = raw_rot.z * gyro_scale;

//        ESP_LOGI(TAG, "xQueueSend: %" PRIu64 ",%f,%f", sample.timestamp_us, sample.ax, sample.ay);
        xQueueSend(g_imu_queue, &sample, 0);

        next_ts += IMU_PERIOD_US;
        int64_t now = esp_timer_get_time();
        int64_t delay = next_ts - now;

        if (delay > 0)
        {
            esp_rom_delay_us(delay);
        }
        else
        {
            next_ts = now;
        }
    }
}

void imu_task_start(void)
{
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(imu_init());

    xTaskCreatePinnedToCore(
        imu_task,
        TAG,
        IMU_TASK_STACK,
        NULL,
        IMU_TASK_PRIORITY,
        NULL,
        IMU_TASK_CORE);
}
