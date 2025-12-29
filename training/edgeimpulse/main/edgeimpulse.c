// FreeRTOS.h must be included before some of the following dependencies.
// Solves b/150260343.
// clang-format off
#include "freertos/FreeRTOS.h"
// clang-format on

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_vfs_dev.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "mpu6050.h"

static const char *TAG = __FILE_NAME__;

/* UART configuration */
#define UART_PORT_NUM UART_NUM_0
#define UART_TX_GPIO GPIO_NUM_43
#define UART_RX_GPIO GPIO_NUM_44
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE (1024)

/* MPU6050 I2C configuration */
#define IMU_I2C_ADDR MPU6050_I2C_ADDRESS_LOW
#define IMU_I2C_PORT I2C_NUM_0
#define IMU_I2C_SDA_GPIO GPIO_NUM_8
#define IMU_I2C_SCL_GPIO GPIO_NUM_9

/* PID-based calibration configuration */
#define CALIBRATION_SAMPLES 500
#define CALIBRATION_KP 0.5f
#define CALIBRATION_KI 0.1f
#define CALIBRATION_KD 0.2f
#define CALIBRATION_ACCEL_TARGET 0.0f
#define CALIBRATION_GYRO_TARGET 0.0f

/* NVS configuration */
#define NVS_NAMESPACE "calibration"
#define NVS_KEY_ACCEL_X "accel_x"
#define NVS_KEY_ACCEL_Y "accel_y"
#define NVS_KEY_ACCEL_Z "accel_z"
#define NVS_KEY_GYRO_X "gyro_x"
#define NVS_KEY_GYRO_Y "gyro_y"
#define NVS_KEY_GYRO_Z "gyro_z"

mpu6050_dev_t mpu6050_dev;
int16_t accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
int16_t gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

typedef struct
{
    float Kp, Ki, Kd;
    float integral_x, integral_y, integral_z;
    float prev_error_x, prev_error_y, prev_error_z;
} pid_controller_t;

/* UART initialization function */
static void InitSerialPort(void)
{
    /* Configure UART parameters */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    /* Set UART parameters */
    uart_param_config(UART_PORT_NUM, &uart_config);

    /* Set UART pins */
    uart_set_pin(UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /* Install UART driver with ring buffer */
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);

    /* Configure stdout to use UART */
    uart_vfs_dev_use_driver(UART_PORT_NUM);

    ESP_LOGI(TAG, "Serial port initialized at %d baud", UART_BAUD_RATE);
}

/* PID-based MPU6050 calibration */
static esp_err_t CalibrateIMUSensorPID(void)
{
    esp_err_t ret = ESP_OK;
    pid_controller_t pid_accel = {CALIBRATION_KP, CALIBRATION_KI, CALIBRATION_KD, 0, 0, 0, 0, 0, 0};
    pid_controller_t pid_gyro = {CALIBRATION_KP, CALIBRATION_KI, CALIBRATION_KD, 0, 0, 0, 0, 0, 0};

    mpu6050_raw_acceleration_t raw_accel;
    mpu6050_raw_rotation_t raw_gyro;

    int32_t accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    ESP_LOGI(TAG, "Starting PID-based IMU calibration (%d samples)...", CALIBRATION_SAMPLES);
    printf("Do not move the sensor during calibration!\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Collect samples and calculate PID corrections */
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        /* Read accelerometer and gyroscope */
        ret = mpu6050_get_raw_acceleration(&mpu6050_dev, &raw_accel);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Calibration: Error reading accelerometer");
            return ret;
        }

        ret = mpu6050_get_raw_rotation(&mpu6050_dev, &raw_gyro);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Calibration: Error reading gyroscope");
            return ret;
        }

        /* Accumulate samples */
        accel_sum_x += raw_accel.x;
        accel_sum_y += raw_accel.y;
        accel_sum_z += raw_accel.z;
        gyro_sum_x += raw_gyro.x;
        gyro_sum_y += raw_gyro.y;
        gyro_sum_z += raw_gyro.z;

        /* Calculate errors */
        float accel_error_x = (float)raw_accel.x - CALIBRATION_ACCEL_TARGET;
        float accel_error_y = (float)raw_accel.y - CALIBRATION_ACCEL_TARGET;
        float accel_error_z = (float)raw_accel.z - CALIBRATION_ACCEL_TARGET;

        float gyro_error_x = (float)raw_gyro.x - CALIBRATION_GYRO_TARGET;
        float gyro_error_y = (float)raw_gyro.y - CALIBRATION_GYRO_TARGET;
        float gyro_error_z = (float)raw_gyro.z - CALIBRATION_GYRO_TARGET;

        /* Update PID integral terms */
        pid_accel.integral_x += accel_error_x;
        pid_accel.integral_y += accel_error_y;
        pid_accel.integral_z += accel_error_z;

        pid_gyro.integral_x += gyro_error_x;
        pid_gyro.integral_y += gyro_error_y;
        pid_gyro.integral_z += gyro_error_z;

        /* Calculate PID outputs (corrections) */
        float accel_correction_x = pid_accel.Kp * accel_error_x + pid_accel.Ki * pid_accel.integral_x / CALIBRATION_SAMPLES +
                                   pid_accel.Kd * (accel_error_x - pid_accel.prev_error_x);
        float accel_correction_y = pid_accel.Kp * accel_error_y + pid_accel.Ki * pid_accel.integral_y / CALIBRATION_SAMPLES +
                                   pid_accel.Kd * (accel_error_y - pid_accel.prev_error_y);
        float accel_correction_z = pid_accel.Kp * accel_error_z + pid_accel.Ki * pid_accel.integral_z / CALIBRATION_SAMPLES +
                                   pid_accel.Kd * (accel_error_z - pid_accel.prev_error_z);

        float gyro_correction_x = pid_gyro.Kp * gyro_error_x + pid_gyro.Ki * pid_gyro.integral_x / CALIBRATION_SAMPLES +
                                  pid_gyro.Kd * (gyro_error_x - pid_gyro.prev_error_x);
        float gyro_correction_y = pid_gyro.Kp * gyro_error_y + pid_gyro.Ki * pid_gyro.integral_y / CALIBRATION_SAMPLES +
                                  pid_gyro.Kd * (gyro_error_y - pid_gyro.prev_error_y);
        float gyro_correction_z = pid_gyro.Kp * gyro_error_z + pid_gyro.Ki * pid_gyro.integral_z / CALIBRATION_SAMPLES +
                                  pid_gyro.Kd * (gyro_error_z - pid_gyro.prev_error_z);

        /* Update offset estimates */
        accel_offset_x = (int16_t)(accel_sum_x / (i + 1) - accel_correction_x);
        accel_offset_y = (int16_t)(accel_sum_y / (i + 1) - accel_correction_y);
        accel_offset_z = (int16_t)(accel_sum_z / (i + 1) - accel_correction_z);

        gyro_offset_x = (int16_t)(gyro_sum_x / (i + 1) - gyro_correction_x);
        gyro_offset_y = (int16_t)(gyro_sum_y / (i + 1) - gyro_correction_y);
        gyro_offset_z = (int16_t)(gyro_sum_z / (i + 1) - gyro_correction_z);

        /* Store previous errors for derivative term */
        pid_accel.prev_error_x = accel_error_x;
        pid_accel.prev_error_y = accel_error_y;
        pid_accel.prev_error_z = accel_error_z;

        pid_gyro.prev_error_x = gyro_error_x;
        pid_gyro.prev_error_y = gyro_error_y;
        pid_gyro.prev_error_z = gyro_error_z;

        /* Print calibration progress */
        if ((i + 1) % 100 == 0)
        {
            printf("Calibration progress: %d/%d\n", i + 1, CALIBRATION_SAMPLES);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Final offset calculation */
    accel_offset_x = (int16_t)(accel_sum_x / CALIBRATION_SAMPLES);
    accel_offset_y = (int16_t)(accel_sum_y / CALIBRATION_SAMPLES);
    accel_offset_z = (int16_t)(accel_sum_z / CALIBRATION_SAMPLES);

    gyro_offset_x = (int16_t)(gyro_sum_x / CALIBRATION_SAMPLES);
    gyro_offset_y = (int16_t)(gyro_sum_y / CALIBRATION_SAMPLES);
    gyro_offset_z = (int16_t)(gyro_sum_z / CALIBRATION_SAMPLES);

    ESP_LOGI(TAG, "Calibration complete!");
    printf("Accelerometer offsets: X=%d Y=%d Z=%d\n", accel_offset_x, accel_offset_y, accel_offset_z);
    printf("Gyroscope offsets: X=%d Y=%d Z=%d\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);

    return ESP_OK;
}

/* Load calibration values from NVS storage */
static esp_err_t LoadCalibrationFromNVS(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (ret == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "NVS namespace not found, calibration data not available");
        return ESP_ERR_NVS_NOT_FOUND;
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Read calibration values from NVS */
    ret |= nvs_get_i16(nvs_handle, NVS_KEY_ACCEL_X, &accel_offset_x);
    ret |= nvs_get_i16(nvs_handle, NVS_KEY_ACCEL_Y, &accel_offset_y);
    ret |= nvs_get_i16(nvs_handle, NVS_KEY_ACCEL_Z, &accel_offset_z);
    ret |= nvs_get_i16(nvs_handle, NVS_KEY_GYRO_X, &gyro_offset_x);
    ret |= nvs_get_i16(nvs_handle, NVS_KEY_GYRO_Y, &gyro_offset_y);
    ret |= nvs_get_i16(nvs_handle, NVS_KEY_GYRO_Z, &gyro_offset_z);

    nvs_close(nvs_handle);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration values loaded from NVS");
        printf("Loaded - Accelerometer offsets: X=%d Y=%d Z=%d\n", accel_offset_x, accel_offset_y, accel_offset_z);
        printf("Loaded - Gyroscope offsets: X=%d Y=%d Z=%d\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);
        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "Error reading calibration from NVS");
        return ret;
    }
}

/* Save calibration values to NVS storage */
static esp_err_t SaveCalibrationToNVS(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Write calibration values to NVS */
    ret |= nvs_set_i16(nvs_handle, NVS_KEY_ACCEL_X, accel_offset_x);
    ret |= nvs_set_i16(nvs_handle, NVS_KEY_ACCEL_Y, accel_offset_y);
    ret |= nvs_set_i16(nvs_handle, NVS_KEY_ACCEL_Z, accel_offset_z);
    ret |= nvs_set_i16(nvs_handle, NVS_KEY_GYRO_X, gyro_offset_x);
    ret |= nvs_set_i16(nvs_handle, NVS_KEY_GYRO_Y, gyro_offset_y);
    ret |= nvs_set_i16(nvs_handle, NVS_KEY_GYRO_Z, gyro_offset_z);

    /* Commit changes */
    ret |= nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration values saved to NVS");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Error saving calibration to NVS: %s", esp_err_to_name(ret));
        return ret;
    }
}

esp_err_t InitIMUSensor()
{
    esp_err_t ret = ESP_OK;

    /* Initialize I2C interface */
    ret = i2cdev_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2cdev_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize MPU6050 device descriptor */
    ret = mpu6050_init_desc(&mpu6050_dev, IMU_I2C_ADDR, IMU_I2C_PORT, IMU_I2C_SDA_GPIO, IMU_I2C_SCL_GPIO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050_init_desc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize MPU6050 sensor */
    ret = mpu6050_init(&mpu6050_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050_init failed: %s", esp_err_to_name(ret));
        mpu6050_free_desc(&mpu6050_dev);
        return ret;
    }

    ESP_LOGI(TAG, "IMU Sensor initialized");

    /* Try to load calibration from NVS */
    esp_err_t load_ret = LoadCalibrationFromNVS();
    if (load_ret != ESP_OK)
    {
        ESP_LOGI(TAG, "Running calibration and saving to NVS...");

        /* Perform PID-based calibration */
        esp_err_t calib_ret = CalibrateIMUSensorPID();
        if (calib_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "IMU calibration failed: %s", esp_err_to_name(calib_ret));
            return calib_ret;
        }

        /* Save calibration to NVS */
        esp_err_t save_ret = SaveCalibrationToNVS();
        if (save_ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to save calibration to NVS, but continuing with calibration values");
        }
    }

    return ESP_OK;
}

void app_main(void)
{
    InitSerialPort();
    printf("ESP32 Serial Port Initialized!\n");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        /* NVS partition was truncated and needs to be erased. Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    InitIMUSensor();

    while (1)
    {
        mpu6050_raw_acceleration_t raw_accel;
        mpu6050_raw_rotation_t raw_gyro;

        ESP_LOGI(TAG, "Starting IMU capture task");

        while (1)
        {
            /* Read accelerometer and gyroscope data */
            esp_err_t ret = mpu6050_get_raw_acceleration(&mpu6050_dev, &raw_accel);
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

            /* output calibrated data to serial port */
            printf("%d,%d,%d,%d,%d,%d\n",
                   raw_accel.x - accel_offset_x, raw_accel.y - accel_offset_y, raw_accel.z - accel_offset_z,
                   raw_gyro.x - gyro_offset_x, raw_gyro.y - gyro_offset_y, raw_gyro.z - gyro_offset_z);

            // ensure we sample at ~100Hz
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}