#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "imu.h"
#include "qmi8658.h"
#include <string.h>
#include <math.h>

static const char *TAG = "IMU";

// Static variables
static i2c_master_dev_handle_t qmi8658_handle = NULL;
static QueueHandle_t imu_data_queue = NULL;
static imu_config_t current_config;

// Gyro bias calibration data
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static bool gyro_calibrated = false;

// EMA filtering state for queue output path
static imu_data_t ema_state = {0};
static bool ema_initialized = false;

// Internal function prototypes
static esp_err_t qmi8658_read_reg(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t qmi8658_write_reg(uint8_t reg_addr, uint8_t data);
static esp_err_t qmi8658_verify_device(void);
static esp_err_t qmi8658_soft_reset(void);
static esp_err_t qmi8658_configure_accel(qmi8658_acc_range_t range, qmi8658_acc_odr_t odr);
static esp_err_t qmi8658_configure_gyro(qmi8658_gyr_range_t range, qmi8658_gyr_odr_t odr);
static esp_err_t qmi8658_configure_filters(bool enable_accel_lpf, bool enable_gyro_lpf, qmi8658_lpf_mode_t lpf_mode);
static esp_err_t qmi8658_enable_sensors(bool accel, bool gyro);
static esp_err_t qmi8658_calibrate_gyro_bias(uint16_t num_samples);
static void imu_task(void *pvParameters);

/**
 * @brief Read register(s) from QMI8658
 */
static esp_err_t qmi8658_read_reg(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (qmi8658_handle == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(qmi8658_handle, &reg_addr, 1, data, len, 1000);
}

/**
 * @brief Write to a single register
 */
static esp_err_t qmi8658_write_reg(uint8_t reg_addr, uint8_t data)
{
    if (qmi8658_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(qmi8658_handle, write_buf, 2, 1000);
}

/**
 * @brief Verify device ID
 */
static esp_err_t qmi8658_verify_device(void)
{
    uint8_t who_am_i;
    esp_err_t ret = qmi8658_read_reg(QMI8658_REG_WHO_AM_I, &who_am_i, 1);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    if (who_am_i != QMI8658_WHO_AM_I_VALUE)
    {
        ESP_LOGE(TAG, "Invalid WHO_AM_I: expected 0x%02X, got 0x%02X",
                 QMI8658_WHO_AM_I_VALUE, who_am_i);
        // return ESP_ERR_NOT_FOUND;
    }

    uint8_t revision;
    ret = qmi8658_read_reg(QMI8658_REG_REVISION_ID, &revision, 1);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "QMI8658C found, revision: 0x%02X", revision);
    }

    return ESP_OK;
}

/**
 * @brief Perform software reset
 */
static esp_err_t qmi8658_soft_reset(void)
{
    ESP_LOGI(TAG, "Performing soft reset");

    esp_err_t ret = qmi8658_write_reg(QMI8658_REG_RESET, QMI8658_RESET_VALUE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write reset register");
        return ret;
    }

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(QMI8658_SYSTEM_STARTUP_TIME_MS));

    return ESP_OK;
}

/**
 * @brief Configure accelerometer
 */
static esp_err_t qmi8658_configure_accel(qmi8658_acc_range_t range, qmi8658_acc_odr_t odr)
{
    uint8_t ctrl2_value = (range & 0x70) | (odr & 0x0F);

    ESP_LOGI(TAG, "Configuring accelerometer: Range=%d, ODR=0x%02X", range, odr);

    return qmi8658_write_reg(QMI8658_REG_CTRL2, ctrl2_value);
}

/**
 * @brief Configure gyroscope
 */
static esp_err_t qmi8658_configure_gyro(qmi8658_gyr_range_t range, qmi8658_gyr_odr_t odr)
{
    uint8_t ctrl3_value = (range & 0x70) | (odr & 0x0F);

    ESP_LOGI(TAG, "Configuring gyroscope: Range=%d, ODR=0x%02X", range, odr);

    return qmi8658_write_reg(QMI8658_REG_CTRL3, ctrl3_value);
}

/**
 * @brief Configure low-pass filters
 */
static esp_err_t qmi8658_configure_filters(bool enable_accel_lpf, bool enable_gyro_lpf, qmi8658_lpf_mode_t lpf_mode)
{
    uint8_t ctrl5_value = 0;

    if (enable_gyro_lpf)
    {
        ctrl5_value |= QMI8658_CTRL5_gLPF_EN;
        ctrl5_value |= ((lpf_mode & 0x03) << 5); // gLPF_MODE at bits [6:5]
    }

    if (enable_accel_lpf)
    {
        ctrl5_value |= QMI8658_CTRL5_aLPF_EN;
        ctrl5_value |= ((lpf_mode & 0x03) << 1); // aLPF_MODE at bits [2:1]
    }

    ESP_LOGI(TAG, "Configuring filters: Accel=%d, Gyro=%d, Mode=%d",
             enable_accel_lpf, enable_gyro_lpf, lpf_mode);

    return qmi8658_write_reg(QMI8658_REG_CTRL5, ctrl5_value);
}

/**
 * @brief Enable/disable sensors
 */
static esp_err_t qmi8658_enable_sensors(bool accel, bool gyro)
{
    uint8_t ctrl7_value = 0;

    if (accel)
    {
        ctrl7_value |= QMI8658_CTRL7_aEN;
    }
    if (gyro)
    {
        ctrl7_value |= QMI8658_CTRL7_gEN;
    }

    ESP_LOGI(TAG, "Enabling sensors: Accel=%d, Gyro=%d", accel, gyro);

    return qmi8658_write_reg(QMI8658_REG_CTRL7, ctrl7_value);
}

/**
 * @brief Calibrate gyroscope bias
 * Device must be stationary during calibration
 */
static esp_err_t qmi8658_calibrate_gyro_bias(uint16_t num_samples)
{
    ESP_LOGI(TAG, "Starting gyroscope bias calibration (%d samples)", num_samples);
    ESP_LOGW(TAG, "Keep device stationary during calibration!");

    // Reset bias
    gyro_bias[0] = 0.0f;
    gyro_bias[1] = 0.0f;
    gyro_bias[2] = 0.0f;

    // Accumulate samples
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < num_samples; i++)
    {
        uint8_t status;
        esp_err_t ret = qmi8658_read_reg(QMI8658_REG_STATUS0, &status, 1);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read status register");
            return ret;
        }

        // Wait for new gyro data
        while (!(status & QMI8658_STATUS0_gDA))
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            qmi8658_read_reg(QMI8658_REG_STATUS0, &status, 1);
        }

        // Read gyro data
        uint8_t gyro_data[6];
        ret = qmi8658_read_reg(QMI8658_REG_GX_L, gyro_data, 6);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read gyro data");
            continue;
        }

        int16_t gx_raw = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
        int16_t gy_raw = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
        int16_t gz_raw = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

        sum_x += gx_raw;
        sum_y += gy_raw;
        sum_z += gz_raw;
        valid_samples++;

        if ((i % 100) == 0)
        {
            ESP_LOGI(TAG, "Calibration progress: %d/%d", i, num_samples);
        }
    }

    if (valid_samples < num_samples / 2)
    {
        ESP_LOGE(TAG, "Insufficient valid samples for calibration");
        return ESP_FAIL;
    }

    // Calculate average bias
    gyro_bias[0] = (float)(sum_x / valid_samples);
    gyro_bias[1] = (float)(sum_y / valid_samples);
    gyro_bias[2] = (float)(sum_z / valid_samples);

    gyro_calibrated = true;

    ESP_LOGI(TAG, "Gyro bias calibration complete:");
    ESP_LOGI(TAG, "  X bias: %.2f LSB (%.3f dps)", gyro_bias[0],
             gyro_bias[0] / current_config.gyro_sensitivity);
    ESP_LOGI(TAG, "  Y bias: %.2f LSB (%.3f dps)", gyro_bias[1],
             gyro_bias[1] / current_config.gyro_sensitivity);
    ESP_LOGI(TAG, "  Z bias: %.2f LSB (%.3f dps)", gyro_bias[2],
             gyro_bias[2] / current_config.gyro_sensitivity);

    return ESP_OK;
}

/**
 * @brief Initialize QMI8658 device
 */
esp_err_t qmi8658_init(i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid bus handle");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure I2C device
    i2c_device_config_t dev_cfg = {
        .device_address = QMI8658_I2C_ADDR_HIGH,
        .scl_speed_hz = QMI8658_I2C_SPEED_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &qmi8658_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Small delay for device power-up
    vTaskDelay(pdMS_TO_TICKS(10));

    // Verify device
    ret = qmi8658_verify_device();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Device verification failed");
        return ret;
    }

    // Perform soft reset
    ret = qmi8658_soft_reset();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Soft reset failed");
        return ret;
    }

    // Configure CTRL1 for proper I2C operation
    // Enable address auto-increment for burst reads
    ret = qmi8658_write_reg(QMI8658_REG_CTRL1, QMI8658_CTRL1_ADDR_AI);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure CTRL1");
        return ret;
    }

    ESP_LOGI(TAG, "QMI8658C initialized successfully");

    return ESP_OK;
}

/**
 * @brief Read raw sensor data
 */
esp_err_t imu_read_raw(imu_raw_data_t *data)
{
    if (data == NULL || qmi8658_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Read accelerometer and gyroscope data (12 bytes)
    uint8_t sensor_data[12];
    esp_err_t ret = qmi8658_read_reg(QMI8658_REG_AX_L, sensor_data, 12);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }

    // Parse accelerometer data
    data->accel_x = (int16_t)((sensor_data[1] << 8) | sensor_data[0]);
    data->accel_y = (int16_t)((sensor_data[3] << 8) | sensor_data[2]);
    data->accel_z = (int16_t)((sensor_data[5] << 8) | sensor_data[4]);

    // Parse gyroscope data
    data->gyro_x = (int16_t)((sensor_data[7] << 8) | sensor_data[6]);
    data->gyro_y = (int16_t)((sensor_data[9] << 8) | sensor_data[8]);
    data->gyro_z = (int16_t)((sensor_data[11] << 8) | sensor_data[10]);

    data->timestamp = esp_timer_get_time();

    return ESP_OK;
}

/**
 * @brief Read processed sensor data with calibration applied
 */
esp_err_t imu_read_processed(imu_data_t *data)
{
    if (data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    imu_raw_data_t raw;
    esp_err_t ret = imu_read_raw(&raw);

    if (ret != ESP_OK)
    {
        return ret;
    }

    // Convert accelerometer to g
    data->accel_x = raw.accel_x / current_config.accel_sensitivity;
    data->accel_y = raw.accel_y / current_config.accel_sensitivity;
    data->accel_z = raw.accel_z / current_config.accel_sensitivity;

    // Convert gyroscope to dps and apply bias correction
    if (gyro_calibrated)
    {
        data->gyro_x = (raw.gyro_x - gyro_bias[0]) / current_config.gyro_sensitivity;
        data->gyro_y = (raw.gyro_y - gyro_bias[1]) / current_config.gyro_sensitivity;
        data->gyro_z = (raw.gyro_z - gyro_bias[2]) / current_config.gyro_sensitivity;
    }
    else
    {
        data->gyro_x = raw.gyro_x / current_config.gyro_sensitivity;
        data->gyro_y = raw.gyro_y / current_config.gyro_sensitivity;
        data->gyro_z = raw.gyro_z / current_config.gyro_sensitivity;
    }

    data->timestamp = raw.timestamp;

    return ESP_OK;
}

/**
 * @brief Read temperature
 */
esp_err_t imu_read_temperature(float *temperature)
{
    if (temperature == NULL || qmi8658_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t temp_data[2];
    esp_err_t ret = qmi8658_read_reg(QMI8658_REG_TEMP_L, temp_data, 2);

    if (ret != ESP_OK)
    {
        return ret;
    }

    int16_t temp_raw = (int16_t)((temp_data[1] << 8) | temp_data[0]);
    *temperature = temp_raw / (float)QMI8658_TEMP_SENSITIVITY;

    return ESP_OK;
}

/**
 * @brief IMU reading task
 */
static void imu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "IMU task started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / current_config.sample_rate_hz);

    while (1)
    {
        imu_data_t imu_data;

        if (imu_read_processed(&imu_data) == ESP_OK)
        {
            if (!ema_initialized)
            {
                ema_state = imu_data;
                ema_initialized = true;
            }
            else
            {
                const float alpha_accel = current_config.ema_alpha_accel;
                const float alpha_gyro = current_config.ema_alpha_gyro;
                const float one_minus_alpha_accel = 1.0f - alpha_accel;
                const float one_minus_alpha_gyro = 1.0f - alpha_gyro;

                ema_state.accel_x = alpha_accel * imu_data.accel_x + one_minus_alpha_accel * ema_state.accel_x;
                ema_state.accel_y = alpha_accel * imu_data.accel_y + one_minus_alpha_accel * ema_state.accel_y;
                ema_state.accel_z = alpha_accel * imu_data.accel_z + one_minus_alpha_accel * ema_state.accel_z;

                ema_state.gyro_x = alpha_gyro * imu_data.gyro_x + one_minus_alpha_gyro * ema_state.gyro_x;
                ema_state.gyro_y = alpha_gyro * imu_data.gyro_y + one_minus_alpha_gyro * ema_state.gyro_y;
                ema_state.gyro_z = alpha_gyro * imu_data.gyro_z + one_minus_alpha_gyro * ema_state.gyro_z;

                ema_state.timestamp = imu_data.timestamp;
            }

            if (imu_data_queue != NULL)
            {
                /* Overwrite the single-slot queue so the queue always contains the latest sample. */
                xQueueOverwrite(imu_data_queue, &ema_state);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read IMU data");
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Initialize IMU with configuration
 */
esp_err_t imu_init(i2c_master_bus_handle_t bus_handle, QueueHandle_t imu_queue)
{
    if (bus_handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid bus handle");
        return ESP_ERR_INVALID_ARG;
    }

    // Default configuration for aircraft instrumentation
    imu_config_t config = {
        .accel_range = QMI8658_ACC_RANGE_8G,      // ±8g for aircraft maneuvers
        .gyro_range = QMI8658_GYR_RANGE_1024_DPS, // ±1024 dps for aerobatics
        .accel_odr = QMI8658_ACC_ODR_500_HZ,      // 1000 Hz for high update rate
        .gyro_odr = QMI8658_GYR_ODR_940_HZ,       // 1880 Hz (6DOF mode: ~1880 Hz)
        .enable_accel_lpf = true,
        .enable_gyro_lpf = true,
        .lpf_mode = QMI8658_LPF_MODE_3, // 13.37% of ODR for noise reduction
        .sample_rate_hz = 60,           // 100 Hz output rate
        .ema_alpha_accel = 0.02f,
        .ema_alpha_gyro = 0.02f,
        .calibrate_on_init = true,
        .calibration_samples = 5000,
    };

    return imu_init_with_config(bus_handle, imu_queue, &config);
}

/**
 * @brief Initialize IMU with custom configuration
 */
esp_err_t imu_init_with_config(i2c_master_bus_handle_t bus_handle,
                               QueueHandle_t imu_queue,
                               const imu_config_t *config)
{
    if (bus_handle == NULL || config == NULL)
    {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    // Save configuration
    memcpy(&current_config, config, sizeof(imu_config_t));
    if (current_config.ema_alpha_accel < 0.0f)
    {
        current_config.ema_alpha_accel = 0.0f;
    }
    else if (current_config.ema_alpha_accel > 1.0f)
    {
        current_config.ema_alpha_accel = 1.0f;
    }
    if (current_config.ema_alpha_gyro < 0.0f)
    {
        current_config.ema_alpha_gyro = 0.0f;
    }
    else if (current_config.ema_alpha_gyro > 1.0f)
    {
        current_config.ema_alpha_gyro = 1.0f;
    }

    ema_initialized = false;
    memset(&ema_state, 0, sizeof(ema_state));

    imu_data_queue = imu_queue;

    // Set sensitivity based on range
    switch (config->accel_range)
    {
    case QMI8658_ACC_RANGE_2G:
        current_config.accel_sensitivity = QMI8658_ACC_SENSITIVITY_2G;
        break;
    case QMI8658_ACC_RANGE_4G:
        current_config.accel_sensitivity = QMI8658_ACC_SENSITIVITY_4G;
        break;
    case QMI8658_ACC_RANGE_8G:
        current_config.accel_sensitivity = QMI8658_ACC_SENSITIVITY_8G;
        break;
    case QMI8658_ACC_RANGE_16G:
        current_config.accel_sensitivity = QMI8658_ACC_SENSITIVITY_16G;
        break;
    default:
        current_config.accel_sensitivity = QMI8658_ACC_SENSITIVITY_8G;
    }

    switch (config->gyro_range)
    {
    case QMI8658_GYR_RANGE_16_DPS:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_16DPS;
        break;
    case QMI8658_GYR_RANGE_32_DPS:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_32DPS;
        break;
    case QMI8658_GYR_RANGE_64_DPS:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_64DPS;
        break;
    case QMI8658_GYR_RANGE_128_DPS:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_128DPS;
        break;
    case QMI8658_GYR_RANGE_256_DPS:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_256DPS;
        break;
    case QMI8658_GYR_RANGE_512_DPS:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_512DPS;
        break;
    case QMI8658_GYR_RANGE_1024_DPS:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_1024DPS;
        break;
    case QMI8658_GYR_RANGE_2048_DPS:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_2048DPS;
        break;
    default:
        current_config.gyro_sensitivity = QMI8658_GYR_SENSITIVITY_1024DPS;
    }

    ESP_LOGI(TAG, "Initializing IMU with config:");
    ESP_LOGI(TAG, "  Accel range: %d, ODR: 0x%02X, Sensitivity: %.0f LSB/g",
             config->accel_range, config->accel_odr, current_config.accel_sensitivity);
    ESP_LOGI(TAG, "  Gyro range: %d, ODR: 0x%02X, Sensitivity: %.0f LSB/dps",
             config->gyro_range, config->gyro_odr, current_config.gyro_sensitivity);
    ESP_LOGI(TAG, "  Filters: Accel=%d, Gyro=%d, Mode=%d",
             config->enable_accel_lpf, config->enable_gyro_lpf, config->lpf_mode);
    ESP_LOGI(TAG, "  EMA alpha: Accel=%.3f, Gyro=%.3f",
             current_config.ema_alpha_accel, current_config.ema_alpha_gyro);

    // Initialize device
    esp_err_t ret = qmi8658_init(bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Device initialization failed");
        return ret;
    }

    // Configure accelerometer
    ret = qmi8658_configure_accel(config->accel_range, config->accel_odr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Accelerometer configuration failed");
        return ret;
    }

    // Configure gyroscope
    ret = qmi8658_configure_gyro(config->gyro_range, config->gyro_odr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Gyroscope configuration failed");
        return ret;
    }

    // Configure filters
    ret = qmi8658_configure_filters(config->enable_accel_lpf,
                                    config->enable_gyro_lpf,
                                    config->lpf_mode);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Filter configuration failed");
        return ret;
    }

    // Enable sensors
    ret = qmi8658_enable_sensors(true, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable sensors");
        return ret;
    }

    // Wait for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(QMI8658_GYRO_STARTUP_TIME_MS + 50));

    // Calibrate gyro bias if requested
    if (config->calibrate_on_init)
    {
        ret = qmi8658_calibrate_gyro_bias(config->calibration_samples);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Gyro calibration failed, continuing without calibration");
        }
    }

    // Create IMU task if queue provided
    if (imu_queue != NULL)
    {
        BaseType_t task_ret = xTaskCreate(
            imu_task,
            "imu_task",
            4096,
            NULL,
            5,
            NULL);

        if (task_ret != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to create IMU task");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "IMU initialization complete");

    return ESP_OK;
}

/**
 * @brief Perform gyro calibration
 */
esp_err_t imu_calibrate_gyro(uint16_t num_samples)
{
    if (qmi8658_handle == NULL)
    {
        ESP_LOGE(TAG, "IMU not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return qmi8658_calibrate_gyro_bias(num_samples);
}

/**
 * @brief Get current gyro bias values
 */
esp_err_t imu_get_gyro_bias(float *bias_x, float *bias_y, float *bias_z)
{
    if (!gyro_calibrated)
    {
        ESP_LOGW(TAG, "Gyro not calibrated");
        return ESP_ERR_INVALID_STATE;
    }

    if (bias_x)
        *bias_x = gyro_bias[0] / current_config.gyro_sensitivity;
    if (bias_y)
        *bias_y = gyro_bias[1] / current_config.gyro_sensitivity;
    if (bias_z)
        *bias_z = gyro_bias[2] / current_config.gyro_sensitivity;

    return ESP_OK;
}

/**
 * @brief Set gyro bias values manually
 */
esp_err_t imu_set_gyro_bias(float bias_x, float bias_y, float bias_z)
{
    gyro_bias[0] = bias_x * current_config.gyro_sensitivity;
    gyro_bias[1] = bias_y * current_config.gyro_sensitivity;
    gyro_bias[2] = bias_z * current_config.gyro_sensitivity;
    gyro_calibrated = true;

    ESP_LOGI(TAG, "Gyro bias set: X=%.3f, Y=%.3f, Z=%.3f dps", bias_x, bias_y, bias_z);

    return ESP_OK;
}

/**
 * @brief Perform self-test
 */
esp_err_t imu_self_test(imu_self_test_result_t *result)
{
    if (result == NULL || qmi8658_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting self-test");

    // Disable sensors first
    qmi8658_enable_sensors(false, false);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Test accelerometer
    esp_err_t ret = qmi8658_write_reg(QMI8658_REG_CTRL2,
                                      QMI8658_CTRL2_aST |
                                          QMI8658_ACC_RANGE_16G |
                                          QMI8658_ACC_ODR_1000_HZ);
    if (ret != ESP_OK)
    {
        result->accel_pass = false;
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(50)); // Wait for self-test

        uint8_t accel_test[6];
        ret = qmi8658_read_reg(QMI8658_REG_dVX_L, accel_test, 6);

        if (ret == ESP_OK)
        {
            int16_t ax = (int16_t)((accel_test[1] << 8) | accel_test[0]);
            int16_t ay = (int16_t)((accel_test[3] << 8) | accel_test[2]);
            int16_t az = (int16_t)((accel_test[5] << 8) | accel_test[4]);

            // Check if absolute values > 200mg (format: signed 5.11)
            float ax_mg = fabsf(ax / 2048.0f * 1000.0f);
            float ay_mg = fabsf(ay / 2048.0f * 1000.0f);
            float az_mg = fabsf(az / 2048.0f * 1000.0f);
            result->accel_pass = (ax_mg > 200.0f) && (ay_mg > 200.0f) && (az_mg > 200.0f);
        }
        else
        {
            result->accel_pass = false;
        }
    }
    // Clear self-test bit
    qmi8658_write_reg(QMI8658_REG_CTRL2,
                      current_config.accel_range |
                          current_config.accel_odr);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Test gyroscope
    ret = qmi8658_write_reg(QMI8658_REG_CTRL3,
                            QMI8658_CTRL3_gST |
                                QMI8658_GYR_RANGE_2048_DPS |
                                QMI8658_GYR_ODR_1880_HZ);
    if (ret != ESP_OK)
    {
        result->gyro_pass = false;
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(50)); // Wait for self-test
        uint8_t gyro_test[6];
        ret = qmi8658_read_reg(QMI8658_REG_dVX_L, gyro_test, 6);
        if (ret == ESP_OK)
        {
            int16_t gx = (int16_t)((gyro_test[1] << 8) | gyro_test[0]);
            int16_t gy = (int16_t)((gyro_test[3] << 8) | gyro_test[2]);
            int16_t gz = (int16_t)((gyro_test[5] << 8) | gyro_test[4]);

            // Check if absolute values > 20 dps (format: signed 7.9)
            float gx_dps = fabsf(gx / 32.0f);
            float gy_dps = fabsf(gy / 32.0f);
            float gz_dps = fabsf(gz / 32.0f);
            result->gyro_pass = (gx_dps > 20.0f) && (gy_dps > 20.0f) && (gz_dps > 20.0f);
        }
        else
        {
            result->gyro_pass = false;
        }
    }
    // Clear self-test bit
    qmi8658_write_reg(QMI8658_REG_CTRL3,
                      current_config.gyro_range |
                          current_config.gyro_odr);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Self-test complete: Accel=%s, Gyro=%s",
             result->accel_pass ? "PASS" : "FAIL",
             result->gyro_pass ? "PASS" : "FAIL");
    return ESP_OK;
}