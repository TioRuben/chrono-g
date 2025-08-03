/**
 * @file imu.c
 * @brief IMU driver with software filtering for basic sensor data
 *
 * This implementation uses the QMI8658 6-axis IMU with optional software filtering
 * for raw accelerometer and gyroscope data in aircraft coordinate system.
 *
 * Key Configurable Parameters:
 * ===========================
 *
 * Software Filter:
 * - FILTER_ALPHA: Low-pass filter coefficient (0.0-1.0, recommended: 0.15)
 * - FILTER_ENABLE_ACCEL: Enable accelerometer filtering (0/1)
 * - FILTER_ENABLE_GYRO: Enable gyroscope filtering (0/1)
 *
 * Performance:
 * - IMU_BASE_FREQUENCY_HZ: Base sampling rate (125Hz)
 * - IMU_STACK_SIZE: IMU task stack size (4096 bytes)
 * - CALIBRATION_STACK_SIZE: Calibration task stack size (4096 bytes)
 *
 * Coordinate System: Aircraft (X=forward/roll, Y=right/pitch, Z=down/yaw)
 */

#include "imu.h"
#include "qmi8658.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <math.h>

static const char *TAG = "IMU";

// Software low-pass filter parameters
// ===================================
// Exponential moving average filter applied to sensor data
// Helps reduce high-frequency noise from sensors

#define FILTER_ALPHA 0.15f // Filter coefficient (0.0 - 1.0)
                           // 0.0 = no new data (infinite filtering)
                           // 1.0 = no filtering (pass-through)
                           // Higher values = less filtering, more responsive
                           // Lower values = more filtering, smoother but delayed
                           // Recommended: 0.1-0.2 for most applications

#define FILTER_ENABLE_ACCEL 1 // Enable accelerometer filtering (recommended)
#define FILTER_ENABLE_GYRO 1  // Enable gyroscope filtering (recommended)

// Parameter validation (compile-time checks for float parameters)
// Note: Float parameter validation (FILTER_ALPHA) is performed at runtime
// FILTER_ALPHA should be between 0.0 and 1.0

// IMU timing and performance parameters
// =====================================
#define IMU_BASE_FREQUENCY_HZ 125          // Base IMU sampling rate in Hz (matches QMI8658 ODR)
#define IMU_TASK_PRIORITY_OFFSET 1         // Priority offset from configMAX_PRIORITIES (higher = more priority)
#define IMU_STACK_SIZE 4096                // Stack size for IMU task in bytes
#define CALIBRATION_TASK_PRIORITY_OFFSET 2 // Priority offset for calibration task (lower than IMU)
#define CALIBRATION_STACK_SIZE 4096        // Stack size for calibration task in bytes

// Gyro calibration parameters
#define CALIBRATION_SAMPLES 2000      // Number of samples for gyro bias calibration
#define CALIBRATION_BATCH_SIZE 500    // Process calibration in batches to reduce CPU load
#define CALIBRATION_QUEUE_SIZE 8      // Size of calibration queue
#define CALIBRATION_LOG_DECIMATION 25 // Log calibration progress every N batches

// Calibration batch structure for internal RAM (better performance)
typedef struct
{
    float gyro_x[CALIBRATION_BATCH_SIZE];
    float gyro_y[CALIBRATION_BATCH_SIZE];
    float gyro_z[CALIBRATION_BATCH_SIZE];
    uint32_t sample_count;
} calibration_batch_t;

// Static variables
static qmi8658_dev_t dev;
static QueueHandle_t imu_data_queue = NULL;
static TaskHandle_t imu_task_handle = NULL;
static TaskHandle_t calibration_task_handle = NULL;
static QueueHandle_t calibration_queue = NULL;
static bool imu_initialized = false;

// Software filter state
static imu_data_t filtered_data = {0};
static bool filter_initialized = false;

// Gyro calibration state
static imu_calibration_status_t calibration_status = IMU_CALIBRATION_NOT_STARTED;
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static uint32_t calibration_sample_count = 0;
static float gyro_sum_x = 0.0f;
static float gyro_sum_y = 0.0f;
static float gyro_sum_z = 0.0f;

// PSRAM buffer for calibration samples
static calibration_batch_t *current_batch = NULL;
static uint32_t batch_index = 0;
static uint32_t calibration_log_counter = 0;

/**
 * @brief Process a batch of calibration samples
 *
 * This function runs in a separate task to avoid blocking the IMU reading task
 */
static void process_calibration_batch(calibration_batch_t *batch)
{
    if (batch == NULL || batch->sample_count == 0)
    {
        return;
    }

    // Process all samples in the batch
    for (uint32_t i = 0; i < batch->sample_count; i++)
    {
        gyro_sum_x += batch->gyro_x[i];
        gyro_sum_y += batch->gyro_y[i];
        gyro_sum_z += batch->gyro_z[i];
        calibration_sample_count++;
    }

    // Log progress with decimation to reduce console spam
    calibration_log_counter++;
    if (calibration_log_counter >= CALIBRATION_LOG_DECIMATION)
    {
        ESP_LOGD(TAG, "Calibration progress: %ld/%d samples", calibration_sample_count, CALIBRATION_SAMPLES);
        calibration_log_counter = 0;
    }

    // Check if calibration is complete
    if (calibration_sample_count >= CALIBRATION_SAMPLES)
    {
        // Calculate average bias
        gyro_bias_x = gyro_sum_x / (float)calibration_sample_count;
        gyro_bias_y = gyro_sum_y / (float)calibration_sample_count;
        gyro_bias_z = gyro_sum_z / (float)calibration_sample_count;

        calibration_status = IMU_CALIBRATION_COMPLETED;
        ESP_LOGI(TAG, "Gyro calibration completed!");
        ESP_LOGD(TAG, "Gyro bias: X=%.6f, Y=%.6f, Z=%.6f deg/s",
                 gyro_bias_x, gyro_bias_y, gyro_bias_z);
    }
}

/**
 * @brief Calibration processing task
 *
 * Dedicated task for processing calibration batches without blocking IMU reading
 */
static void calibration_task(void *pvParameters)
{
    calibration_batch_t *received_batch;

    while (1)
    {
        // Wait for calibration batch from queue
        if (xQueueReceive(calibration_queue, &received_batch, portMAX_DELAY) == pdTRUE)
        {
            if (calibration_status == IMU_CALIBRATION_IN_PROGRESS)
            {
                process_calibration_batch(received_batch);
            }

            // Free the PSRAM buffer after processing
            if (received_batch != NULL)
            {
                heap_caps_free(received_batch);
            }
        }
    }
}

/**
 * @brief Add gyro sample to calibration buffer
 *
 * Collects samples in batches and sends them to calibration task
 */
static void add_calibration_sample(float gyro_x, float gyro_y, float gyro_z)
{
    if (calibration_status == IMU_CALIBRATION_NOT_STARTED)
    {
        // Start calibration
        calibration_status = IMU_CALIBRATION_IN_PROGRESS;
        calibration_sample_count = 0;
        gyro_sum_x = 0.0f;
        gyro_sum_y = 0.0f;
        gyro_sum_z = 0.0f;
        batch_index = 0;
        calibration_log_counter = 0;

        // Allocate first batch in internal RAM for better performance
        current_batch = heap_caps_malloc(sizeof(calibration_batch_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        if (current_batch == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate calibration batch in internal RAM");
            calibration_status = IMU_CALIBRATION_NOT_STARTED;
            return;
        }
        current_batch->sample_count = 0;
    }

    if (calibration_status == IMU_CALIBRATION_IN_PROGRESS && current_batch != NULL)
    {
        // Add sample to current batch
        current_batch->gyro_x[batch_index] = gyro_x;
        current_batch->gyro_y[batch_index] = gyro_y;
        current_batch->gyro_z[batch_index] = gyro_z;
        batch_index++;
        current_batch->sample_count = batch_index;

        // If batch is full, send it to calibration task
        if (batch_index >= CALIBRATION_BATCH_SIZE)
        {
            // Send batch to calibration task (non-blocking)
            if (xQueueSend(calibration_queue, &current_batch, 0) == pdTRUE)
            {
                // Allocate new batch for next samples in internal RAM
                current_batch = heap_caps_malloc(sizeof(calibration_batch_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
                if (current_batch == NULL)
                {
                    ESP_LOGE(TAG, "Failed to allocate new calibration batch in internal RAM");
                    calibration_status = IMU_CALIBRATION_COMPLETED; // Stop calibration
                    return;
                }
                current_batch->sample_count = 0;
                batch_index = 0;
            }
            else
            {
                ESP_LOGW(TAG, "Calibration queue full, dropping batch");
                // Reset current batch index to overwrite
                batch_index = 0;
                current_batch->sample_count = 0;
            }
        }
    }
}

/**
 * @brief Apply software low-pass filter to IMU data
 *
 * Uses a simple exponential moving average filter for sensor data
 * filtered_value = alpha * new_value + (1 - alpha) * old_value
 */
/**
 * @brief Apply software low-pass filter to IMU data
 *
 * Uses a simple exponential moving average filter for sensor data
 * filtered_value = alpha * new_value + (1 - alpha) * old_value
 */
static void apply_software_filter(imu_data_t *raw_data, imu_data_t *filtered_output)
{
    if (!filter_initialized)
    {
        // Initialize filter with first sample
        *filtered_output = *raw_data;
        filter_initialized = true;
        return;
    }

    // Apply exponential moving average filter
    if (FILTER_ENABLE_ACCEL)
    {
        filtered_output->accel_x = FILTER_ALPHA * raw_data->accel_x + (1.0f - FILTER_ALPHA) * filtered_output->accel_x;
        filtered_output->accel_y = FILTER_ALPHA * raw_data->accel_y + (1.0f - FILTER_ALPHA) * filtered_output->accel_y;
        filtered_output->accel_z = FILTER_ALPHA * raw_data->accel_z + (1.0f - FILTER_ALPHA) * filtered_output->accel_z;
    }
    else
    {
        // Pass-through without filtering
        filtered_output->accel_x = raw_data->accel_x;
        filtered_output->accel_y = raw_data->accel_y;
        filtered_output->accel_z = raw_data->accel_z;
    }

    if (FILTER_ENABLE_GYRO)
    {
        filtered_output->gyro_x = FILTER_ALPHA * raw_data->gyro_x + (1.0f - FILTER_ALPHA) * filtered_output->gyro_x;
        filtered_output->gyro_y = FILTER_ALPHA * raw_data->gyro_y + (1.0f - FILTER_ALPHA) * filtered_output->gyro_y;
        filtered_output->gyro_z = FILTER_ALPHA * raw_data->gyro_z + (1.0f - FILTER_ALPHA) * filtered_output->gyro_z;
    }
    else
    {
        // Pass-through without filtering
        filtered_output->gyro_x = raw_data->gyro_x;
        filtered_output->gyro_y = raw_data->gyro_y;
        filtered_output->gyro_z = raw_data->gyro_z;
    }

    filtered_output->timestamp = raw_data->timestamp;
    filtered_output->calibration_status = calibration_status;
}

/**
 * @brief IMU reading task
 *
 * Continuously reads IMU data at 125Hz and sends it to the queue
 */
static void imu_task(void *pvParameters)
{
    imu_data_t raw_imu_data;
    imu_data_t mapped_imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(8); // 125Hz = 8ms period

    while (1)
    {
        // Read accelerometer and gyroscope data
        esp_err_t ret = qmi8658_read_accel(&dev, &raw_imu_data.accel_x, &raw_imu_data.accel_y, &raw_imu_data.accel_z);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to read accelerometer data: %d", ret);
            vTaskDelayUntil(&last_wake_time, frequency);
            continue;
        }

        ret = qmi8658_read_gyro(&dev, &raw_imu_data.gyro_x, &raw_imu_data.gyro_y, &raw_imu_data.gyro_z);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to read gyroscope data: %d", ret);
            vTaskDelayUntil(&last_wake_time, frequency);
            continue;
        }

        // Add timestamp
        raw_imu_data.timestamp = esp_timer_get_time();

        // Map axis to match the expected orientation
        mapped_imu_data.accel_x = -raw_imu_data.accel_z;
        mapped_imu_data.accel_y = raw_imu_data.accel_x;
        mapped_imu_data.accel_z = raw_imu_data.accel_y;
        mapped_imu_data.gyro_x = -raw_imu_data.gyro_z;
        mapped_imu_data.gyro_y = raw_imu_data.gyro_x;
        mapped_imu_data.gyro_z = raw_imu_data.gyro_y;
        mapped_imu_data.timestamp = raw_imu_data.timestamp;

        // Perform gyro calibration if not completed
        if (calibration_status != IMU_CALIBRATION_COMPLETED)
        {
            add_calibration_sample(mapped_imu_data.gyro_x, mapped_imu_data.gyro_y, mapped_imu_data.gyro_z);
        }

        // Apply gyro bias correction if calibration is completed
        if (calibration_status == IMU_CALIBRATION_COMPLETED)
        {
            mapped_imu_data.gyro_x -= gyro_bias_x;
            mapped_imu_data.gyro_y -= gyro_bias_y;
            mapped_imu_data.gyro_z -= gyro_bias_z;
        }

        // Apply software low-pass filter
        apply_software_filter(&mapped_imu_data, &filtered_data);

        // Send filtered data to queue (overwrite if queue is full - we only want the latest data)
        if (xQueueOverwrite(imu_data_queue, &filtered_data) != pdTRUE)
        {
            ESP_LOGW(TAG, "Failed to send IMU data to queue");
        }

        // Wait for next reading
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

float imu_calculate_g_load_factor(float accel_x_mg, float accel_y_mg, float accel_z_mg)
{
    // Convert milli-g to g by dividing by 1000
    float accel_x_g = accel_x_mg / 1000.0f;
    float accel_y_g = accel_y_mg / 1000.0f;
    float accel_z_g = accel_z_mg / 1000.0f;

    // Calculate magnitude (total G-force load factor)
    return sqrtf(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
}

float imu_calculate_turn_rate(float gyro_x_degs, float gyro_y_degs, float gyro_z_degs)
{
    // Aircraft turn indicators display turn rate around the yaw axis (Z-axis)
    // This represents horizontal turning (left/right turns)
    // Aircraft coordinate system: X=forward(roll), Y=right(pitch), Z=down(yaw)
    // Gyroscope data is already in deg/s, so return Z-axis directly
    // Positive values indicate right turn, negative values indicate left turn
    return gyro_z_degs;
}

esp_err_t imu_init(QueueHandle_t imu_queue)
{
    if (imu_initialized)
    {
        ESP_LOGW(TAG, "IMU already initialized");
        return ESP_OK;
    }

    if (imu_queue == NULL)
    {
        ESP_LOGE(TAG, "Invalid queue handle");
        return ESP_ERR_INVALID_ARG;
    }

    imu_data_queue = imu_queue;

    // Runtime parameter validation
    if (FILTER_ALPHA < 0.0f || FILTER_ALPHA > 1.0f)
    {
        ESP_LOGE(TAG, "Invalid FILTER_ALPHA (%.3f). Must be between 0.0 and 1.0", FILTER_ALPHA);
        return ESP_ERR_INVALID_ARG;
    }

    // Get I2C bus handle
    i2c_master_bus_handle_t bus_handle = bsp_i2c_get_handle();

    // Initialize QMI8658
    esp_err_t ret = qmi8658_init(&dev, bus_handle, QMI8658_ADDRESS_HIGH);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize QMI8658: %d", ret);
        return ret;
    }

    // Read chip identification (silent check)
    uint8_t who_am_i, revision;
    qmi8658_get_who_am_i(&dev, &who_am_i);
    qmi8658_read_register(&dev, QMI8658_REVISION, &revision, 1);
    ESP_LOGD(TAG, "QMI8658 ID: 0x%02X, Revision: 0x%02X", who_am_i, revision);

    // Configure accelerometer
    ret = qmi8658_set_accel_range(&dev, QMI8658_ACCEL_RANGE_8G);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set accelerometer range: %d", ret);
        return ret;
    }

    ret = qmi8658_set_accel_odr(&dev, QMI8658_ACCEL_ODR_125HZ);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set accelerometer ODR: %d", ret);
        return ret;
    }

    qmi8658_set_accel_unit_mg(&dev, true);

    // Configure gyroscope
    ret = qmi8658_set_gyro_range(&dev, QMI8658_GYRO_RANGE_512DPS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set gyroscope range: %d", ret);
        return ret;
    }

    ret = qmi8658_set_gyro_odr(&dev, QMI8658_GYRO_ODR_125HZ);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set gyroscope ODR: %d", ret);
        return ret;
    }

    qmi8658_set_gyro_unit_dps(&dev, true);

    // Set display precision
    qmi8658_set_display_precision(&dev, 4);

    // Initialize software filter
    filter_initialized = false;
    ESP_LOGI(TAG, "Software low-pass filter: alpha=%.3f, accel=%s, gyro=%s",
             FILTER_ALPHA,
             FILTER_ENABLE_ACCEL ? "enabled" : "disabled",
             FILTER_ENABLE_GYRO ? "enabled" : "disabled");
    ESP_LOGI(TAG, "Aircraft coordinate system: X=forward(roll), Y=right(pitch), Z=down(yaw)");

    // Initialize calibration state
    calibration_status = IMU_CALIBRATION_NOT_STARTED;
    calibration_sample_count = 0;
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;
    gyro_sum_x = 0.0f;
    gyro_sum_y = 0.0f;
    gyro_sum_z = 0.0f;
    current_batch = NULL;
    batch_index = 0;
    calibration_log_counter = 0;

    // Create calibration queue
    calibration_queue = xQueueCreate(CALIBRATION_QUEUE_SIZE, sizeof(calibration_batch_t *));
    if (calibration_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create calibration queue");
        return ESP_ERR_NO_MEM;
    }

    // Create calibration processing task
    BaseType_t cal_task_ret = xTaskCreate(
        calibration_task,
        "calibration_task",
        CALIBRATION_STACK_SIZE,                                  // Stack size from define
        NULL,                                                    // Parameters
        configMAX_PRIORITIES - CALIBRATION_TASK_PRIORITY_OFFSET, // Priority from define
        &calibration_task_handle);

    if (cal_task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create calibration task");
        vQueueDelete(calibration_queue);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Gyro calibration will start automatically");

    // Create IMU reading task
    BaseType_t task_ret = xTaskCreate(
        imu_task,
        "imu_task",
        IMU_STACK_SIZE,                                  // Stack size from define
        NULL,                                            // Parameters
        configMAX_PRIORITIES - IMU_TASK_PRIORITY_OFFSET, // Priority from define
        &imu_task_handle);

    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_ERR_NO_MEM;
    }

    imu_initialized = true;
    ESP_LOGI(TAG, "IMU initialized successfully");

    return ESP_OK;
}

void imu_deinit(void)
{
    if (!imu_initialized)
    {
        return;
    }

    // Delete calibration task if it exists
    if (calibration_task_handle != NULL)
    {
        vTaskDelete(calibration_task_handle);
        calibration_task_handle = NULL;
    }

    // Delete IMU task if it exists
    if (imu_task_handle != NULL)
    {
        vTaskDelete(imu_task_handle);
        imu_task_handle = NULL;
    }

    // Delete calibration queue if it exists
    if (calibration_queue != NULL)
    {
        vQueueDelete(calibration_queue);
        calibration_queue = NULL;
    }

    // Free current batch if allocated
    if (current_batch != NULL)
    {
        heap_caps_free(current_batch);
        current_batch = NULL;
    }

    imu_data_queue = NULL;
    imu_initialized = false;
    filter_initialized = false;

    // Reset calibration state
    calibration_status = IMU_CALIBRATION_NOT_STARTED;
    calibration_sample_count = 0;
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;
    gyro_sum_x = 0.0f;
    gyro_sum_y = 0.0f;
    gyro_sum_z = 0.0f;
    batch_index = 0;
    calibration_log_counter = 0;

    ESP_LOGI(TAG, "IMU deinitialized");
}

imu_calibration_status_t imu_get_calibration_status(void)
{
    return calibration_status;
}