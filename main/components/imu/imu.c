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
#define FILTER_ALPHA 0.15f // Filter coefficient (0.0 = no filtering, 1.0 = no smoothing)

// Madgwick filter parameters
#define MADGWICK_BETA 0.1f          // Filter gain (configurable)
#define MADGWICK_SAMPLE_FREQ 31.25f // Effective sample frequency in Hz (125/4)
#define MADGWICK_DECIMATION 4       // Run Madgwick every 4th sample (31.25Hz)

// Gyro calibration parameters
#define CALIBRATION_SAMPLES 2000      // Number of samples for gyro bias calibration
#define CALIBRATION_TIMEOUT_MS 20000  // Maximum time for calibration (20 seconds)
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

// Madgwick filter state
static float madgwick_q0 = 1.0f, madgwick_q1 = 0.0f, madgwick_q2 = 0.0f, madgwick_q3 = 0.0f; // Quaternion components
static bool madgwick_initialized = false;
static uint32_t madgwick_sample_counter = 0;
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
        ESP_LOGI(TAG, "Calibration progress: %ld/%d samples", calibration_sample_count, CALIBRATION_SAMPLES);
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
        ESP_LOGI(TAG, "Gyro bias: X=%.6f, Y=%.6f, Z=%.6f rad/s",
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
 * @brief Fast inverse square root function
 *
 * Optimized implementation for quaternion normalization
 */
static float fast_inv_sqrt(float x)
{
    // Use standard sqrt for ESP32-S3 (has hardware FPU)
    return 1.0f / sqrtf(x);
}

/**
 * @brief Madgwick AHRS algorithm implementation (6DOF - without magnetometer)
 *
 * @param gx Gyroscope X-axis (rad/s)
 * @param gy Gyroscope Y-axis (rad/s)
 * @param gz Gyroscope Z-axis (rad/s)
 * @param ax Accelerometer X-axis (normalized)
 * @param ay Accelerometer Y-axis (normalized)
 * @param az Accelerometer Z-axis (normalized)
 */
static void madgwick_update_6dof(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Sample period
    float samplePeriod = 1.0f / MADGWICK_SAMPLE_FREQ;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-madgwick_q1 * gx - madgwick_q2 * gy - madgwick_q3 * gz);
    qDot2 = 0.5f * (madgwick_q0 * gx + madgwick_q2 * gz - madgwick_q3 * gy);
    qDot3 = 0.5f * (madgwick_q0 * gy - madgwick_q1 * gz + madgwick_q3 * gx);
    qDot4 = 0.5f * (madgwick_q0 * gz + madgwick_q1 * gy - madgwick_q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * madgwick_q0;
        _2q1 = 2.0f * madgwick_q1;
        _2q2 = 2.0f * madgwick_q2;
        _2q3 = 2.0f * madgwick_q3;
        _4q0 = 4.0f * madgwick_q0;
        _4q1 = 4.0f * madgwick_q1;
        _4q2 = 4.0f * madgwick_q2;
        _8q1 = 8.0f * madgwick_q1;
        _8q2 = 8.0f * madgwick_q2;
        q0q0 = madgwick_q0 * madgwick_q0;
        q1q1 = madgwick_q1 * madgwick_q1;
        q2q2 = madgwick_q2 * madgwick_q2;
        q3q3 = madgwick_q3 * madgwick_q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * madgwick_q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * madgwick_q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * madgwick_q3 - _2q1 * ax + 4.0f * q2q2 * madgwick_q3 - _2q2 * ay;
        recipNorm = fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= MADGWICK_BETA * s0;
        qDot2 -= MADGWICK_BETA * s1;
        qDot3 -= MADGWICK_BETA * s2;
        qDot4 -= MADGWICK_BETA * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    madgwick_q0 += qDot1 * samplePeriod;
    madgwick_q1 += qDot2 * samplePeriod;
    madgwick_q2 += qDot3 * samplePeriod;
    madgwick_q3 += qDot4 * samplePeriod;

    // Normalise quaternion
    recipNorm = fast_inv_sqrt(madgwick_q0 * madgwick_q0 + madgwick_q1 * madgwick_q1 + madgwick_q2 * madgwick_q2 + madgwick_q3 * madgwick_q3);
    madgwick_q0 *= recipNorm;
    madgwick_q1 *= recipNorm;
    madgwick_q2 *= recipNorm;
    madgwick_q3 *= recipNorm;
}

/**
 * @brief Initialize Madgwick filter with initial orientation from accelerometer
 *
 * @param ax Accelerometer X-axis (mg)
 * @param ay Accelerometer Y-axis (mg)
 * @param az Accelerometer Z-axis (mg)
 */
static void madgwick_init_orientation(float ax, float ay, float az)
{
    if (madgwick_initialized)
    {
        return;
    }

    // Normalize accelerometer reading
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
    {
        // Use identity quaternion if invalid accelerometer data
        madgwick_q0 = 1.0f;
        madgwick_q1 = 0.0f;
        madgwick_q2 = 0.0f;
        madgwick_q3 = 0.0f;
    }
    else
    {
        ax /= norm;
        ay /= norm;
        az /= norm;

        // Calculate initial quaternion from accelerometer (assuming device is level)
        // This gives us the initial tilt with respect to gravity
        float pitch = asinf(-ax);
        float roll = atan2f(ay, az);

        // Convert to quaternion (yaw = 0)
        float cy = cosf(0 * 0.5f); // yaw/2
        float sy = sinf(0 * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);

        madgwick_q0 = cy * cp * cr + sy * sp * sr;
        madgwick_q1 = cy * cp * sr - sy * sp * cr;
        madgwick_q2 = sy * cp * sr + cy * sp * cr;
        madgwick_q3 = sy * cp * cr - cy * sp * sr;
    }

    madgwick_initialized = true;
}

/**
 * @brief Apply software low-pass filter to IMU data and perform Madgwick fusion
 *
 * Uses a simple exponential moving average filter for sensor data
 * and applies Madgwick AHRS algorithm for quaternion calculation
 * filtered_value = alpha * new_value + (1 - alpha) * old_value
 */
static void apply_software_filter(imu_data_t *raw_data, imu_data_t *filtered_output)
{
    if (!filter_initialized)
    {
        // Initialize filter with first sample
        *filtered_output = *raw_data;
        filter_initialized = true;

        // Initialize Madgwick filter with first accelerometer reading
        madgwick_init_orientation(raw_data->accel_x, raw_data->accel_y, raw_data->accel_z);

        // Set initial quaternion in output
        filtered_output->quat_w = madgwick_q0;
        filtered_output->quat_x = madgwick_q1;
        filtered_output->quat_y = madgwick_q2;
        filtered_output->quat_z = madgwick_q3;
        return;
    }

    // Apply exponential moving average filter
    filtered_output->accel_x = FILTER_ALPHA * raw_data->accel_x + (1.0f - FILTER_ALPHA) * filtered_output->accel_x;
    filtered_output->accel_y = FILTER_ALPHA * raw_data->accel_y + (1.0f - FILTER_ALPHA) * filtered_output->accel_y;
    filtered_output->accel_z = FILTER_ALPHA * raw_data->accel_z + (1.0f - FILTER_ALPHA) * filtered_output->accel_z;

    filtered_output->gyro_x = FILTER_ALPHA * raw_data->gyro_x + (1.0f - FILTER_ALPHA) * filtered_output->gyro_x;
    filtered_output->gyro_y = FILTER_ALPHA * raw_data->gyro_y + (1.0f - FILTER_ALPHA) * filtered_output->gyro_y;
    filtered_output->gyro_z = FILTER_ALPHA * raw_data->gyro_z + (1.0f - FILTER_ALPHA) * filtered_output->gyro_z;

    // Apply Madgwick fusion algorithm only if calibration is completed
    if (madgwick_initialized && calibration_status == IMU_CALIBRATION_COMPLETED)
    {
        // Increment counter and only run Madgwick every N samples for performance
        madgwick_sample_counter++;
        if (madgwick_sample_counter >= MADGWICK_DECIMATION)
        {
            madgwick_sample_counter = 0;

            // Madgwick expects normalized accelerometer data (mg values work directly)
            // No unit conversion needed - Madgwick normalizes internally
            madgwick_update_6dof(filtered_output->gyro_x, filtered_output->gyro_y, filtered_output->gyro_z,
                                 filtered_output->accel_x, filtered_output->accel_y, filtered_output->accel_z);
        }

        // Always update quaternion in output (keeps smooth interpolation)
        filtered_output->quat_w = madgwick_q0;
        filtered_output->quat_x = madgwick_q1;
        filtered_output->quat_y = madgwick_q2;
        filtered_output->quat_z = madgwick_q3;
    }
    else
    {
        // Keep previous quaternion values if calibration not completed
        // (or set to identity if first run)
        if (!madgwick_initialized)
        {
            filtered_output->quat_w = 1.0f;
            filtered_output->quat_x = 0.0f;
            filtered_output->quat_y = 0.0f;
            filtered_output->quat_z = 0.0f;
        }
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

        // Map axis (1:1 mapping for now - modify as needed)
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

    qmi8658_set_gyro_unit_rads(&dev, true);

    // Set display precision
    qmi8658_set_display_precision(&dev, 4);

    // Initialize software filter
    filter_initialized = false;
    ESP_LOGI(TAG, "Software low-pass filter enabled (alpha = %.2f)", FILTER_ALPHA);

    // Initialize Madgwick filter state
    madgwick_q0 = 1.0f;
    madgwick_q1 = 0.0f;
    madgwick_q2 = 0.0f;
    madgwick_q3 = 0.0f;
    madgwick_initialized = false;
    madgwick_sample_counter = 0;
    ESP_LOGI(TAG, "Madgwick AHRS filter enabled (beta = %.2f, freq = %.1f Hz)", MADGWICK_BETA, MADGWICK_SAMPLE_FREQ);

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
        4096,                     // Stack size
        NULL,                     // Parameters
        configMAX_PRIORITIES - 2, // Priority (lower than IMU task)
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
        4096,                     // Stack size
        NULL,                     // Parameters
        configMAX_PRIORITIES - 1, // Priority (higher than default)
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

    // Reset Madgwick filter state
    madgwick_q0 = 1.0f;
    madgwick_q1 = 0.0f;
    madgwick_q2 = 0.0f;
    madgwick_q3 = 0.0f;
    madgwick_initialized = false;
    madgwick_sample_counter = 0;

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