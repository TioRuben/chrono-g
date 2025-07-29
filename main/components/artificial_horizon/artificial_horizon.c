#include "artificial_horizon.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "qmi8658.h"
#include "lvgl.h"
#include <math.h>

static const char *TAG = "ArtificialHorizon";

// Static variables for sensor logging
static qmi8658_dev_t *g_imu_dev = NULL;
static esp_timer_handle_t g_sensor_log_timer = NULL;

// Conversion constants
#define DEG_TO_RAD (M_PI / 180.0f) ///< Degrees to radians conversion
#define G_TO_MPS2 (9.81f)          ///< Standard gravity in m/s²

/**
 * @brief Map QMI8658 sensor readings to aircraft coordinate system
 *
 * Maps the raw sensor readings to standard aircraft coordinates for VERTICAL MOUNTING on instrument panel.
 * Device orientation: Screen facing pilot, mounted vertically like a traditional attitude indicator.
 *
 * ESP-DSP EKF coordinate convention:
 * - Aircraft X-axis: Forward (nose direction)
 * - Aircraft Y-axis: Right wing direction
 * - Aircraft Z-axis: Down (gravity direction in level flight = +1g)
 *
 * Based on sensor analysis for vertical mounting:
 * - Level flight: QMI8658 Y≈10g (pointing UP) → Aircraft Z should be +1g (DOWN)
 * - Nose down: QMI8658 Z≈-9.8g (pointing FORWARD) → Aircraft X should be +1g
 * - Left bank: QMI8658 X≈-9.5g (pointing RIGHT) → Aircraft Y should be +1g
 *
 * @param[in] qmi_accel Raw accelerometer data from QMI8658 [g]
 * @param[in] qmi_gyro Raw gyroscope data from QMI8658 [dps]
 * @param[out] aircraft_accel Mapped accelerometer data [g] in aircraft coordinates
 * @param[out] aircraft_gyro Mapped gyroscope data [rad/s] in aircraft coordinates
 */
static void map_sensor_to_aircraft_coordinates(
    const float qmi_accel[3], const float qmi_gyro[3],
    float aircraft_accel[3], float aircraft_gyro[3])
{
    // Map accelerometer readings to aircraft coordinates for VERTICAL mounting
    //
    // Analysis of QMI8658 readings (device mounted vertically on instrument panel):
    // - Level flight: X=-0.104, Y=10.135, Z=0.130 → Target: [0, 0, 1] aircraft
    //   * Y≈10g means QMI8658 Y-axis points UP, so Aircraft Z (DOWN) = -QMI8658_Y
    //
    // - Nose down: X=0.220, Y=0.385, Z=-9.758 → Target: [1, 0, 0] aircraft
    //   * Z≈-9.8g means QMI8658 Z-axis points FORWARD, so Aircraft X (FWD) = -QMI8658_Z
    //
    // - Left bank: X=-9.523, Y=-0.150, Z=0.297 → Target: [0, 1, 0] aircraft
    //   * X≈-9.5g means QMI8658 X-axis points RIGHT, so Aircraft Y (RIGHT) = -QMI8658_X

    aircraft_accel[0] = -qmi_accel[2]; // Aircraft forward (X) = -QMI8658 Z-axis
    aircraft_accel[1] = -qmi_accel[0]; // Aircraft right (Y) = -QMI8658 X-axis
    aircraft_accel[2] = qmi_accel[1];  // Aircraft down (Z) = QMI8658 Y-axis (positive Y = positive down)

    // Map gyroscope readings to aircraft coordinates and convert to rad/s
    // Same axis mapping as accelerometer, with unit conversion
    aircraft_gyro[0] = -qmi_gyro[2] * DEG_TO_RAD; // Aircraft pitch rate = -QMI8658 Z-axis [rad/s]
    aircraft_gyro[1] = -qmi_gyro[0] * DEG_TO_RAD; // Aircraft roll rate = -QMI8658 X-axis [rad/s]
    aircraft_gyro[2] = qmi_gyro[1] * DEG_TO_RAD;  // Aircraft yaw rate = QMI8658 Y-axis [rad/s]
}

/**
 * @brief Prepare sensor data for ESP-DSP EKF input
 *
 * Converts mapped aircraft sensor data to the units and format expected by the EKF:
 * - Accelerometer: [g] units (1g = 9.81 m/s²) normalized to unit gravity
 * - Gyroscope: [rad/s] units
 * - Apply any additional filtering or bias correction if needed
 *
 * The ESP-DSP EKF expects accelerometer data to be normalized such that:
 * - Level flight reads approximately [0, 0, 1]
 * - The magnitude represents the gravity vector direction
 *
 * @param[in] aircraft_accel Aircraft accelerometer data [g]
 * @param[in] aircraft_gyro Aircraft gyroscope data [rad/s]
 * @param[out] ekf_accel EKF-ready accelerometer data [g]
 * @param[out] ekf_gyro EKF-ready gyroscope data [rad/s]
 */
static void prepare_ekf_sensor_data(
    const float aircraft_accel[3], const float aircraft_gyro[3],
    float ekf_accel[3], float ekf_gyro[3])
{
    // ESP-DSP EKF expects:
    // - Accelerometer in [g] units, normalized to unit gravity magnitude
    // - Gyroscope in [rad/s] units

    // Normalize accelerometer data to unit gravity magnitude
    float accel_magnitude = sqrtf(aircraft_accel[0] * aircraft_accel[0] +
                                  aircraft_accel[1] * aircraft_accel[1] +
                                  aircraft_accel[2] * aircraft_accel[2]);

    if (accel_magnitude > 0.1f)
    { // Avoid division by zero
        for (int i = 0; i < 3; i++)
        {
            ekf_accel[i] = aircraft_accel[i] / accel_magnitude;
        }
    }
    else
    {
        // Fallback to default gravity vector if magnitude is too small
        ekf_accel[0] = 0.0f;
        ekf_accel[1] = 0.0f;
        ekf_accel[2] = 1.0f;
    }

    // Copy gyroscope data (already in [rad/s] units)
    for (int i = 0; i < 3; i++)
    {
        ekf_gyro[i] = aircraft_gyro[i];
    }

    // Note: Additional processing can be added here:
    // - Bias correction using EKF gyro bias estimates
    // - Low-pass filtering for noise reduction
    // - Temperature compensation
    // - Calibration matrix application
}

/**
 * @brief Timer callback function to log sensor data every 5 seconds
 *
 * This function reads raw accelerometer and gyroscope data from the QMI8658,
 * applies the aircraft coordinate mapping, and logs both raw and processed values
 * to help with verification and debugging.
 */
static void sensor_log_timer_callback(void *arg)
{
    if (g_imu_dev == NULL)
    {
        ESP_LOGW(TAG, "IMU device not initialized");
        return;
    }

    qmi8658_data_t sensor_data;
    esp_err_t ret = qmi8658_read_sensor_data(g_imu_dev, &sensor_data);

    if (ret == ESP_OK)
    {
        // Raw QMI8658 sensor data
        float qmi_accel[3] = {sensor_data.accelX, sensor_data.accelY, sensor_data.accelZ};
        float qmi_gyro[3] = {sensor_data.gyroX, sensor_data.gyroY, sensor_data.gyroZ};

        // Mapped aircraft coordinate data
        float aircraft_accel[3], aircraft_gyro[3];
        map_sensor_to_aircraft_coordinates(qmi_accel, qmi_gyro, aircraft_accel, aircraft_gyro);

        // EKF-ready data
        float ekf_accel[3], ekf_gyro[3];
        prepare_ekf_sensor_data(aircraft_accel, aircraft_gyro, ekf_accel, ekf_gyro);

        ESP_LOGI(TAG, "=== SENSOR DATA ANALYSIS ===");
        ESP_LOGI(TAG, "RAW QMI8658:");
        ESP_LOGI(TAG, "  Accel [g]:  X=%.3f  Y=%.3f  Z=%.3f",
                 qmi_accel[0], qmi_accel[1], qmi_accel[2]);
        ESP_LOGI(TAG, "  Gyro [dps]: X=%.2f  Y=%.2f  Z=%.2f",
                 qmi_gyro[0], qmi_gyro[1], qmi_gyro[2]);
        ESP_LOGI(TAG, "AIRCRAFT COORDINATES:");
        ESP_LOGI(TAG, "  Accel [g]:  Fwd=%.3f  Right=%.3f  Down=%.3f",
                 aircraft_accel[0], aircraft_accel[1], aircraft_accel[2]);
        ESP_LOGI(TAG, "  Gyro [rad/s]: Pitch=%.4f  Roll=%.4f  Yaw=%.4f",
                 aircraft_gyro[0], aircraft_gyro[1], aircraft_gyro[2]);
        ESP_LOGI(TAG, "EKF INPUT (units verified):");
        ESP_LOGI(TAG, "  Accel [g]:  X=%.3f  Y=%.3f  Z=%.3f",
                 ekf_accel[0], ekf_accel[1], ekf_accel[2]);
        ESP_LOGI(TAG, "  Gyro [rad/s]: X=%.4f  Y=%.4f  Z=%.4f",
                 ekf_gyro[0], ekf_gyro[1], ekf_gyro[2]);
        ESP_LOGI(TAG, "Temperature: %.1f°C", sensor_data.temperature);
        ESP_LOGI(TAG, "============================");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
    }
}

void artificial_horizon_init(lv_obj_t *parent, qmi8658_dev_t *imu_dev)
{
    ESP_LOGI(TAG, "Initializing artificial horizon");

    if (imu_dev == NULL)
    {
        ESP_LOGE(TAG, "IMU device pointer is NULL");
        return;
    }

    // Store the IMU device reference
    g_imu_dev = imu_dev;

    // Configure QMI8658 sensor
    ESP_LOGI(TAG, "Configuring QMI8658 IMU sensor...");

    // Check if sensor is communicating
    uint8_t who_am_i;
    esp_err_t ret = qmi8658_get_who_am_i(imu_dev, &who_am_i);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "QMI8658 WHO_AM_I: 0x%02X", who_am_i);

    // Configure accelerometer
    ret = qmi8658_set_accel_range(imu_dev, QMI8658_ACCEL_RANGE_4G); // ±4g range
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set accelerometer range: %s", esp_err_to_name(ret));
        return;
    }

    ret = qmi8658_set_accel_odr(imu_dev, QMI8658_ACCEL_ODR_125HZ); // 125Hz output rate
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set accelerometer ODR: %s", esp_err_to_name(ret));
        return;
    }

    // Configure gyroscope
    ret = qmi8658_set_gyro_range(imu_dev, QMI8658_GYRO_RANGE_1024DPS); // ±1024 dps range
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set gyroscope range: %s", esp_err_to_name(ret));
        return;
    }

    ret = qmi8658_set_gyro_odr(imu_dev, QMI8658_GYRO_ODR_125HZ); // 125Hz output rate
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set gyroscope ODR: %s", esp_err_to_name(ret));
        return;
    }

    // Set units for sensor readings
    qmi8658_set_accel_unit_mg(imu_dev, false); // Use g units (not mg)
    qmi8658_set_gyro_unit_dps(imu_dev, true);  // Use degrees per second
    qmi8658_set_display_precision(imu_dev, 3); // 3 decimal places

    // Enable accelerometer and gyroscope
    ret = qmi8658_enable_sensors(imu_dev, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable sensors: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "QMI8658 sensor configured successfully");
    ESP_LOGI(TAG, "Accelerometer: ±4g range, 125Hz ODR, units in [g]");
    ESP_LOGI(TAG, "Gyroscope: ±1024dps range, 125Hz ODR, units in [dps]");

    // Create timer for periodic sensor data logging (every 5 seconds)
    const esp_timer_create_args_t timer_args = {
        .callback = &sensor_log_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sensor_log_timer",
        .skip_unhandled_events = true};

    ret = esp_timer_create(&timer_args, &g_sensor_log_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create sensor log timer: %s", esp_err_to_name(ret));
        return;
    }

    // Start the timer (5000000 microseconds = 5 seconds)
    ret = esp_timer_start_periodic(g_sensor_log_timer, 5000000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start sensor log timer: %s", esp_err_to_name(ret));
        esp_timer_delete(g_sensor_log_timer);
        g_sensor_log_timer = NULL;
        return;
    }

    ESP_LOGI(TAG, "Sensor data logging started (every 5 seconds)");
    ESP_LOGI(TAG, "=== AXIS MAPPING GUIDE ===");
    ESP_LOGI(TAG, "Position the device in different orientations and note the sensor values:");
    ESP_LOGI(TAG, "1. Flat on table (screen up) - note which axis reads ~1g");
    ESP_LOGI(TAG, "2. Vertical (screen facing you) - note axis changes");
    ESP_LOGI(TAG, "3. Rotated left/right - note which axis corresponds to roll");
    ESP_LOGI(TAG, "4. Tilted forward/back - note which axis corresponds to pitch");
    ESP_LOGI(TAG, "=========================");
}

// Function to de-initialize the artificial horizon if needed
void artificial_horizon_deinit(void)
{
    ESP_LOGI(TAG, "De-initializing artificial horizon");

    // Stop and delete the sensor log timer
    if (g_sensor_log_timer != NULL)
    {
        esp_timer_stop(g_sensor_log_timer);
        esp_timer_delete(g_sensor_log_timer);
        g_sensor_log_timer = NULL;
        ESP_LOGI(TAG, "Sensor log timer stopped and deleted");
    }

    // Clear IMU device reference
    g_imu_dev = NULL;
}

void artificial_horizon_set_visible(bool visible)
{
    ESP_LOGI(TAG, "Setting artificial horizon visibility to %s", visible ? "true" : "false");
    // Here you would typically show/hide the LVGL object representing the horizon
    // For example: lv_obj_set_hidden(horizon_obj, !visible);
}

esp_err_t artificial_horizon_read_sensor_data(float ekf_accel[3], float ekf_gyro[3])
{
    if (g_imu_dev == NULL)
    {
        ESP_LOGE(TAG, "IMU device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ekf_accel == NULL || ekf_gyro == NULL)
    {
        ESP_LOGE(TAG, "Output arrays cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Read raw sensor data
    qmi8658_data_t sensor_data;
    esp_err_t ret = qmi8658_read_sensor_data(g_imu_dev, &sensor_data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Raw QMI8658 sensor data
    float qmi_accel[3] = {sensor_data.accelX, sensor_data.accelY, sensor_data.accelZ};
    float qmi_gyro[3] = {sensor_data.gyroX, sensor_data.gyroY, sensor_data.gyroZ};

    // Map to aircraft coordinates
    float aircraft_accel[3], aircraft_gyro[3];
    map_sensor_to_aircraft_coordinates(qmi_accel, qmi_gyro, aircraft_accel, aircraft_gyro);

    // Prepare for EKF input
    prepare_ekf_sensor_data(aircraft_accel, aircraft_gyro, ekf_accel, ekf_gyro);

    return ESP_OK;
}