#ifndef IMU_H
#define IMU_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief IMU calibration status
     */
    typedef enum
    {
        IMU_CALIBRATION_NOT_STARTED = 0,
        IMU_CALIBRATION_IN_PROGRESS,
        IMU_CALIBRATION_COMPLETED
    } imu_calibration_status_t;

    /**
     * @brief Euler angles structure containing pitch, yaw, and roll in radians
     */
    typedef struct
    {
        float pitch; // Pitch angle in radians (rotation around Y-axis)
        float yaw;   // Yaw angle in radians (rotation around Z-axis)
        float roll;  // Roll angle in radians (rotation around X-axis)
    } imu_euler_angles_t;

    /**
     * @brief IMU data structure containing accelerometer, gyroscope, quaternion and timestamp
     */
    typedef struct
    {
        float accel_x;                               // Accelerometer X-axis (mg)
        float accel_y;                               // Accelerometer Y-axis (mg)
        float accel_z;                               // Accelerometer Z-axis (mg)
        float gyro_x;                                // Gyroscope X-axis (rad/s) - bias corrected
        float gyro_y;                                // Gyroscope Y-axis (rad/s) - bias corrected
        float gyro_z;                                // Gyroscope Z-axis (rad/s) - bias corrected
        float quat_w;                                // Quaternion W component (scalar)
        float quat_x;                                // Quaternion X component (vector)
        float quat_y;                                // Quaternion Y component (vector)
        float quat_z;                                // Quaternion Z component (vector)
        int64_t timestamp;                           // Timestamp in microseconds
        imu_calibration_status_t calibration_status; // Gyro calibration status
    } imu_data_t;

    /**
     * @brief Initialize the IMU module
     *
     * This function initializes the QMI8658 IMU sensor with the following configuration:
     * - Accelerometer: 8G range, 125Hz ODR, mg units
     * - Gyroscope: 512DPS range, 125Hz ODR, rad/s units
     * - 4 decimal precision for display
     * - Gyro bias calibration will start automatically (2000 samples)
     *
     * @param imu_queue Queue handle to send IMU data to
     * @return esp_err_t ESP_OK on success, error code otherwise
     */
    esp_err_t imu_init(QueueHandle_t imu_queue);

    /**
     * @brief Deinitialize the IMU module
     *
     * Stops the IMU reading task and cleans up resources
     */
    void imu_deinit(void);

    /**
     * @brief Get current gyro calibration status
     *
     * @return imu_calibration_status_t Current calibration status
     */
    imu_calibration_status_t imu_get_calibration_status(void);

    /**
     * @brief Convert quaternion to Euler angles
     *
     * Converts a quaternion (w, x, y, z) to Euler angles (pitch, yaw, roll) in radians
     * using ZYX Tait-Bryan convention
     *
     * @param quat_w Quaternion W component (scalar)
     * @param quat_x Quaternion X component (vector)
     * @param quat_y Quaternion Y component (vector)
     * @param quat_z Quaternion Z component (vector)
     * @return imu_euler_angles_t Euler angles in radians
     */
    imu_euler_angles_t imu_quaternion_to_euler(float quat_w, float quat_x, float quat_y, float quat_z);

    /**
     * @brief Calculate total G-force load factor from accelerometer readings
     *
     * Calculates the total G-force magnitude from X, Y, Z accelerometer readings
     * in milli-g units, returning the load factor in G units
     *
     * @param accel_x_mg Accelerometer X-axis reading in milli-g
     * @param accel_y_mg Accelerometer Y-axis reading in milli-g
     * @param accel_z_mg Accelerometer Z-axis reading in milli-g
     * @return float Total G-force load factor
     */
    float imu_calculate_g_load_factor(float accel_x_mg, float accel_y_mg, float accel_z_mg);

#ifdef __cplusplus
}
#endif

#endif // IMU_H