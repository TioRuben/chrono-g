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
     * @brief IMU data structure containing accelerometer, gyroscope and timestamp
     */
    typedef struct
    {
        float accel_x;                               // Accelerometer X-axis (mg)
        float accel_y;                               // Accelerometer Y-axis (mg)
        float accel_z;                               // Accelerometer Z-axis (mg)
        float gyro_x;                                // Gyroscope X-axis (deg/s) - bias corrected
        float gyro_y;                                // Gyroscope Y-axis (deg/s) - bias corrected
        float gyro_z;                                // Gyroscope Z-axis (deg/s) - bias corrected
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
     * @brief Calculate aircraft G-force load factor from Z-axis accelerometer reading
     *
     * Returns the signed Z-axis acceleration as the G-force load factor.
     * In aircraft coordinate system (Z=down):
     * - +1G = normal upright flight (gravity pulling down)
     * - -1G = inverted flight (gravity pulling up relative to aircraft)
     * - >1G = pulling up (positive load factor)
     * - <-1G = pushing down while inverted (negative load factor)
     *
     * @param accel_x_mg Accelerometer X-axis reading in milli-g (unused for G-force)
     * @param accel_y_mg Accelerometer Y-axis reading in milli-g (unused for G-force)
     * @param accel_z_mg Accelerometer Z-axis reading in milli-g (primary axis for G-force)
     * @return float Signed G-force load factor (can be negative for inverted flight)
     */
    float imu_calculate_g_load_factor(float accel_x_mg, float accel_y_mg, float accel_z_mg);

    /**
     * @brief Calculate total acceleration magnitude from accelerometer readings
     *
     * Calculates the total acceleration magnitude from X, Y, Z accelerometer readings
     * in milli-g units, returning the magnitude in G units. Always positive.
     * Useful for detecting motion, vibration, or total acceleration regardless of direction.
     *
     * @param accel_x_mg Accelerometer X-axis reading in milli-g
     * @param accel_y_mg Accelerometer Y-axis reading in milli-g
     * @param accel_z_mg Accelerometer Z-axis reading in milli-g
     * @return float Total acceleration magnitude (always positive)
     */
    float imu_calculate_total_acceleration(float accel_x_mg, float accel_y_mg, float accel_z_mg);

    /**
     * @brief Calculate aircraft turn rate like a standard turn indicator
     *
     * Returns the yaw axis (Z-axis) gyroscope reading directly as the turn rate.
     * Aircraft turn indicators display turn rate in degrees per second.
     * Standard rate turn in aviation is 3°/s (360° turn in 2 minutes).
     *
     * Aircraft coordinate system: X=forward(roll), Y=right(pitch), Z=down(yaw)
     *
     * @param gyro_x_degs Gyroscope X-axis reading in deg/s (roll rate)
     * @param gyro_y_degs Gyroscope Y-axis reading in deg/s (pitch rate)
     * @param gyro_z_degs Gyroscope Z-axis reading in deg/s (yaw rate)
     * @return float Turn rate in degrees per second (positive = right turn, negative = left turn)
     */
    float imu_calculate_turn_rate(float gyro_x_degs, float gyro_y_degs, float gyro_z_degs);
#ifdef __cplusplus
}
#endif

#endif // IMU_H