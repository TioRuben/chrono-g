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
        float gyro_x;                                // Gyroscope X-axis (rad/s) - bias corrected
        float gyro_y;                                // Gyroscope Y-axis (rad/s) - bias corrected
        float gyro_z;                                // Gyroscope Z-axis (rad/s) - bias corrected
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

#ifdef __cplusplus
}
#endif

#endif // IMU_H