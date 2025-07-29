#ifndef ARTIFICIAL_HORIZON_EKF_WRAPPER_H
#define ARTIFICIAL_HORIZON_EKF_WRAPPER_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration of the EKF wrapper structure
typedef struct ekf_wrapper_t ekf_wrapper_t;

/**
 * @brief Create and initialize ESP-DSP EKF wrapper for 13-state IMU fusion
 * 
 * Creates an Extended Kalman Filter instance for fusing 6-DOF IMU data (accelerometer + gyroscope)
 * with optional magnetometer data. The 13-state filter includes:
 * - States [0-3]: Attitude quaternion (w, x, y, z) [dimensionless]
 * - States [4-6]: Gyroscope bias error [rad/s]
 * - States [7-9]: Magnetometer vector amplitude [sensor units]
 * - States [10-12]: Magnetometer offset [sensor units]
 * 
 * @return Pointer to EKF wrapper instance, or NULL on failure
 */
ekf_wrapper_t* ekf_wrapper_create(void);

/**
 * @brief Destroy EKF wrapper and clean up resources
 * @param wrapper Pointer to EKF wrapper instance
 */
void ekf_wrapper_destroy(ekf_wrapper_t* wrapper);

/**
 * @brief Process EKF prediction step with gyroscope data
 * 
 * Performs the time update (prediction) step of the EKF using gyroscope measurements.
 * This integrates the angular velocity to update the attitude quaternion and propagate
 * the uncertainty through the system dynamics.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param gyro Gyroscope angular velocity data [gx, gy, gz] in [rad/s]
 * @param dt Time step since last update in [seconds]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_process(ekf_wrapper_t* wrapper, float gyro[3], float dt);

/**
 * @brief Update EKF with accelerometer and magnetometer measurements (regular operation)
 * 
 * Performs the measurement update (correction) step using accelerometer and magnetometer data.
 * This method should be used during regular operation after initial calibration.
 * Only updates attitude quaternion and gyroscope bias, keeping magnetometer parameters fixed.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param accel Accelerometer data [ax, ay, az] in [g] (where 1g = 9.81 m/s²)
 * @param magn Magnetometer data [mx, my, mz] in [µT] (microtesla)
 * @param R Measurement noise covariance diagonal values [accel_x, accel_y, accel_z, magn_x, magn_y, magn_z] 
 *          Lower values indicate higher trust in measurements [dimensionless]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_update(ekf_wrapper_t* wrapper, float accel[3], float magn[3], float R[6]);

/**
 * @brief Update EKF with accelerometer and magnetometer measurements (calibration mode)
 * 
 * Performs the measurement update including magnetometer calibration parameters.
 * This method should be used during the initial calibration phase to estimate
 * magnetometer amplitude and offset parameters along with attitude and gyro bias.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param accel Accelerometer data [ax, ay, az] in [g] (where 1g = 9.81 m/s²)
 * @param magn Magnetometer data [mx, my, mz] in [µT] (microtesla)
 * @param R Measurement noise covariance diagonal values [accel_x, accel_y, accel_z, magn_x, magn_y, magn_z] 
 *          Lower values indicate higher trust in measurements [dimensionless]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_update_calibration(ekf_wrapper_t* wrapper, float accel[3], float magn[3], float R[6]);

/**
 * @brief Update EKF with accelerometer, magnetometer and reference attitude
 * 
 * Performs measurement update using all available measurements including a reference attitude.
 * This method can be used during initialization or when the system is in a known stable state.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param accel Accelerometer data [ax, ay, az] in [g] (where 1g = 9.81 m/s²)
 * @param magn Magnetometer data [mx, my, mz] in [µT] (microtesla)
 * @param attitude Reference attitude quaternion [w, x, y, z] [dimensionless, normalized]
 * @param R Measurement noise covariance diagonal values [accel_x, accel_y, accel_z, magn_x, magn_y, magn_z, quat_w, quat_x, quat_y, quat_z]
 *          Lower values indicate higher trust in measurements [dimensionless]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_update_with_attitude(ekf_wrapper_t* wrapper, float accel[3], float magn[3], float attitude[4], float R[10]);

/**
 * @brief Get current attitude estimate from EKF
 * 
 * Extracts the current attitude estimate from the EKF state vector and converts
 * the quaternion representation to Euler angles using standard aircraft convention.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param pitch Pointer to store pitch angle (nose up/down) in [radians] (range: -π/2 to π/2)
 * @param roll Pointer to store roll angle (wing down) in [radians] (range: -π to π)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_get_attitude(ekf_wrapper_t* wrapper, float* pitch, float* roll);

/**
 * @brief Get current attitude estimate as quaternion
 * 
 * Retrieves the raw quaternion attitude estimate from the EKF state vector.
 * The quaternion is automatically normalized.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param quaternion Output quaternion [w, x, y, z] [dimensionless, normalized]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_get_quaternion(ekf_wrapper_t* wrapper, float quaternion[4]);

/**
 * @brief Get current gyroscope bias estimate
 * 
 * Retrieves the estimated gyroscope bias error from the EKF state vector.
 * This bias should be subtracted from raw gyroscope measurements.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param gyro_bias Output gyroscope bias [bx, by, bz] in [rad/s]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_get_gyro_bias(ekf_wrapper_t* wrapper, float gyro_bias[3]);

/**
 * @brief Get magnetometer calibration parameters
 * 
 * Retrieves the estimated magnetometer amplitude and offset parameters.
 * These can be used for magnetometer calibration validation.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param magn_amplitude Output magnetometer amplitude vector [ax, ay, az] in [sensor units]
 * @param magn_offset Output magnetometer offset vector [ox, oy, oz] in [sensor units]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_get_magnetometer_params(ekf_wrapper_t* wrapper, float magn_amplitude[3], float magn_offset[3]);

/**
 * @brief Reset EKF state to initial conditions
 * 
 * Reinitializes the EKF state vector and covariance matrix to default values.
 * Useful for restarting the filter or handling large disturbances.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_reset(ekf_wrapper_t* wrapper);

/**
 * @brief Set reference vectors for accelerometer and magnetometer
 * 
 * Sets the reference vectors used by the EKF for measurement updates.
 * The accelerometer reference should point to local gravity direction.
 * The magnetometer reference should point to local magnetic north.
 * 
 * @param wrapper Pointer to EKF wrapper instance
 * @param accel_ref Reference accelerometer vector [ax, ay, az] in [g] (normalized)
 * @param magn_ref Reference magnetometer vector [mx, my, mz] in [µT] (normalized)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_set_reference_vectors(ekf_wrapper_t* wrapper, float accel_ref[3], float magn_ref[3]);

/**
 * @brief Check if EKF wrapper is properly initialized
 * @param wrapper Pointer to EKF wrapper instance
 * @return true if initialized, false otherwise
 */
bool ekf_wrapper_is_initialized(ekf_wrapper_t* wrapper);

#ifdef __cplusplus
}
#endif

#endif // ARTIFICIAL_HORIZON_EKF_WRAPPER_H
