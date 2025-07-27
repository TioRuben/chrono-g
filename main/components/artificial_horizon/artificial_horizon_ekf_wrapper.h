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
 * @brief Create and initialize ESP-DSP EKF wrapper
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
 * @param wrapper Pointer to EKF wrapper instance
 * @param gyro Gyroscope data [gx, gy, gz] in rad/s
 * @param dt Time step in seconds
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_process(ekf_wrapper_t* wrapper, float gyro[3], float dt);

/**
 * @brief Update EKF with accelerometer and magnetometer measurements
 * @param wrapper Pointer to EKF wrapper instance
 * @param accel Accelerometer data [ax, ay, az] in m/s²
 * @param magn Magnetometer data [mx, my, mz] in µT
 * @param R Measurement noise covariance diagonal [accel_noise_x, accel_noise_y, accel_noise_z, magn_noise_x, magn_noise_y, magn_noise_z]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_update(ekf_wrapper_t* wrapper, float accel[3], float magn[3], float R[6]);

/**
 * @brief Get current attitude estimate from EKF
 * @param wrapper Pointer to EKF wrapper instance
 * @param pitch Pointer to store pitch angle in radians
 * @param roll Pointer to store roll angle in radians
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ekf_wrapper_get_attitude(ekf_wrapper_t* wrapper, float* pitch, float* roll);

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
