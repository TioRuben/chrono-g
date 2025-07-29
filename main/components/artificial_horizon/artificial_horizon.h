#ifndef ARTIFICIAL_HORIZON_H
#define ARTIFICIAL_HORIZON_H

#include "qmi8658.h"
#include "lvgl.h"
#include "esp_err.h"

void artificial_horizon_init(lv_obj_t *parent, qmi8658_dev_t *imu_dev);
void artificial_horizon_deinit(void);
void artificial_horizon_set_visible(bool visible);

/**
 * @brief Read and map sensor data for EKF processing
 *
 * Reads raw sensor data from QMI8658, applies aircraft coordinate mapping,
 * and prepares the data in the correct units for ESP-DSP EKF processing.
 *
 * @param[out] ekf_accel EKF-ready accelerometer data [g] in aircraft coordinates [Fwd, Right, Down]
 * @param[out] ekf_gyro EKF-ready gyroscope data [rad/s] in aircraft coordinates [Pitch, Roll, Yaw]
 * @return ESP_OK on success, error code on failure
 */
esp_err_t artificial_horizon_read_sensor_data(float ekf_accel[3], float ekf_gyro[3]);

#endif // ARTIFICIAL_HORIZON_H