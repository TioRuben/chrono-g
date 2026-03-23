#ifndef IMU_H
#define IMU_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "qmi8658.h"

// IMU Data Structures
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int64_t timestamp;
} imu_raw_data_t;

typedef struct {
    float accel_x; // in g
    float accel_y; // in g
    float accel_z; // in g
    float gyro_x; // in dps
    float gyro_y; // in dps
    float gyro_z; // in dps
    int64_t timestamp;
} imu_data_t;

typedef struct {
    bool accel_pass;
    bool gyro_pass;
} imu_self_test_result_t;

typedef struct {
    qmi8658_acc_range_t accel_range;
    qmi8658_gyr_range_t gyro_range;
    qmi8658_acc_odr_t accel_odr;
    qmi8658_gyr_odr_t gyro_odr;
    bool enable_accel_lpf;
    bool enable_gyro_lpf;
    qmi8658_lpf_mode_t lpf_mode;
    uint16_t sample_rate_hz;
    bool calibrate_on_init;
    uint16_t calibration_samples;
    float accel_sensitivity;
    float gyro_sensitivity;
} imu_config_t;

// Function Prototypes
esp_err_t qmi8658_init(i2c_master_bus_handle_t bus_handle);
esp_err_t imu_init(i2c_master_bus_handle_t bus_handle, QueueHandle_t imu_queue);
esp_err_t imu_init_with_config(i2c_master_bus_handle_t bus_handle, QueueHandle_t imu_queue, const imu_config_t *config);
esp_err_t imu_read_raw(imu_raw_data_t *data);
esp_err_t imu_read_processed(imu_data_t *data);
esp_err_t imu_read_temperature(float *temperature);
esp_err_t imu_calibrate_gyro(uint16_t num_samples);
esp_err_t imu_get_gyro_bias(float *bias_x, float *bias_y, float *bias_z);
esp_err_t imu_set_gyro_bias(float bias_x, float bias_y, float bias_z);
esp_err_t imu_self_test(imu_self_test_result_t *result);

#endif // IMU_H