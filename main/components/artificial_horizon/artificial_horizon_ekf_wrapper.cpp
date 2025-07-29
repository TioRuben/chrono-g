#include "artificial_horizon_ekf_wrapper.h"
#include "ekf_imu13states.h"
#include "esp_log.h"
#include <cmath>

static const char *TAG = "EKF_Wrapper";

/**
 * @brief C++ implementation of the EKF wrapper structure
 * 
 * This structure encapsulates the ESP-DSP EKF instance and maintains
 * initialization state for safe operation from C code.
 */
struct ekf_wrapper_t {
    ekf_imu13states *ekf_filter;    ///< ESP-DSP EKF filter instance
    bool initialized;               ///< Initialization status flag
    bool calibration_mode;          ///< Flag indicating if in calibration phase
    
    ekf_wrapper_t() : ekf_filter(nullptr), initialized(false), calibration_mode(true) {}
    
    ~ekf_wrapper_t() {
        if (ekf_filter) {
            delete ekf_filter;
        }
    }
};

// C interface functions
extern "C" {

ekf_wrapper_t* ekf_wrapper_create(void) {
    ekf_wrapper_t* wrapper = new ekf_wrapper_t();
    if (!wrapper) {
        ESP_LOGE(TAG, "Failed to allocate EKF wrapper");
        return nullptr;
    }
    
    wrapper->ekf_filter = new ekf_imu13states();
    if (!wrapper->ekf_filter) {
        ESP_LOGE(TAG, "Failed to create ESP-DSP EKF instance");
        delete wrapper;
        return nullptr;
    }
    
    // Initialize the EKF with default parameters
    wrapper->ekf_filter->Init();
    wrapper->initialized = true;
    wrapper->calibration_mode = true;
    
    ESP_LOGI(TAG, "ESP-DSP EKF wrapper created and initialized successfully");
    ESP_LOGI(TAG, "Filter state vector has %d states", wrapper->ekf_filter->NUMX);
    ESP_LOGI(TAG, "Filter control vector has %d inputs", wrapper->ekf_filter->NUMU);
    
    return wrapper;
}

void ekf_wrapper_destroy(ekf_wrapper_t* wrapper) {
    if (wrapper) {
        ESP_LOGI(TAG, "Destroying EKF wrapper");
        delete wrapper;  // Destructor will handle cleanup
    }
}

esp_err_t ekf_wrapper_process(ekf_wrapper_t* wrapper, float gyro[3], float dt) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter) {
        ESP_LOGE(TAG, "Invalid wrapper or uninitialized EKF");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!gyro || dt <= 0.0f) {
        ESP_LOGE(TAG, "Invalid gyro data or time step");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Process gyroscope data through the EKF prediction step
        wrapper->ekf_filter->Process(gyro, dt);
        
        // Normalize quaternion to prevent numerical drift
        float quat_norm = 0.0f;
        for (int i = 0; i < 4; i++) {
            float q_val = wrapper->ekf_filter->X(i, 0);
            quat_norm += q_val * q_val;
        }
        quat_norm = sqrtf(quat_norm);
        
        if (quat_norm > 1e-6f) {
            for (int i = 0; i < 4; i++) {
                wrapper->ekf_filter->X(i, 0) /= quat_norm;
            }
        }
        
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in EKF process step");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_update(ekf_wrapper_t* wrapper, float accel[3], float magn[3], float R[6]) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter) {
        ESP_LOGE(TAG, "Invalid wrapper or uninitialized EKF");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!accel || !magn || !R) {
        ESP_LOGE(TAG, "Invalid measurement data pointers");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Use regular measurement update (post-calibration mode)
        // This updates attitude and gyro bias only
        wrapper->ekf_filter->UpdateRefMeasurement(accel, magn, R);
        wrapper->calibration_mode = false;
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in EKF measurement update");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_update_calibration(ekf_wrapper_t* wrapper, float accel[3], float magn[3], float R[6]) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter) {
        ESP_LOGE(TAG, "Invalid wrapper or uninitialized EKF");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!accel || !magn || !R) {
        ESP_LOGE(TAG, "Invalid measurement data pointers");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Use full measurement update including magnetometer calibration
        // This updates all states including magnetometer parameters
        wrapper->ekf_filter->UpdateRefMeasurementMagn(accel, magn, R);
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in EKF calibration update");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_update_with_attitude(ekf_wrapper_t* wrapper, float accel[3], float magn[3], float attitude[4], float R[10]) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter) {
        ESP_LOGE(TAG, "Invalid wrapper or uninitialized EKF");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!accel || !magn || !attitude || !R) {
        ESP_LOGE(TAG, "Invalid measurement data pointers");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Use measurement update with reference attitude
        // This is useful for initialization or when attitude is known
        wrapper->ekf_filter->UpdateRefMeasurement(accel, magn, attitude, R);
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in EKF attitude update");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_get_attitude(ekf_wrapper_t* wrapper, float* pitch, float* roll) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter || !pitch || !roll) {
        ESP_LOGE(TAG, "Invalid arguments for attitude retrieval");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Get quaternion from EKF state vector (first 4 elements)
        float quaternion[4];
        for (int i = 0; i < 4; i++) {
            quaternion[i] = wrapper->ekf_filter->X(i, 0);
        }
        
        // Normalize quaternion to ensure valid rotation
        float quat_norm = 0.0f;
        for (int i = 0; i < 4; i++) {
            quat_norm += quaternion[i] * quaternion[i];
        }
        quat_norm = sqrtf(quat_norm);
        
        if (quat_norm < 1e-6f) {
            ESP_LOGE(TAG, "Invalid quaternion magnitude");
            return ESP_FAIL;
        }
        
        for (int i = 0; i < 4; i++) {
            quaternion[i] /= quat_norm;
        }
        
        // Convert quaternion to Euler angles using ESP-DSP utility functions
        dspm::Mat quat_euler = ekf::quat2eul(quaternion);
        
        // Extract pitch and roll (standard aircraft convention)
        // quat2eul returns [roll, pitch, yaw] in radians
        *roll = quat_euler(0, 0);   // Roll (rotation around X-axis)
        *pitch = quat_euler(1, 0);  // Pitch (rotation around Y-axis)
        
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in getting attitude from EKF");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_get_quaternion(ekf_wrapper_t* wrapper, float quaternion[4]) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter || !quaternion) {
        ESP_LOGE(TAG, "Invalid arguments for quaternion retrieval");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Get quaternion from EKF state vector (first 4 elements) and normalize
        float quat_norm = 0.0f;
        for (int i = 0; i < 4; i++) {
            quaternion[i] = wrapper->ekf_filter->X(i, 0);
            quat_norm += quaternion[i] * quaternion[i];
        }
        quat_norm = sqrtf(quat_norm);
        
        if (quat_norm < 1e-6f) {
            ESP_LOGE(TAG, "Invalid quaternion magnitude");
            return ESP_FAIL;
        }
        
        for (int i = 0; i < 4; i++) {
            quaternion[i] /= quat_norm;
        }
        
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in getting quaternion from EKF");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_get_gyro_bias(ekf_wrapper_t* wrapper, float gyro_bias[3]) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter || !gyro_bias) {
        ESP_LOGE(TAG, "Invalid arguments for gyro bias retrieval");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Get gyroscope bias from EKF state vector (elements 4-6)
        for (int i = 0; i < 3; i++) {
            gyro_bias[i] = wrapper->ekf_filter->X(4 + i, 0);
        }
        
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in getting gyro bias from EKF");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_get_magnetometer_params(ekf_wrapper_t* wrapper, float magn_amplitude[3], float magn_offset[3]) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter || !magn_amplitude || !magn_offset) {
        ESP_LOGE(TAG, "Invalid arguments for magnetometer parameters retrieval");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Get magnetometer amplitude from EKF state vector (elements 7-9)
        for (int i = 0; i < 3; i++) {
            magn_amplitude[i] = wrapper->ekf_filter->X(7 + i, 0);
        }
        
        // Get magnetometer offset from EKF state vector (elements 10-12)
        for (int i = 0; i < 3; i++) {
            magn_offset[i] = wrapper->ekf_filter->X(10 + i, 0);
        }
        
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in getting magnetometer parameters from EKF");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_reset(ekf_wrapper_t* wrapper) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter) {
        ESP_LOGE(TAG, "Invalid wrapper or uninitialized EKF");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Reinitialize the EKF to default state
        wrapper->ekf_filter->Init();
        wrapper->calibration_mode = true;
        
        ESP_LOGI(TAG, "EKF state reset to initial conditions");
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in EKF reset");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_set_reference_vectors(ekf_wrapper_t* wrapper, float accel_ref[3], float magn_ref[3]) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter || !accel_ref || !magn_ref) {
        ESP_LOGE(TAG, "Invalid arguments for setting reference vectors");
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Set reference accelerometer vector (gravity direction)
        for (int i = 0; i < 3; i++) {
            wrapper->ekf_filter->accel0(i, 0) = accel_ref[i];
        }
        
        // Set reference magnetometer vector (magnetic north direction)
        for (int i = 0; i < 3; i++) {
            wrapper->ekf_filter->mag0(i, 0) = magn_ref[i];
        }
        
        ESP_LOGI(TAG, "Reference vectors updated: accel=[%.3f,%.3f,%.3f], magn=[%.3f,%.3f,%.3f]",
                 accel_ref[0], accel_ref[1], accel_ref[2],
                 magn_ref[0], magn_ref[1], magn_ref[2]);
        
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in setting reference vectors");
        return ESP_FAIL;
    }
}

bool ekf_wrapper_is_initialized(ekf_wrapper_t* wrapper) {
    return wrapper && wrapper->initialized;
}

} // extern "C"
