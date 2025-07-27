#include "artificial_horizon_ekf_wrapper.h"
#include "ekf_imu13states.h"
#include "esp_log.h"
#include <cmath>

static const char *TAG = "EKF_Wrapper";

// C++ implementation of the EKF wrapper
struct ekf_wrapper_t {
    ekf_imu13states *ekf_filter;
    bool initialized;
    
    ekf_wrapper_t() : ekf_filter(nullptr), initialized(false) {}
    
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
    
    // Initialize the EKF
    wrapper->ekf_filter->Init();
    wrapper->initialized = true;
    
    ESP_LOGI(TAG, "ESP-DSP EKF wrapper created and initialized successfully");
    return wrapper;
}

void ekf_wrapper_destroy(ekf_wrapper_t* wrapper) {
    if (wrapper) {
        delete wrapper;  // Destructor will handle cleanup
    }
}

esp_err_t ekf_wrapper_process(ekf_wrapper_t* wrapper, float gyro[3], float dt) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter) {
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        wrapper->ekf_filter->Process(gyro, dt);
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in EKF process step");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_update(ekf_wrapper_t* wrapper, float accel[3], float magn[3], float R[6]) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter) {
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        wrapper->ekf_filter->UpdateRefMeasurement(accel, magn, R);
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in EKF update step");
        return ESP_FAIL;
    }
}

esp_err_t ekf_wrapper_get_attitude(ekf_wrapper_t* wrapper, float* pitch, float* roll) {
    if (!wrapper || !wrapper->initialized || !wrapper->ekf_filter || !pitch || !roll) {
        return ESP_ERR_INVALID_ARG;
    }
    
    try {
        // Get quaternion from EKF state vector (first 4 elements)
        float quaternion[4];
        for (int i = 0; i < 4; i++) {
            quaternion[i] = wrapper->ekf_filter->X(i, 0);
        }
        
        // Convert quaternion to Euler angles using ESP-DSP utility functions
        dspm::Mat quat_euler = ekf::quat2eul(quaternion);
        
        // Extract pitch and roll (standard aircraft convention)
        *roll = quat_euler(0, 0);   // Roll (rotation around X-axis)
        *pitch = quat_euler(1, 0);  // Pitch (rotation around Y-axis)
        
        return ESP_OK;
    } catch (...) {
        ESP_LOGE(TAG, "Exception in getting attitude from EKF");
        return ESP_FAIL;
    }
}

bool ekf_wrapper_is_initialized(ekf_wrapper_t* wrapper) {
    return wrapper && wrapper->initialized;
}

} // extern "C"
