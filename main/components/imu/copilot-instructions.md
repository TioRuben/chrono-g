# Copilot Instructions for IMU Component

This document provides guidelines for developing and maintaining the IMU component in the ESP32-S3 AMOLED Round Display project.

-----

## Component Overview

This IMU component provides high-precision aircraft attitude estimation using a **QMI8658 6-axis IMU sensor** on the **Waveshare ESP32-S3-Touch-AMOLED-1.75** board. It implements:

- **Madgwick AHRS filter** for sensor fusion (6DOF - accelerometer + gyroscope)
- **Aircraft coordinate system** mapping for aviation applications
- **Automatic gyroscope calibration** with bias correction
- **Software low-pass filtering** for noise reduction
- **Real-time data streaming** via FreeRTOS queues

-----

## Key Features

### Aircraft Coordinate System
**Critical:** This component uses aviation-standard coordinate system:
- **X-axis (Roll):** Forward direction (nose to tail)
- **Y-axis (Pitch):** Right wing direction (starboard)
- **Z-axis (Yaw):** Downward direction (earth reference)

**Axis Mapping:** Raw QMI8658 sensor data is remapped in `imu_task()`:
```c
mapped_imu_data.accel_x = -raw_imu_data.accel_z;  // Forward
mapped_imu_data.accel_y = raw_imu_data.accel_x;   // Right
mapped_imu_data.accel_z = raw_imu_data.accel_y;   // Down
```

### Automatic Calibration Process
The component performs **automatic gyroscope bias calibration** on startup:

1. **Collects 2000 samples** at 125Hz (≈16 seconds)
2. **Processes in batches** of 500 samples to prevent task blocking
3. **Calculates average bias** for each axis (X, Y, Z)
4. **Applies bias correction** to all subsequent gyroscope readings
5. **IMU data is invalid** until calibration completes (`IMU_CALIBRATION_COMPLETED`)

**Important:** Device must remain **stationary** during calibration for accurate bias measurement.

### Data Pipeline
```
QMI8658 Sensor → Axis Mapping → Gyro Calibration → Low-pass Filter → Madgwick AHRS → Queue
```

Output provides:
- **Raw sensor data** (accelerometer: mg, gyroscope: deg/s)
- **Filtered sensor data** (software low-pass filtering)
- **Quaternion** (w, x, y, z) for attitude representation
- **Timestamp** (microseconds since boot)
- **Calibration status** (not started, in progress, completed)

-----

## Usage Examples

### Initialize IMU Component
```c
// Create queue for IMU data (typically size 1, overwrite mode)
QueueHandle_t imu_queue = xQueueCreate(1, sizeof(imu_data_t));

// Initialize IMU component
esp_err_t ret = imu_init(imu_queue);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize IMU: %d", ret);
}
```

### Read IMU Data
```c
imu_data_t imu_data;
if (xQueueReceive(imu_queue, &imu_data, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Check calibration status first
    if (imu_data.calibration_status == IMU_CALIBRATION_COMPLETED) {
        // Convert quaternion to Euler angles
        imu_euler_angles_t euler = imu_quaternion_to_euler(
            imu_data.quat_w, imu_data.quat_x, 
            imu_data.quat_y, imu_data.quat_z
        );
        
        // Calculate G-force load factor
        float g_force = imu_calculate_g_load_factor(
            imu_data.accel_x, imu_data.accel_y, imu_data.accel_z
        );
        
        // Get turn rate (yaw axis)
        float turn_rate = imu_calculate_turn_rate(
            imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z
        );
    }
}
```

### Check Calibration Status
```c
imu_calibration_status_t status = imu_get_calibration_status();
switch (status) {
    case IMU_CALIBRATION_NOT_STARTED:
        ESP_LOGI(TAG, "Calibration not started");
        break;
    case IMU_CALIBRATION_IN_PROGRESS:
        ESP_LOGI(TAG, "Calibration in progress...");
        break;
    case IMU_CALIBRATION_COMPLETED:
        ESP_LOGI(TAG, "Calibration completed, IMU ready");
        break;
}
```

-----

## Performance Characteristics

### Timing and Frequency
- **Sampling Rate:** 125Hz (8ms period) - matches QMI8658 ODR
- **Madgwick Filter:** 31.25Hz (decimation=4) for CPU optimization
- **Queue Mode:** Overwrite (always latest data, no blocking)

### Task Priorities
- **IMU Task:** `configMAX_PRIORITIES - 1` (high priority)
- **Calibration Task:** `configMAX_PRIORITIES - 2` (lower priority)

### Memory Usage
- **IMU Task Stack:** 4096 bytes
- **Calibration Task Stack:** 4096 bytes
- **Calibration Batches:** Internal RAM allocation for performance

-----

## Key Configuration Parameters

### Madgwick Filter Tuning
```c
#define MADGWICK_BETA 0.1f              // Filter gain (0.001-1.0)
#define MADGWICK_SAMPLE_FREQ 31.25f     // Update frequency (Hz)
#define MADGWICK_DECIMATION 4           // Run every N samples
```

### Software Filter
```c
#define FILTER_ALPHA 0.15f              // Low-pass coefficient (0.0-1.0)
#define FILTER_ENABLE_ACCEL 1           // Enable accel filtering
#define FILTER_ENABLE_GYRO 1            // Enable gyro filtering
```

### Calibration Parameters
```c
#define CALIBRATION_SAMPLES 2000        // Number of samples for bias calc
#define CALIBRATION_BATCH_SIZE 500      // Batch processing size
```

-----

## Development Guidelines

### Code Style
- **Function Naming:** Use `imu_` prefix for public functions
- **Variable Naming:** Use clear, descriptive names (`gyro_bias_x` not `gbx`)
- **Error Handling:** Always check return values, use ESP_LOG macros
- **Documentation:** Document coordinate system assumptions clearly

### Testing Considerations
- **Calibration Testing:** Ensure device remains stationary during calibration
- **Coordinate System:** Verify axis mapping matches aircraft conventions
- **Performance:** Monitor task execution times, queue overflow
- **Filtering:** Test filter parameters with real motion data

### Common Pitfalls
1. **Using data before calibration completes** - always check `calibration_status`
2. **Incorrect coordinate system assumptions** - remember aircraft axes mapping
3. **Queue blocking** - use appropriate timeouts for `xQueueReceive`
4. **Filter parameter tuning** - balance noise reduction vs. responsiveness

-----

## Hardware Dependencies

- **Sensor:** QMI8658 6-axis IMU (I2C interface)
- **Board:** Waveshare ESP32-S3-Touch-AMOLED-1.75
- **I2C:** Uses BSP I2C handle from board support package
- **Address:** QMI8658_ADDRESS_HIGH configuration

-----

## Integration Notes

This component is designed for the artificial horizon display but can be used by any module requiring accurate attitude estimation. The quaternion output is preferred for attitude calculations to avoid gimbal lock issues inherent with Euler angles.

When integrating with UI components, remember that attitude data is only valid after calibration completion. Display appropriate calibration progress indicators to users during the initial setup phase.
