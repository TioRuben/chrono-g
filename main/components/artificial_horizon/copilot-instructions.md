# Copilot Instructions for Artificial Horizon Component

This document provides specific guidelines for developing and maintaining the artificial horizon component of the ESP32-S3 AMOLED Round Display project.

-----

## C### Common Issues and Solutions

### Code Quality Issues
- **Problem:** Duplicate definitions or unused includes
- **Solution:** Regular code cleanup to remove duplications, unused headers, and commented-out code

### Display Artifacts
- **Problem:** Flickering during updates
- **Solution:** Use `bsp_display_lock(0)` and `bsp_display_unlock()` around multi-object operations

### Performance Issues
- **Problem:** Slow response or dropped frames
- **Solution:** Verify motion threshold settings (1° for good responsiveness) and task priorities

### Angle Accuracy
- **Problem:** Drift or incorrect readings
- **Solution:** Tune ESP-DSP EKF parameters (`PROCESS_NOISE_Q`, `ACCEL_NOISE_VARIANCE`, etc.)

### Memory Leaks
- **Problem:** Increasing memory usage over time
- **Solution:** Ensure proper cleanup in `artificial_horizon_deinit()` and verify all includes are necessary

The artificial horizon component provides a real-time graphical representation of aircraft attitude using accelerometer and gyroscope data from the QMI8658 sensor. It implements an Extended Kalman Filter (EKF) for accurate angle estimation and displays the horizon using LVGL graphics.

### Key Features

- **Real-time Attitude Display:** Shows pitch and roll angles using a moving brown ground rectangle against a blue sky background
- **ESP-DSP Extended Kalman Filter:** Uses ESP-IDF's optimized ESP-DSP library for accurate angle estimation by fusing accelerometer and gyroscope data
- **G-Force Monitoring:** Displays current and maximum G-force with color-coded warnings and click-to-reset functionality
- **Performance Optimization:** Adaptive update rates based on visibility and motion thresholds
- **Aircraft Reference Symbol:** Yellow inverted-T symbol representing the aircraft's position
- **Code Quality:** Clean, optimized codebase with no duplications or unused dependencies

-----

## Hardware Integration

### IMU Sensor Configuration
- **Sensor:** QMI8658 6-axis IMU (accelerometer + gyroscope)
- **Mounting Orientation:** Device mounted vertically with specific axis mapping:
  - **X-axis:** Points left (pitch axis)
  - **Y-axis:** Points DOWN toward gravity (reference for level flight)
  - **Z-axis:** Points forward in flight direction (roll axis)

### Coordinate System Mapping
```c
// Sensor to EKF axis mapping (corrected for ESP-DSP implementation)
float gyro_data[3] = {
    filtered_data.gyro_x,     // X-axis: roll gyro
    -filtered_data.gyro_z,    // Y-axis: pitch gyro (negated)
    filtered_data.gyro_y      // Z-axis: yaw gyro
};

float accel_data[3] = {
    filtered_data.accel_x,    // X-axis: roll accel
    -filtered_data.accel_z,   // Y-axis: pitch accel (negated)
    filtered_data.accel_y     // Z-axis: vertical accel
};

// EKF output to display mapping
estimated_angles_t angles = {
    .pitch = ekf_pitch,       // Nose up/down
    .roll = -ekf_roll         // Banking left/right (negated)
};
```

-----

## Architecture

### Multi-Threading Design
1. **IMU/EKF Task:** High-priority FreeRTOS task for sensor reading and angle estimation
2. **Display Update Timer:** LVGL timer for rendering updates at 25 FPS
3. **Inter-Task Communication:** FreeRTOS queue for passing angle data between tasks

### State Management
```c
static struct {
    lv_obj_t *container;              // Main LVGL container
    lv_obj_t *brown_square;           // Ground representation
    lv_obj_t *aircraft_line_h;        // Aircraft horizontal line
    lv_obj_t *aircraft_line_v;        // Aircraft vertical line
    lv_timer_t *display_update_timer; // LVGL display timer
    qmi8658_dev_t *imu_dev;           // IMU device handle
    ekf_wrapper_t *ekf_wrapper;       // ESP-DSP EKF wrapper
    QueueHandle_t angle_queue;        // FreeRTOS queue for angle data
    TaskHandle_t imu_ekf_task_handle; // IMU/EKF task handle
    estimated_angles_t last_angles;   // Last known angles for threshold checking
    filtered_data_t filtered_data;    // Filtered sensor data
    bool is_visible;                  // Visibility state for optimization
    int container_width, container_height; // Container dimensions
    struct {
        float current_g, max_g;       // G-force tracking
        lv_obj_t *button, *label;     // G-force display UI elements
    } g_data;
} ah_state = {0};
```

-----

## ESP-DSP Extended Kalman Filter Implementation

### EKF Parameters (Optimized for ESP-DSP Library)
```c
// Sensor noise parameters for ESP-DSP EKF
#define PROCESS_NOISE_Q 0.001f      // Process noise for gyroscope integration (rad/s)²
#define ACCEL_NOISE_VARIANCE 0.01f  // Accelerometer measurement noise (m/s²)²
#define GYRO_NOISE_VARIANCE 0.001f  // Gyroscope measurement noise (rad/s)²
#define MAGN_NOISE_VARIANCE 0.1f    // Magnetometer measurement noise (µT)²

// Low-pass filter parameters for sensor data smoothing
#define ACCEL_FILTER_ALPHA 0.3f     // Accelerometer filter (0.1-0.8)
#define GYRO_FILTER_ALPHA 0.85f     // Gyroscope filter (0.7-0.95)
```

### ESP-DSP Integration
- **Library:** Uses ESP-IDF's optimized ESP-DSP library for mathematical operations
- **Wrapper:** Custom `ekf_wrapper_t` provides simplified interface to ESP-DSP EKF functions
- **Initialization:** EKF wrapper handles state initialization and parameter setup
- **Processing:** Separate prediction and update steps for optimal performance

### Performance Optimizations
- **Motion Threshold:** Only updates display when motion exceeds 1.0° threshold (reduced from 2°)
- **Adaptive Sampling:** 20ms updates when visible, 100ms when hidden
- **Low-pass Filtering:** Dual-stage filtering with separate constants for accelerometer and gyroscope
- **Queue Management:** Single-element overwrite queue prevents data backup

-----

## Display Rendering

### Visual Elements
1. **Sky Background:** Light blue (`0x87CEEB`) covering entire container
2. **Ground Rectangle:** Brown (`0x8B4513`) square with white horizon line
3. **Aircraft Symbol:** Yellow (`0xFFFF00`) inverted-T in center
4. **G-Force Display:** Bottom panel with current and maximum G readings

### Transformation Logic
```c
// Pitch affects vertical position (inverted for correct horizon behavior)
float pitch_offset_pixels = -pitch_rad * (container_height / 2.0f);

// Roll affects rotation around center point
int roll_decidegrees = (int)(roll_rad * 1800.0f / M_PI);
lv_obj_set_style_transform_angle(brown_square, roll_decidegrees, 0);
```

-----

## Development Guidelines

### Code Style Conventions
- **Function Naming:** Use `artificial_horizon_` prefix for public functions
- **Static Functions:** Use descriptive names for internal helpers
- **Constants:** Use `#define` with descriptive names and comprehensive comments
- **Error Handling:** Always check return values and log errors appropriately
- **Code Cleanliness:** Avoid duplicate definitions, unused includes, and commented-out code
- **Include Optimization:** Only include headers that are actually used in the code

### Memory Management
- **Stack Size:** IMU/EKF task uses 8KB stack (2048 * 4) - optimized size
- **Queue Size:** Single-element overwrite queue for angle data (latest value only)
- **LVGL Objects:** Parent-child relationship ensures proper cleanup
- **Include Minimization:** Removed unused includes (`freertos/semphr.h`, `<string.h>`) to reduce memory footprint

### Performance Considerations
- **Display Locking:** Use `bsp_display_lock(0)` during multi-object updates
- **Floating Point:** All angle calculations use `float` for ESP32-S3 FPU efficiency
- **Memory Allocation:** Avoid dynamic allocation in real-time paths

### Error Handling Patterns
```c
if (function_call() != ESP_OK) {
    ESP_LOGE(TAG, "Descriptive error message");
    // Cleanup and return appropriate error code
    return NULL;
}
```

-----

## Configuration Parameters

### Timing Constants
```c
#define DISPLAY_REFRESH_PERIOD_MS 17           // ~60 FPS display updates (was 25 FPS)
#define MOTION_THRESHOLD (1.0f * M_PI / 180.0f)  // 1 degree threshold (improved sensitivity)
```

### Color Definitions
```c
#define SKY_COLOR lv_color_hex(0x87CEEB)           // Light blue
#define GROUND_COLOR lv_color_hex(0x8B4513)        // Saddle brown
#define AIRCRAFT_LINE_COLOR lv_color_hex(0xFFFF00) // Yellow
```

### G-Force Thresholds
- **Normal:** 0.8G to 2.0G (white text)
- **Warning:** 0.5G to 0.8G or 2.0G to 4.0G (orange text)
- **Danger:** Below 0.5G or above 4.0G (red text)

-----

## Testing and Validation

### Unit Testing Focus Areas
1. **EKF Accuracy:** Verify angle estimation against known attitudes
2. **Performance:** Measure update rates and CPU usage
3. **Memory Usage:** Check for leaks in long-running scenarios
4. **Edge Cases:** Handle sensor failures and extreme attitudes

### Debug Logging
- Use `ESP_LOGI` for initialization and major state changes
- Use `ESP_LOGW` for recoverable errors and warnings
- Use `ESP_LOGE` for critical errors requiring attention

### Performance Metrics
- **Target Frame Rate:** ~60 FPS display updates (17ms refresh period)
- **IMU Sample Rate:** 50 Hz when visible, 10 Hz when hidden
- **Memory Footprint:** Optimized with removed unused includes and no duplicate definitions
- **Motion Sensitivity:** 1° threshold for improved responsiveness

-----

## Integration Points

### With Main Application
```c
// Initialize artificial horizon with IMU device
lv_obj_t *ah_container = artificial_horizon_init(tileview_tile, &imu_dev);

// Set visibility based on current page
artificial_horizon_set_visible(current_page == ARTIFICIAL_HORIZON_PAGE);
```

### With Tileview Navigation
- Call `artificial_horizon_set_visible(false)` when navigating away
- Call `artificial_horizon_set_visible(true)` when returning to page
- Properly handle deinitialization when application exits

-----

## Common Issues and Solutions

### Display Artifacts
- **Problem:** Flickering during updates
- **Solution:** Use `bsp_display_lock(0)` and `bsp_display_unlock()` around multi-object operations

### Performance Issues
- **Problem:** Slow response or dropped frames
- **Solution:** Verify motion threshold settings and task priorities

### Angle Accuracy
- **Problem:** Drift or incorrect readings
- **Solution:** Tune EKF parameters (`Q_angle`, `Q_bias`, `R_measure`)

### Memory Leaks
- **Problem:** Increasing memory usage over time
- **Solution:** Ensure proper cleanup in `artificial_horizon_deinit()`

-----

## Future Enhancements

### Potential Improvements
1. **Attitude Indicators:** Add pitch ladder markings for precise angle reading
2. **Turn Coordinator:** Integrate turn rate indication using gyroscope Z-axis
3. **Calibration:** Add IMU calibration routine for improved accuracy
4. **Configuration:** Runtime parameter adjustment via settings menu

### Performance Optimizations
1. **Sensor Fusion:** Implement complementary filter option for comparison
2. **Rendering:** Use LVGL canvas for more efficient custom drawing
3. **Power Management:** Further reduce update rates during low activity

-----
