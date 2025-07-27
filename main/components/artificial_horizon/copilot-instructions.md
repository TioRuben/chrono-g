# Copilot Instructions for Artificial Horizon Component

This document provides specific guidelines for developing and maintaining the artificial horizon component of the ESP32-S3 AMOLED Round Display project.

-----

## Component Overview

The artificial horizon component provides a real-time graphical representation of aircraft attitude using accelerometer and gyroscope data from the QMI8658 sensor. It implements an Extended Kalman Filter (EKF) for accurate angle estimation and displays the horizon using LVGL graphics.

### Key Features

- **Real-time Attitude Display:** Shows pitch and roll angles using a moving brown ground rectangle against a blue sky background
- **Extended Kalman Filter:** Provides accurate angle estimation by fusing accelerometer and gyroscope data
- **G-Force Monitoring:** Displays current and maximum G-force with color-coded warnings
- **Performance Optimization:** Adaptive update rates based on visibility and motion thresholds
- **Aircraft Reference Symbol:** Yellow inverted-T symbol representing the aircraft's position

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
// EKF to Display Mapping (corrected for aircraft instrument panel)
display_roll = ekf_pitch;   // Banking left/right
display_pitch = ekf_roll;   // Nose up/down
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
    lv_obj_t *container;           // Main LVGL container
    lv_obj_t *brown_square;        // Ground representation
    lv_obj_t *aircraft_line_h;     // Aircraft horizontal line
    lv_obj_t *aircraft_line_v;     // Aircraft vertical line
    qmi8658_dev_t *imu_dev;        // IMU device handle
    ekf_state_t ekf;               // EKF state
    // ... additional state variables
} ah_state;
```

-----

## Extended Kalman Filter Implementation

### EKF Parameters (Tuned for Real-time Performance)
```c
#define Q_angle 0.005f      // Process noise for angles
#define Q_bias 0.0005f      // Process noise for gyro bias
#define R_measure 0.05f     // Measurement noise (accelerometer)
```

### State Vector
- `pitch`: Pitch angle in radians
- `roll`: Roll angle in radians  
- `bias_gx`: Gyroscope X-axis bias
- `bias_gy`: Gyroscope Y-axis bias

### Performance Optimizations
- **Motion Threshold:** Only updates display when motion exceeds 2° threshold
- **Adaptive Sampling:** 20ms updates when visible, 100ms when hidden
- **Low-pass Filtering:** Separate filter constants for accelerometer (α=0.15) and gyroscope (α=0.8)

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
- **Static Functions:** Use descriptive names with `_` prefix for internal helpers
- **Constants:** Use `#define` with descriptive names and comments
- **Error Handling:** Always check return values and log errors appropriately

### Memory Management
- **Stack Size:** IMU/EKF task uses 8KB stack (2048 * 4)
- **Queue Size:** Single-element queue for angle data (latest value only)
- **LVGL Objects:** Parent-child relationship ensures proper cleanup

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
#define DISPLAY_REFRESH_PERIOD_MS 40    // 25 FPS display updates
#define MOTION_THRESHOLD (2.0f * M_PI / 180.0f)  // 2 degree threshold
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
- **Target Frame Rate:** 25 FPS display updates
- **IMU Sample Rate:** 50 Hz when visible, 10 Hz when hidden
- **Memory Footprint:** Monitor heap usage during operation

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
