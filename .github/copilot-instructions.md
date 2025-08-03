# Copilot Instructions for ESP32-S3 AMOLED Round Display Project

This document provides guidelines and essential information for developing on the ESP32-S3 Round Display project.

-----

## Project Overview

This project targets the **Waveshare ESP32-S3-Touch-AMOLED-1.75** development board. The core functionality revolves around a round display with four horizontally swipeable pages, each dedicated to a specific function:

  * **Cyan Stopwatch:** A digital stopwatch with a cyan theme.
  * **Yellow Stopwatch:** A digital stopwatch with a yellow theme.
  * **G-Meter:** A digital G-force meter showing current, minimum, and maximum G-force values with color-coded warnings.
  * **Artificial Horizon:** A graphical representation of an artificial horizon using raw IMU sensor data with G-force monitoring and color-coded warnings.

The project leverages two timers for the stopwatches and implements advanced IMU sensor data processing for both the G-meter and artificial horizon with proper aircraft-style G-force calculations.

-----

## Hardware

  * **Development Board:** Waveshare ESP32-S3-Touch-AMOLED-1.75
      * [Product Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.75)

-----

## Software and Dependencies

This project is built using the **ESP-IDF framework** (C language). The following components from the ESP-IDF Component Registry are utilized:

  * **QMI8658 Driver:** `waveshare/qmi8658` (Version 1.0.1)

      * [Component Page](https://components.espressif.com/components/waveshare/qmi8658/versions/1.0.1)
      * Used for reading data from the QMI8658 accelerometer/gyroscope.

  * **LVGL Graphics Library:** `lvgl/lvgl` (Version 9.3.0)

      * [Component Page](https://components.espressif.com/components/lvgl/lvgl/versions/9.3.0)
      * Used for all graphical user interface (GUI) elements and display management.

  * **ESP32-S3-Touch-AMOLED-1.75 Board Support Package:** `waveshare/esp32_s3_touch_amoled_1_75` (Version 1.0.1)

      * [Component Page](https://components.espressif.com/components/waveshare/esp32_s3_touch_amoled_1_75/versions/1.0.1)
      * Provides board-specific drivers and configurations for the display and touch functionality.

-----

## Project Structure

The project is structured to ensure modularity and maintainability. Each display page will reside in its own `.c` file, accompanied by a corresponding header file (`.h`). These individual page files will then be included and managed within `main.c`. The project also includes a visibility management system to optimize UI updates based on which tile is currently visible.

```
.
├── main/
│   ├── main.c
│   ├── main.h
│   ├── visibility_manager.c
│   ├── visibility_manager.h
│   ├── lv_font_g_meter_96.c        # Custom font for G-meter display
│   ├── lv_font_seven_segment_64.c  # Custom font for stopwatches
│   ├── components/
│   │   ├── cyan_stopwatch/
│   │   │   ├── cyan_stopwatch.c
│   │   │   └── cyan_stopwatch.h
│   │   ├── yellow_stopwatch/
│   │   │   ├── yellow_stopwatch.c
│   │   │   └── yellow_stopwatch.h
│   │   ├── g_meter/
│   │   │   ├── g_meter.c
│   │   │   └── g_meter.h
│   │   ├── artificial_horizon/
│   │   │   ├── artificial_horizon.c
│   │   │   └── artificial_horizon.h
│   │   └── imu/
│   │       ├── imu.c
│   │       └── imu.h
│   └── CMakeLists.txt
├── CMakeLists.txt
└── sdkconfig.defaults
```

-----

## Key Features and Implementation Details

### Display Navigation

  * **LVGL Tileview:** The four display pages are implemented as tiles within an **LVGL Tileview** object.
  * **Horizontal Swiping:** Users navigate between the stopwatch pages, G-meter, and artificial horizon by horizontally swiping across the display. Refer to the [LVGL Tileview Documentation](https://docs.lvgl.io/master/details/widgets/tileview.html) for implementation details.
  * **Visibility Management:** The project includes a visibility management system that tracks which tile is currently visible and optimizes UI updates accordingly. Components continue data processing but skip UI updates when not visible.

### Component Visibility System

Each component must implement visibility control to optimize performance:

  * **Visibility Function:** Each component should provide a `<component>_set_visible(bool visible)` function
  * **UI Update Control:** When not visible, components should continue data processing but skip UI rendering operations
  * **State Synchronization:** When becoming visible, components should update their UI to reflect the current state
  * **Integration:** New components must be integrated into the `tileview_event_cb` function in `main.c` to receive visibility notifications

### Component Implementation Guidelines

When creating a new tile component, follow these patterns:

1. **Header File (.h):**
   ```c
   // Include visibility control function
   void <component>_set_visible(bool visible);
   ```

2. **Source File (.c):**
   ```c
   // Add visibility state to component structure
   static struct {
       // ... other fields
       bool is_visible;
   } component_state = {0};

   // Check visibility before UI updates
   if (component_state.is_visible) {
       // Perform UI updates
   }

   // Implement visibility control function
   void <component>_set_visible(bool visible) {
       component_state.is_visible = visible;
       // Update UI if becoming visible and state is out of sync
   }
   ```

3. **Main Integration:**
   - Add the component to the `tile_index_t` enum in `visibility_manager.h`
   - Include the visibility call in `tileview_event_cb` function in `main.c`
   - Update `CMakeLists.txt` to include the new component files

### Stopwatch Functionality

  * **Two Independent Timers:** Each stopwatch (Cyan and Yellow) operates independently using dedicated timers.
  * **Display Updates:** The stopwatch values are updated and rendered on their respective LVGL pages with custom seven-segment font.
  * **Precision:** High-precision timing with millisecond accuracy using FreeRTOS tick counters.

### G-Meter (G-Force Meter)

  * **Real-time G-Force Display:** Shows current G-force with signed values (+/-) to indicate direction.
  * **Color-Coded Warnings:** 
      * **White:** Normal range (0.8G to 1.2G)
      * **Orange:** Warning range (outside normal but within limits)
      * **Red:** Danger range (below -1.0G or above 3.0G)
  * **Min/Max Tracking:** Continuously tracks and displays minimum and maximum G-forces encountered.
  * **Reset Functionality:** Button to reset min/max values to current G-force.
  * **Aircraft G-Force Calculation:** Uses signed Z-axis acceleration for proper inverted flight detection.
  * **Display Format:** Shows values like "+1.0G" (normal flight) or "-1.0G" (inverted flight).

### IMU System

  * **QMI8658 IMU Sensor:** 6-axis IMU (accelerometer + gyroscope) data acquisition.
  * **Software Filtering:** Optional exponential moving average low-pass filtering for accelerometer and gyroscope data to reduce noise.
  * **Aircraft Coordinate System:** X=forward(roll), Y=right(pitch), Z=down(yaw)
  * **Gyroscope Calibration:** Automatic gyroscope bias calibration on startup (2000 samples) processed in batches to avoid blocking.
  * **G-Force Calculation:** 
      * **Aircraft G-Force:** Returns signed Z-axis acceleration for proper inverted flight detection
      * **Total Acceleration:** Separate function for magnitude calculation when needed
  * **Performance:** Optimized for 125Hz sampling rate with efficient task management.
  * **Key Functions:**
      * `imu_calculate_g_load_factor()`: Returns signed G-force (+1G normal, -1G inverted)
      * `imu_calculate_total_acceleration()`: Returns total acceleration magnitude
      * `imu_calculate_turn_rate()`: Returns yaw axis turn rate in deg/s

### Artificial Horizon

  * **IMU Integration:** Uses the same IMU system as G-meter for consistent sensor data.
  * **Aircraft-Style Display:** Graphical representation following aviation standards.
  * **G-Force Monitoring:** Integrated G-force display with color-coded warnings.
  * **Turn Rate Display:** Shows aircraft turn rate from yaw axis gyroscope data.

-----

## Development Guidelines

  * **Code Style:** Adhere to standard C coding practices and ESP-IDF conventions.
  * **Code Quality:** Maintain clean code by eliminating duplications, unused includes, and commented-out code.
  * **Modularity:** Keep functions and modules focused on a single responsibility.
  * **Resource Management:** Pay close attention to memory usage and task scheduling, especially given the embedded nature of the project.
  * **Include Optimization:** Only include headers that are actually used to minimize memory footprint.
  * **Error Handling:** Implement robust error handling for sensor readings, display operations, and timer management.
  * **Performance:** Optimize code for performance, particularly for display rendering and sensor data processing, to ensure smooth user experience.
  * **Visibility Optimization:** Always implement visibility control in UI components to prevent unnecessary updates when tiles are not visible, improving battery life and system performance.

-----

## Getting Started

1.  **ESP-IDF Setup:** Ensure you have the ESP-IDF development environment set up correctly. Follow the official [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html).
2.  **Clone Repository:** Clone this project's repository.
3.  **Install Components:** The `CMakeLists.txt` should automatically fetch the required components from the ESP-IDF Component Registry. If not, ensure your `idf_component.yml` (if used) or `CMakeLists.txt` correctly references them.
4.  **Build and Flash:**
    ```bash
    idf.py build
    idf.py flash
    ```
5.  **Monitor:**
    ```bash
    idf.py monitor
    ```

-----