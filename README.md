# Chrono-G: ESP32-S3 AMOLED Round Display Project

A multi-functional display project for the **Waveshare ESP32-S3-Touch-AMOLED-1.75** development board, featuring five horizontally swipeable pages with distinct functionality.

## Project Overview

This project creates an interactive round display interface with five main pages:

- **Cyan Stopwatch:** A digital stopwatch with a cyan theme
- **Yellow Stopwatch:** A digital stopwatch with a yellow theme
- **Magenta Stopwatch:** A digital stopwatch with a magenta theme with LAP reset
- **G-Meter:** Real-time signed G-force display with min/max tracking and color-coded warnings
- **Turn Indicator:** Aircraft-style turn/rate slip/skid indicator derived from IMU data

## Hardware

- **Development Board:** Waveshare ESP32-S3-Touch-AMOLED-1.75
  - [Product Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.75)
  - 1.75" Round AMOLED Display (466x466 pixels)
  - Capacitive Touch Screen
  - QMI8658 6-axis IMU (Accelerometer + Gyroscope)

## Key Features

- **Touch Navigation:** Horizontal swiping between pages using LVGL Tileview
- **Dual Independent Stopwatches:** Two separate timing functions with different themes
- **Artificial Horizon:** Real-time attitude display using ESP-DSP Extended Kalman Filter for optimal accuracy
- **Advanced Sensor Fusion:** ESP-DSP library implementation with optimized filtering parameters
- **G-Force Monitoring:** Real-time load factor calculation with color-coded warnings and click-to-reset max G tracking
- **Performance Optimized:** Motion threshold-based updates (1° sensitivity), adaptive task scheduling, and ~60 FPS rendering
- **Clean Code Architecture:** Modular design with eliminated code duplications, optimized includes, and improved memory efficiency
- **Robust Error Handling:** Comprehensive error checking and graceful failure recovery

## Software Dependencies

This project uses the ESP-IDF framework with the following components from the ESP-IDF Component Registry:

- **LVGL Graphics Library:** `lvgl/lvgl` (v9.3.0) - GUI framework for all display elements
- **QMI8658 Driver:** `waveshare/qmi8658` (v1.0.1) - IMU sensor driver for accelerometer/gyroscope data
- **Board Support Package:** `waveshare/esp32_s3_touch_amoled_1_75` (v1.0.1) - Display and touch drivers

## Project Structure

```
.
├── CMakeLists.txt
├── main/
│   ├── main.c
│   ├── CMakeLists.txt
│   ├── idf_component.yml
│   └── components/
│       ├── cyan_stopwatch/
│       │   ├── cyan_stopwatch.c
│       │   └── cyan_stopwatch.h
│       ├── yellow_stopwatch/
│       │   ├── yellow_stopwatch.c
│       │   └── yellow_stopwatch.h
│       ├── magenta_stopwatch/
│       │   ├── magenta_stopwatch.c
│       │   └── magenta_stopwatch.h
│       ├── g_meter/
│       │   ├── g_meter.c
│       │   └── g_meter.h
│       ├── turn_indicator/
│       │   ├── turn_indicator.c
│       │   └── turn_indicator.h
│       ├── imu/
│       │   ├── imu.c
│       │   └── imu.h
│       ├── aircraft_selector/
│       │   ├── aircraft_selector.c
│       │   └── aircraft_selector.h
│       └── aircraft_header/
│           ├── aircraft_header.c
│           └── aircraft_header.h
├── managed_components/
├── partitions.csv
├── sdkconfig.defaults
└── README.md
```

## Getting Started

### Prerequisites

1. **ESP-IDF Setup:** Install ESP-IDF development environment following the [official guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html)
2. **Hardware:** Waveshare ESP32-S3-Touch-AMOLED-1.75 development board

### Building and Flashing

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd chrono-g
   ```

2. **Build the project:**
   ```bash
   idf.py build
   ```

3. **Flash to device:**
   ```bash
   idf.py flash
   ```

4. **Monitor output:**
   ```bash
   idf.py monitor
   ```

## Development Notes

- The visibility manager (see `main/visibility_manager.h`) tracks which tile is active and notifies components so they can avoid unnecessary UI updates when hidden.
- The IMU module (`main/components/imu`) handles gyro calibration and provides filtered accel/gyro samples to the G-Meter and Turn Indicator.
- Stopwatches use independent timers and custom seven-segment fonts for display.
- When adding new tile components, follow the existing pattern: provide a `<component>_set_visible(bool)` function and integrate it into `main.c` tile setup and the visibility manager.

## License

This project is open source. Please refer to the license file for details.
