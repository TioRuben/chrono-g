# Chrono-G: ESP32-S3 AMOLED Round Display Project

A multi-functional display project for the **Waveshare ESP32-S3-Touch-AMOLED-1.75** development board, featuring three horizontally swipeable pages with distinct functionalities.

## Project Overview

This project creates an interactive round display interface with three main pages:

- **Cyan Stopwatch:** A digital stopwatch with a cyan theme
- **Yellow Stopwatch:** A digital stopwatch with a yellow theme  
- **Artificial Horizon:** A graphical artificial horizon using accelerometer data

## Hardware

- **Development Board:** Waveshare ESP32-S3-Touch-AMOLED-1.75
  - [Product Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.75)
  - 1.75" Round AMOLED Display (390×390 pixels)
  - Capacitive Touch Screen
  - QMI8658 6-axis IMU (Accelerometer + Gyroscope)

## Key Features

- **Touch Navigation:** Horizontal swiping between pages using LVGL Tileview
- **Dual Independent Stopwatches:** Two separate timing functions with different themes
- **Artificial Horizon:** Real-time attitude display using filtered accelerometer data
- **Extended Kalman Filter (EKF):** Advanced filtering for accurate and responsive horizon display

## Software Dependencies

This project uses the ESP-IDF framework with the following components from the ESP-IDF Component Registry:

- **LVGL Graphics Library:** `lvgl/lvgl` (v9.3.0) - GUI framework
- **QMI8658 Driver:** `waveshare/qmi8658` (v1.0.1) - IMU sensor driver
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
│       └── artificial_horizon/
│           ├── artificial_horizon.c
│           └── artificial_horizon.h
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

- **Modular Design:** Each display page is implemented as a separate component for maintainability
- **Real-time Performance:** Optimized for smooth 60fps display updates and responsive touch input
- **Memory Efficiency:** Careful resource management for embedded environment constraints
- **Error Handling:** Robust error handling for sensor readings and display operations

## License

This project is open source. Please refer to the license file for details.
