# Copilot Instructions for ESP32-S3 AMOLED Round Display Project

This document provides guidelines and essential information for developing on the ESP32-S3 Round Display project.

-----

## Project Overview

This project targets the **Waveshare ESP32-S3-Touch-AMOLED-1.75** development board. The core functionality revolves around a round display with three horizontally swipeable pages, each dedicated to a specific function:

  * **Cyan Stopwatch:** A digital stopwatch with a cyan theme.
  * **Yellow Stopwatch:** A digital stopwatch with a yellow theme.
  * **Artificial Horizon:** A graphical representation of an artificial horizon, utilizing accelerometer data.

The project leverages two timers for the stopwatches and an accelerometer (QMI8658) for the artificial horizon.

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

The project is structured to ensure modularity and maintainability. Each display page will reside in its own `.c` file, accompanied by a corresponding header file (`.h`). These individual page files will then be included and managed within `main.c`.

```
.
├── main/
│   ├── main.c
│   ├── components/
│   │   ├── cyan_stopwatch/
│   │   │   ├── cyan_stopwatch.c
│   │   │   └── cyan_stopwatch.h
│   │   ├── yellow_stopwatch/
│   │   │   ├── yellow_stopwatch.c
│   │   │   └── yellow_stopwatch.h
│   │   └── artificial_horizon/
│   │       ├── artificial_horizon.c
│   │       └── artificial_horizon.h
│   └── CMakeLists.txt
├── CMakeLists.txt
└── sdkconfig.defaults
```

-----

## Key Features and Implementation Details

### Display Navigation

  * **LVGL Tileview:** The three display pages will be implemented as tiles within an **LVGL Tileview** object.
  * **Horizontal Swiping:** Users will navigate between the stopwatch pages and the artificial horizon by horizontally swiping across the display. Refer to the [LVGL Tileview Documentation](https://docs.lvgl.io/master/details/widgets/tileview.html) for implementation details.

### Stopwatch Functionality

  * **Two Independent Timers:** Each stopwatch (Cyan and Yellow) will operate independently using dedicated timers.
  * **Display Updates:** The stopwatch values will be updated and rendered on their respective LVGL pages.

### Artificial Horizon

  * **QMI8658 Accelerometer:** Accelerometer data from the QMI8658 will be used as input for the artificial horizon.
  * **Extended Kalman Filter (EKF):** The accelerometer data will be processed and filtered using an **Extended Kalman Filter (EKF)**.
      * **Goal:** Minimize lag while prioritizing accuracy in the horizon's representation.
      * The EKF implementation should be carefully optimized for real-time performance on the ESP32-S3.

-----

## Development Guidelines

  * **Code Style:** Adhere to standard C coding practices and ESP-IDF conventions.
  * **Modularity:** Keep functions and modules focused on a single responsibility.
  * **Resource Management:** Pay close attention to memory usage and task scheduling, especially given the embedded nature of the project.
  * **Error Handling:** Implement robust error handling for sensor readings, display operations, and timer management.
  * **Performance:** Optimize code for performance, particularly for the EKF and display rendering, to ensure a smooth user experience.

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