# Chrono-G: ESP32-S3 AMOLED Round Display Project

A multi-functional aviation instrument display for the **Waveshare ESP32-S3-Touch-AMOLED-1.75** development board, featuring five horizontally swipeable pages.

## Project Overview

On startup the user selects an aircraft callsign, after which the main interface is shown with a persistent header displaying the selected callsign. Five pages are navigated by horizontal swipe:

| # | Page | Description |
|---|------|-------------|
| 1 | **Cyan Stopwatch** | Digital stopwatch with cyan theme — START/STOP/RESET |
| 2 | **Yellow Stopwatch** | Digital stopwatch with yellow theme — START/STOP/RESET |
| 3 | **Magenta Stopwatch** | Digital stopwatch with magenta theme — START/STOP/RESET/LAP |
| 4 | **G-Meter** | Real-time signed G-force with min/max tracking and color-coded warnings |
| 5 | **Turn Indicator** | Aircraft-style turn rate and slip/skid indicator with on-demand gyro calibration |

## Hardware

- **Development Board:** Waveshare ESP32-S3-Touch-AMOLED-1.75
  - [Product Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.75)
  - 1.75" Round AMOLED Display (466×466 pixels)
  - Capacitive touch screen
  - QMI8658 6-axis IMU (accelerometer + gyroscope)

## Key Features

- **Touch Navigation:** Horizontal swiping between pages via LVGL Tileview
- **Three Independent Stopwatches:** Separate timers with distinct color themes; the magenta stopwatch adds LAP recording
- **G-Force Monitoring:** Signed Z-axis G-force displayed at 8 Hz with color-coded warnings (white → orange → red), continuous min/max tracking, and tap-to-reset
- **Turn Indicator:** Rotating aircraft silhouette driven by gyroscope turn rate; slip/skid ball driven by accelerometer bank angle; standard-rate turn markings (3°/s = 2 min to 180°)
- **Aircraft Selector:** Startup callsign selection screen with configurable aircraft names
- **Persistent Aircraft Header:** Callsign banner visible across all tiles
- **Visibility-Aware Rendering:** Components continue sensor processing when hidden but skip UI updates, reducing needless redraws
- **On-Demand Gyro Calibration:** Turn Indicator page provides a hidden calibration button; device must be held still for ~5 seconds

## G-Force Color Thresholds

| Color | Condition |
|-------|-----------|
| White | 0.6 G – 1.4 G (normal) |
| Orange | Outside normal range |
| Red | ≤ −1.0 G or ≥ 3.8 G (danger) |

## Software Dependencies

This project uses the ESP-IDF framework with the following components from the ESP-IDF Component Registry:

- **LVGL Graphics Library:** `lvgl/lvgl` (v9.3.0) — GUI framework for all display elements
- **QMI8658 Driver:** `waveshare/qmi8658` (v1.0.1) — IMU sensor driver
- **Board Support Package:** `waveshare/esp32_s3_touch_amoled_1_75` (v1.0.1) — Display and touch drivers

## Project Structure

```
.
├── CMakeLists.txt
├── main/
│   ├── main.c
│   ├── main.h
│   ├── visibility_manager.c
│   ├── visibility_manager.h
│   ├── display_port.c
│   ├── display_port.h
│   ├── lv_font_g_meter_96.c        # Custom font for G-meter display
│   ├── lv_font_seven_segment_64.c  # Custom font for stopwatches
│   ├── CMakeLists.txt
│   ├── idf_component.yml
│   └── components/
│       ├── cyan_stopwatch/
│       ├── yellow_stopwatch/
│       ├── magenta_stopwatch/
│       ├── g_meter/
│       ├── turn_indicator/
│       ├── imu/
│       ├── aircraft_selector/
│       └── aircraft_header/
├── managed_components/
├── partitions.csv
├── sdkconfig.defaults
└── README.md
```

## Getting Started

### Prerequisites

1. **ESP-IDF Setup:** Install the ESP-IDF development environment following the [official guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html)
2. **Hardware:** Waveshare ESP32-S3-Touch-AMOLED-1.75 development board

### Building and Flashing

```bash
git clone <repository-url>
cd chrono-g
idf.py build
idf.py flash
idf.py monitor
```

## Development Notes

- **Visibility manager** (`main/visibility_manager.h`) tracks the active tile and notifies components via `<component>_set_visible(bool)` so they can skip UI work when hidden.
- **IMU module** (`main/components/imu`) runs a dedicated task at 125 Hz, performs gyro calibration on startup, and exposes `imu_get_g_extrema()`, `imu_reset_g_extrema()`, and related helpers.
- **Stopwatches** use independent FreeRTOS tick counters and a custom seven-segment font; only the magenta stopwatch supports lap recording.
- **Turn Indicator** updates at ~30 FPS; the calibration button is transparent and positioned in the lower quarter of the screen.
- When adding new tile components, provide a `<component>_set_visible(bool)` function and integrate it into the `tileview_event_cb` in `main.c` and the `tile_index_t` enum in `visibility_manager.h`.

## License

This project is open source. Please refer to the LICENSE file for details.
