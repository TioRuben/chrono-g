#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "display_port.h"
#include "cyan_stopwatch.h"
#include "yellow_stopwatch.h"
#include "magenta_stopwatch.h"
#include "g_meter.h"
#include "turn_indicator.h"
#include "imu.h"
#include "visibility_manager.h"
#include "aircraft_selector.h"
#include "aircraft_header.h"
#include <math.h>

static const char *TAG = "Main";

// IMU data queue (single element queue for latest data)
static QueueHandle_t imu_queue = NULL;

// Global UI objects
static lv_obj_t *aircraft_selector_obj = NULL;
static lv_obj_t *aircraft_header_obj = NULL;
static lv_obj_t *tileview = NULL;

// /**
//  * @brief Convert calibration status enum to readable string
//  *
//  * @param status Calibration status enum
//  * @return const char* Human-readable status string
//  */
// static const char *get_calibration_status_string(imu_calibration_status_t status)
// {
//     switch (status)
//     {
//     case IMU_CALIBRATION_NOT_STARTED:
//         return "NOT_STARTED";
//     case IMU_CALIBRATION_IN_PROGRESS:
//         return "IN_PROGRESS";
//     case IMU_CALIBRATION_COMPLETED:
//         return "COMPLETED";
//     default:
//         return "UNKNOWN";
//     }
// }

// /**
//  * @brief Log IMU data for debugging
//  *
//  * @param imu_data Pointer to IMU data structure
//  */
// static void log_imu_data(const imu_data_t *imu_data)
// {
//     // Calculate G-load factor
//     float g_load = imu_calculate_g_load_factor(
//         imu_data->accel_x, imu_data->accel_y, imu_data->accel_z);

//     // Calculate turn rate like aircraft turn indicator
//     float turn_rate = imu_calculate_turn_rate(
//         imu_data->gyro_x, imu_data->gyro_y, imu_data->gyro_z);

//     // Log calibration status and timestamp
//     ESP_LOGI(TAG, "=== IMU Data Report ===");
//     ESP_LOGI(TAG, "Calibration: %s | Timestamp: %lld us",
//              get_calibration_status_string(imu_data->calibration_status),
//              imu_data->timestamp);

//     // Log raw accelerometer data
//     ESP_LOGI(TAG, "Accelerometer (mg): X=%.3f, Y=%.3f, Z=%.3f",
//              imu_data->accel_x, imu_data->accel_y, imu_data->accel_z);

//     // Log raw gyroscope data
//     ESP_LOGI(TAG, "Gyroscope (deg/s): X=%.3f, Y=%.3f, Z=%.3f",
//              imu_data->gyro_x, imu_data->gyro_y, imu_data->gyro_z);

//     // Log G-load factor
//     ESP_LOGI(TAG, "G-Load Factor: %.2f G", g_load);

//     // Log turn rate (aircraft turn indicator style)
//     ESP_LOGI(TAG, "Turn Rate: %.1f°/s %s",
//              fabsf(turn_rate),
//              turn_rate > 0.1f ? "(Right)" : turn_rate < -0.1f ? "(Left)"
//                                                               : "(Straight)");
//     if (fabsf(turn_rate) >= 2.5f && fabsf(turn_rate) <= 3.5f)
//     {
//         ESP_LOGI(TAG, "*** STANDARD RATE TURN ***");
//     }

//     ESP_LOGI(TAG, "=======================");
// }

// Getter function to access IMU queue from other modules
QueueHandle_t get_imu_queue(void)
{
    return imu_queue;
}

/**
 * @brief Callback function called when user selects an aircraft
 */
static void aircraft_selection_callback(const char *aircraft_id)
{
    ESP_LOGI(TAG, "Aircraft selection callback: %s", aircraft_id);

    // Clean up the aircraft selector
    if (aircraft_selector_obj)
    {
        aircraft_selector_cleanup(aircraft_selector_obj);
        aircraft_selector_obj = NULL;
    }

    // Create the aircraft header with the selected aircraft ID
    aircraft_header_obj = aircraft_header_init(lv_scr_act(), aircraft_id);

    // Show the main application screen (tileview)
    if (tileview)
    {
        lv_obj_clear_flag(tileview, LV_OBJ_FLAG_HIDDEN);
    }

    ESP_LOGI(TAG, "Main application screen is now visible with aircraft: %s", aircraft_id);
}

// Tileview event callback for visibility optimization
static void tileview_event_cb(lv_event_t *e)
{
    lv_obj_t *tileview = lv_event_get_target(e);
    lv_obj_t *active_tile = lv_tileview_get_tile_act(tileview);

    // Get tile index by checking which tile is active
    int32_t tile_index = lv_obj_get_index(active_tile);

    ESP_LOGI(TAG, "Tile switched to index %d", (int)tile_index);

    // Update visibility manager with the new active tile
    visibility_manager_set_tile_visible((tile_index_t)tile_index, true);

    // Notify individual components of their visibility state
    switch (tile_index)
    {
    case TILE_CYAN_STOPWATCH:
        cyan_stopwatch_set_visible(true);
        yellow_stopwatch_set_visible(false);
        magenta_stopwatch_set_visible(false);
        g_meter_set_visible(false);
        turn_indicator_set_visible(false);
        break;
    case TILE_YELLOW_STOPWATCH:
        cyan_stopwatch_set_visible(false);
        yellow_stopwatch_set_visible(true);
        magenta_stopwatch_set_visible(false);
        g_meter_set_visible(false);
        turn_indicator_set_visible(false);
        break;
    case TILE_MAGENTA_STOPWATCH:
        cyan_stopwatch_set_visible(false);
        yellow_stopwatch_set_visible(false);
        magenta_stopwatch_set_visible(true);
        g_meter_set_visible(false);
        turn_indicator_set_visible(false);
        break;
    case TILE_G_METER:
        cyan_stopwatch_set_visible(false);
        yellow_stopwatch_set_visible(false);
        magenta_stopwatch_set_visible(false);
        g_meter_set_visible(true);
        turn_indicator_set_visible(false);
        break;
    case TILE_TURN_INDICATOR:
        cyan_stopwatch_set_visible(false);
        yellow_stopwatch_set_visible(false);
        magenta_stopwatch_set_visible(false);
        g_meter_set_visible(false);
        turn_indicator_set_visible(true);
        break;
    default:
        ESP_LOGW(TAG, "Unknown tile index: %d", (int)tile_index);
        break;
    }
}

void app_main(void)
{
    // Initialize visibility manager first
    visibility_manager_init();

    // Initialize display with optimized buffer configuration
    // Reduce buffer size to save memory - use smaller partial buffer instead of full framebuffer
    // bsp_display_cfg_t disp_cfg = {
    //     .double_buffer = true,        // Disable double buffering to save ~435KB memory
    //     .buffer_size = 466 * 466 / 6, // Use partial rendering buffer (50 lines instead of full screen)
    //     .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
    //     .flags = {
    //         .buff_dma = true,
    //         .buff_spiram = true}};

    // Start display using local port (no managed component changes)
    const lvgl_port_cfg_t port_cfg = {
        .task_priority = 6,
        .task_stack = 7168,
        .task_affinity = 0,
        .task_max_sleep_ms = 100,
        .timer_period_ms = 2,
    };
    lv_display_t *disp = app_display_start(&port_cfg,
                                           232,   // taller buffer height to reduce flush count
                                           true,  // double buffer
                                           80,    // QSPI pclk MHz (fallback to 40 if unstable)
                                           true); // enable touch
    if (!disp)
    {
        ESP_LOGE(TAG, "Display initialization failed. Halting UI setup.");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Create UI under LVGL lock to avoid race with LVGL task
    lvgl_port_lock(0);

    // Create and initialize dark theme
    lv_theme_t *theme = lv_theme_default_init(disp,
                                              lv_color_hex(0x1E1E1E), // Primary color (black)
                                              lv_color_hex(0x3E3E3E), // Secondary color (dark gray)
                                              true,                   // Dark mode
                                              LV_FONT_DEFAULT);       // Default font

    lv_disp_set_theme(disp, theme);
    // bsp_display_rotate(disp, LV_DISP_ROTATION_270);
    // Set screen background to pure black
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);

    // Create tileview (initially hidden)
    tileview = lv_tileview_create(lv_scr_act());
    lv_obj_set_size(tileview, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(tileview, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_add_flag(tileview, LV_OBJ_FLAG_HIDDEN); // Hide initially until aircraft is selected

    // Create tiles for each component
    lv_obj_t *cyan_tile = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_HOR);
    lv_obj_t *yellow_tile = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_HOR);
    lv_obj_t *magenta_tile = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR);
    lv_obj_t *g_meter_tile = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_HOR);
    lv_obj_t *turn_indicator_tile = lv_tileview_add_tile(tileview, 4, 0, LV_DIR_HOR);
    lv_obj_set_style_anim(tileview, NULL, LV_PART_MAIN); // Disable animations for performance

    // Make tiles borderless
    lv_obj_set_style_border_width(cyan_tile, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(yellow_tile, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(magenta_tile, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_meter_tile, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(turn_indicator_tile, 0, LV_PART_MAIN);

    // Initialize components
    cyan_stopwatch_init(cyan_tile);
    yellow_stopwatch_init(yellow_tile);
    magenta_stopwatch_init(magenta_tile);
    g_meter_init(g_meter_tile);
    turn_indicator_init(turn_indicator_tile);

    // Add tileview event handler for visibility optimization
    lv_obj_add_event_cb(tileview, tileview_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Create aircraft selection screen
    aircraft_selector_obj = aircraft_selector_init(lv_scr_act(), aircraft_selection_callback);

    lvgl_port_unlock();

    // Make sure LVGL timers are enabled (defensive; display_port re-enables as well)
    lv_timer_enable(true);

    // Create IMU data queue (single element for latest data)
    imu_queue = xQueueCreate(1, sizeof(imu_data_t));
    if (imu_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create IMU queue");
        vTaskDelete(NULL);
    }

    // Initialize IMU module
    esp_err_t ret = imu_init(imu_queue);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize IMU module (error: %d)", ret);
        vTaskDelete(NULL);
    }

    // Main loop - read IMU data and log values every 5 seconds
    // imu_data_t imu_data;
    while (1)
    {
        // // Try to read the latest IMU data from queue (non-blocking)
        // if (xQueueReceive(imu_queue, &imu_data, 0) == pdTRUE)
        // {
        //     // Log comprehensive IMU data using the new logging function
        //     log_imu_data(&imu_data);
        // }
        // else
        // {
        //     ESP_LOGW(TAG, "No IMU data available in queue");
        // }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}