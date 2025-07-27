#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#include "qmi8658.h"
#include "cyan_stopwatch.h"
#include "yellow_stopwatch.h"
#include "artificial_horizon.h"

static const char *TAG = "Main";

// Tileview event callback for visibility optimization
static void tileview_event_cb(lv_event_t *e)
{
    lv_obj_t *tileview = lv_event_get_target(e);
    lv_obj_t *active_tile = lv_tileview_get_tile_act(tileview);

    // Get tile index by checking which tile is active
    // We can use the tile's user data or check the tile directly
    int32_t tile_index = lv_obj_get_index(active_tile);

    // Tile 2 is the artificial horizon (index 2)
    bool horizon_visible = (tile_index == 2);

    // Set visibility for artificial horizon
    artificial_horizon_set_visible(horizon_visible);

    ESP_LOGI(TAG, "Tile switched to index %d, horizon visible: %s",
             (int)tile_index, horizon_visible ? "true" : "false");
}

void app_main(void)
{
    // Initialize display with optimized buffer configuration
    // Reduce buffer size to save memory - use smaller partial buffer instead of full framebuffer
    bsp_display_cfg_t disp_cfg = {
        .double_buffer = true,   // Disable double buffering to save ~435KB memory
        .buffer_size = 466 * 50, // Use partial rendering buffer (50 lines instead of full screen)
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .flags = {
            .buff_dma = true,
            .buff_spiram = true}};

    lv_display_t *disp = bsp_display_start_with_config(&disp_cfg);
    // lv_display_t *disp = bsp_display_start();

    // Initialize IMU
    i2c_master_bus_handle_t bus_handle = bsp_i2c_get_handle();
    qmi8658_dev_t dev;
    esp_err_t ret = qmi8658_init(&dev, bus_handle, QMI8658_ADDRESS_HIGH);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize QMI8658 (error: %d)", ret);
        vTaskDelete(NULL);
    }

    qmi8658_set_accel_range(&dev, QMI8658_ACCEL_RANGE_8G);
    qmi8658_set_accel_odr(&dev, QMI8658_ACCEL_ODR_1000HZ);
    qmi8658_set_gyro_range(&dev, QMI8658_GYRO_RANGE_512DPS);
    qmi8658_set_gyro_odr(&dev, QMI8658_GYRO_ODR_1000HZ);
    qmi8658_set_accel_unit_mps2(&dev, true);
    qmi8658_set_gyro_unit_rads(&dev, true);
    qmi8658_set_display_precision(&dev, 4);

    // Create and initialize dark theme
    lv_theme_t *theme = lv_theme_default_init(disp,
                                              lv_color_hex(0x1E1E1E), // Primary color (black)
                                              lv_color_hex(0x3E3E3E), // Secondary color (dark gray)
                                              true,                   // Dark mode
                                              LV_FONT_DEFAULT);       // Default font

    lv_disp_set_theme(disp, theme);
    bsp_display_rotate(disp, LV_DISP_ROTATION_270);
    bsp_display_lock(0);

    // Set screen background to pure black
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);

    // Create tileview
    lv_obj_t *tileview = lv_tileview_create(lv_scr_act());
    lv_obj_set_size(tileview, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(tileview, lv_color_hex(0x000000), LV_PART_MAIN);

    // Create tiles for each component
    lv_obj_t *cyan_tile = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_HOR);
    lv_obj_t *yellow_tile = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_HOR);
    lv_obj_t *horizon_tile = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR);

    // Make tiles borderless
    lv_obj_set_style_border_width(cyan_tile, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(yellow_tile, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(horizon_tile, 0, LV_PART_MAIN);

    // Initialize components
    cyan_stopwatch_init(cyan_tile);
    yellow_stopwatch_init(yellow_tile);
    artificial_horizon_init(horizon_tile, &dev);

    // Add tileview event handler for visibility optimization
    lv_obj_add_event_cb(tileview, tileview_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    bsp_display_unlock();

    // Main loop is now only for system tasks
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}