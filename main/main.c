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

// Global variables for stopwatch
static uint32_t seconds_counted = 0;
static bool stopwatch_running = false;
static lv_timer_t *timer = NULL;
static lv_obj_t *time_label = NULL;
static lv_obj_t *btn = NULL;
static lv_obj_t *reset_btn = NULL;

char *TAG = "Stopwatch - Main";

// Convert seconds to HH:MM:SS format
static void format_time(char *buf, size_t buf_size, uint32_t total_seconds)
{
    uint32_t hours = total_seconds / 3600;
    uint32_t minutes = (total_seconds % 3600) / 60;
    uint32_t seconds = total_seconds % 60;
    snprintf(buf, buf_size, "%02u:%02u:%02u", (unsigned int)hours, (unsigned int)minutes, (unsigned int)seconds);
}

// Timer callback to update stopwatch
static void update_stopwatch(lv_timer_t *timer)
{
    if (stopwatch_running)
    {
        seconds_counted++;
        char buf[16];
        format_time(buf, sizeof(buf), seconds_counted);
        lv_label_set_text(time_label, buf);
    }
}

// Button event handler
static void btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (target == btn)
    {
        if (!stopwatch_running)
        {
            // Starting the stopwatch
            stopwatch_running = true;
            lv_timer_resume(timer); // Resume the timer
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x770000), LV_PART_MAIN);
            lv_label_set_text(lv_obj_get_child(btn, 0), "STOP");
            lv_obj_set_size(btn, lv_pct(100), lv_pct(100));
            if (reset_btn)
            {
                lv_obj_add_flag(reset_btn, LV_OBJ_FLAG_HIDDEN);
            }
        }
        else
        {
            // Stopping the stopwatch
            stopwatch_running = false;
            lv_timer_pause(timer); // Pause the timer
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x007700), LV_PART_MAIN);
            lv_label_set_text(lv_obj_get_child(btn, 0), "         START");
            lv_obj_set_size(btn, lv_pct(50), lv_pct(100));

            if (seconds_counted > 0)
            {
                // Show reset button
                if (!reset_btn)
                {
                    // Create reset button
                    reset_btn = lv_btn_create(lv_obj_get_parent(btn));
                    lv_obj_set_size(reset_btn, lv_pct(50), lv_pct(100));
                    lv_obj_set_style_bg_color(reset_btn, lv_color_hex(0x808080), LV_PART_MAIN);
                    lv_obj_set_style_radius(reset_btn, 0, LV_PART_MAIN);
                    lv_obj_set_style_border_width(reset_btn, 0, LV_PART_MAIN);
                    lv_obj_set_style_pad_all(reset_btn, 0, LV_PART_MAIN);
                    lv_obj_t *reset_label = lv_label_create(reset_btn);
                    lv_label_set_text(reset_label, "RESET         ");
                    lv_obj_set_style_text_align(reset_label, LV_TEXT_ALIGN_LEFT, 0);
                    lv_obj_set_style_text_font(reset_label, &lv_font_montserrat_24, 0);
                    lv_obj_center(reset_label);
                    lv_obj_add_event_cb(reset_btn, btn_event_cb, LV_EVENT_CLICKED, NULL);
                }
                else
                {
                    lv_obj_clear_flag(reset_btn, LV_OBJ_FLAG_HIDDEN);
                }
            }
        }
    }
    else if (target == reset_btn)
    {
        // Reset the stopwatch
        seconds_counted = 0;
        lv_label_set_text(time_label, "00:00:00");
        lv_obj_del(reset_btn); // Delete the reset button instead of hiding it
        reset_btn = NULL;
        stopwatch_running = false;
        lv_timer_pause(timer);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x007700), LV_PART_MAIN);
        lv_obj_set_size(btn, lv_pct(100), lv_pct(100)); // Restore full width
        lv_label_set_text(lv_obj_get_child(btn, 0), "START");
    }
}

void app_main(void)
{
    lv_display_t *disp = bsp_display_start();

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

    // Create title label
    lv_obj_t *title_label = lv_label_create(lv_scr_act());
    lv_label_set_text(title_label, "EC-HH4");
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_32, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 20);

    // Create time display label
    time_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_48, 0);
    lv_label_set_text(time_label, "00:00:00");
    // lv_obj_set_style_transform_scale(time_label, 1.2, LV_PART_MAIN);
    lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 0);

    // Create button container that takes the bottom fourth of the screen
    lv_obj_t *btn_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(btn_container, lv_pct(100), lv_pct(25));
    lv_obj_align(btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(btn_container, LV_OPA_0, 0);
    lv_obj_set_style_pad_all(btn_container, 0, 0);
    lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_border_width(btn_container, 0, 0);
    lv_obj_set_style_pad_all(btn_container, 0, LV_PART_MAIN);

    // Create start/stop button
    btn = lv_btn_create(btn_container);
    lv_obj_set_size(btn, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x007700), LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn, 0, LV_PART_MAIN);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "START");
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_24, 0);
    lv_obj_center(btn_label);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    // Create and start the timer for stopwatch updates
    timer = lv_timer_create(update_stopwatch, 1000, NULL);
    lv_timer_pause(timer);

    bsp_display_unlock();

    qmi8658_data_t data;

    while (1)
    {
        bool ready;
        ret = qmi8658_is_data_ready(&dev, &ready);
        if (ret == ESP_OK && ready)
        {
            ret = qmi8658_read_sensor_data(&dev, &data);
            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "Accel: X=%.4f m/s², Y=%.4f m/s², Z=%.4f m/s²",
                         data.accelX, data.accelY, data.accelZ);
                ESP_LOGI(TAG, "Gyro:  X=%.4f rad/s, Y=%.4f rad/s, Z=%.4f rad/s",
                         data.gyroX, data.gyroY, data.gyroZ);
                ESP_LOGI(TAG, "Temp:  %.2f °C, Timestamp: %lu",
                         data.temperature, data.timestamp);
                ESP_LOGI(TAG, "----------------------------------------");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read sensor data (error: %d)", ret);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Data not ready or error reading status (error: %d)", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow other tasks to run
    }
}