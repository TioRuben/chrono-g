#include "turn_indicator.h"
#include "main.h"
#include "imu.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "bsp/display.h"

static const char *TAG = "TURN_INDICATOR";

// Update rate: 60ms as requested
#define UPDATE_INTERVAL_MS 33

LV_IMG_DECLARE(turn_coord_img);
LV_IMG_DECLARE(aircraft_turn_img);

// Component state
static struct
{
    float turn_rate;
    float gravity_angle;
    float turn_time_180;
    float last_displayed_turn_rate;
    float last_displayed_gravity_angle;
    float last_displayed_turn_time_180;
    bool is_visible;
    lv_timer_t *update_timer;
    QueueHandle_t imu_queue;
    lv_obj_t *aircraft_img;   // Aircraft image object for rotation
    lv_obj_t *slip_skid_ball; // Aircraft image object for rotation
    lv_obj_t *calib_btn;      // Calibration button
    bool calibrating;         // Flag to track calibration state
} turn_indicator_state = {
    .turn_rate = 0.0f,
    .gravity_angle = 0.0f,
    .turn_time_180 = 0.0f,
    .last_displayed_turn_rate = 999.0f,     // Initialize to different value to force first update
    .last_displayed_gravity_angle = 999.0f, // Initialize to different value to force first update
    .last_displayed_turn_time_180 = 999.0f, // Initialize to different value to force first update
    .is_visible = false,
    .update_timer = NULL,
    .imu_queue = NULL,
    .aircraft_img = NULL,
    .slip_skid_ball = NULL,
    .calib_btn = NULL,
    .calibrating = false};

/**
 * @brief Timer callback to restore button state
 */
static void hide_calib_btn_cb(lv_timer_t *timer)
{
    // Re-enable turn indicator updates
    turn_indicator_state.calibrating = false;

    // Hide the button
    lv_obj_set_style_bg_opa(turn_indicator_state.calib_btn, LV_OPA_0, LV_PART_MAIN);

    // Clear any text
    lv_obj_t *label = lv_obj_get_child(turn_indicator_state.calib_btn, 0);
    if (label)
    {
        lv_label_set_text(label, "");
    }
}

/**
 * @brief Calibration complete callback
 */
static void calibration_complete_cb(void *params)
{
    bool success = (bool)params;
    lv_obj_t *label = lv_obj_get_child(turn_indicator_state.calib_btn, 0);

    if (success)
    {
        if (label)
        {
            lv_label_set_text(label, "Calibration Complete");
            lv_obj_set_style_text_color(label, lv_color_hex(0x00FF00), 0);
        }
    }
    else
    {
        if (label)
        {
            lv_label_set_text(label, "Calibration Failed");
            lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), 0);
        }
    }

    // Create timer to hide button after 2 seconds
    lv_timer_t *hide_timer = lv_timer_create(hide_calib_btn_cb, 2000, NULL);
    lv_timer_set_repeat_count(hide_timer, 1);
}

/**
 * @brief Task to perform IMU calibration
 */
static void calibration_task(void *params)
{
    esp_err_t calib_ret = imu_calibrate_gyro(5000);

    // Schedule UI update in main thread via LVGL
    lv_async_call(calibration_complete_cb, (void *)(calib_ret == ESP_OK));

    vTaskDelete(NULL);
}

/**
 * @brief Calibration button event handler
 */
static void calib_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED && !turn_indicator_state.calibrating)
    {
        // Set calibrating state to pause turn indicator updates
        turn_indicator_state.calibrating = true;

        // Make button visible with calibration message
        lv_obj_set_style_bg_opa(turn_indicator_state.calib_btn, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_t *label = lv_obj_get_child(turn_indicator_state.calib_btn, 0);
        if (label)
        {
            lv_label_set_text(label, "Keep aircraft still!\nCalibrating...");
            lv_obj_set_style_text_color(label, lv_color_white(), 0);
        }

        // Start calibration in a separate task
        BaseType_t ret = xTaskCreate(calibration_task, "imu_calib", 4096, NULL, 5, NULL);

        if (ret != pdPASS)
        {
            // If task creation fails, show error immediately
            if (label)
            {
                lv_label_set_text(label, "Calibration Failed");
                lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), 0);
            }
            // Create timer to hide button after error
            lv_timer_t *hide_timer = lv_timer_create(hide_calib_btn_cb, 2000, NULL);
            lv_timer_set_repeat_count(hide_timer, 1);
        }
    }
}

/**
 * @brief Calculate gravity vector angle between Z and Y axis
 * Returns signed angle in degrees where 0° means gravity vector is fully aligned with Z axis
 * Positive angle means tilting towards Y axis, negative means tilting away from Y axis
 */
static float calculate_gravity_vector_angle(float accel_y_mg, float accel_x_mg)
{
    // Calculate signed angle between gravity vector and Z axis using Y and Z components
    // atan2(y, z) gives angle from Z axis towards Y axis with proper sign
    float angle_rad = atan2f(accel_y_mg, fabsf(accel_x_mg));
    float angle_deg = angle_rad * 180.0f / M_PI;

    return angle_deg;
}

/**
 * @brief Update turn indicator display with latest IMU data
 */
static void update_turn_indicator(lv_timer_t *timer)
{
    // Skip updates when not visible or calibrating
    if (!turn_indicator_state.is_visible || turn_indicator_state.calibrating)
    {
        return;
    }

    // Get IMU queue if not available
    if (!turn_indicator_state.imu_queue)
    {
        turn_indicator_state.imu_queue = get_imu_queue();
        if (!turn_indicator_state.imu_queue)
        {
            return;
        }
    }

    // Process IMU data if available
    imu_data_t imu_data;
    if (xQueueReceive(turn_indicator_state.imu_queue, &imu_data, 0) == pdTRUE)
    {
        turn_indicator_state.turn_rate = imu_data.gyro_x;
        turn_indicator_state.gravity_angle = calculate_gravity_vector_angle(imu_data.accel_y, imu_data.accel_x);

        if (turn_indicator_state.aircraft_img)
        {
            // 3°/s turn rate = 20° rotation mapping kept for consistency
            float rotation_angle = -turn_indicator_state.turn_rate * (20.0f / 3.0f);

            // Cap rotation to ±40°
            if (rotation_angle > 40.0f)
            {
                rotation_angle = 40.0f;
            }
            else if (rotation_angle < -40.0f)
            {
                rotation_angle = -40.0f;
            }

            int32_t rotation_angle_x10 = (int32_t)(rotation_angle * 10.0f);

            bsp_display_lock(0);
            lv_image_set_rotation(turn_indicator_state.aircraft_img, rotation_angle_x10);
            bsp_display_unlock();
        }

        if (turn_indicator_state.slip_skid_ball)
        {
            int32_t rotation = (int16_t)(turn_indicator_state.gravity_angle * 7.0f) - 70;
            if (rotation > 160)
            {
                rotation = 160; // Cap at 16.0 (10x) => ~16.0°
            }
            else if (rotation < -160)
            {
                rotation = -160;
            }
            bsp_display_lock(0);
            lv_obj_set_style_transform_rotation(turn_indicator_state.slip_skid_ball,
                                                rotation, LV_PART_MAIN);
            bsp_display_unlock();
        }
    }
}

lv_obj_t *turn_indicator_init(lv_obj_t *parent)
{
    ESP_LOGI(TAG, "Initializing turn indicator component");
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(container, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);

    // Get IMU queue handle
    turn_indicator_state.imu_queue = get_imu_queue();
    ESP_LOGI(TAG, "IMU queue handle: %p", turn_indicator_state.imu_queue);
    if (!turn_indicator_state.imu_queue)
    {
        ESP_LOGW(TAG, "IMU queue not available yet, will try to get it later");
    }

    // Load image turn_coord_img
    lv_obj_t *bg_img = lv_img_create(parent);
    lv_img_set_src(bg_img, &turn_coord_img);
    lv_obj_set_size(bg_img, LV_PCT(100), LV_PCT(100));
    lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);

    // Load aircraft image and store reference in state
    turn_indicator_state.aircraft_img = lv_img_create(parent);
    lv_img_set_src(turn_indicator_state.aircraft_img, &aircraft_turn_img);
    lv_obj_set_size(turn_indicator_state.aircraft_img, LV_PCT(100), LV_PCT(100));
    lv_obj_align(turn_indicator_state.aircraft_img, LV_ALIGN_CENTER, 0, 0);

    turn_indicator_state.slip_skid_ball = lv_obj_create(parent);
    lv_obj_set_size(turn_indicator_state.slip_skid_ball, 42, 42);
    lv_obj_set_style_bg_color(turn_indicator_state.slip_skid_ball, lv_color_hex(0x222222), LV_PART_MAIN);
    lv_obj_set_style_border_width(turn_indicator_state.slip_skid_ball, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(turn_indicator_state.slip_skid_ball, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_align(turn_indicator_state.slip_skid_ball, LV_ALIGN_BOTTOM_MID, 0, -37);
    lv_obj_set_style_radius(turn_indicator_state.slip_skid_ball, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_y(turn_indicator_state.slip_skid_ball, -435, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_x(turn_indicator_state.slip_skid_ball, 20, LV_PART_MAIN);

    // Create label at 100px from bottom with "1 min." text
    lv_obj_t *time_label = lv_label_create(parent);
    lv_label_set_text(time_label, "2 min.");
    lv_obj_set_style_text_color(time_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(time_label, LV_ALIGN_BOTTOM_MID, 0, -100);

    // Create hidden calibration button in bottom quarter
    turn_indicator_state.calib_btn = lv_btn_create(parent);
    lv_obj_set_size(turn_indicator_state.calib_btn, LV_PCT(100), LV_PCT(25));
    lv_obj_align(turn_indicator_state.calib_btn, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(turn_indicator_state.calib_btn, LV_OPA_0, LV_PART_MAIN); // Start transparent
    lv_obj_set_style_radius(turn_indicator_state.calib_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(turn_indicator_state.calib_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(turn_indicator_state.calib_btn, lv_color_hex(0x404040), LV_PART_MAIN);
    lv_obj_add_event_cb(turn_indicator_state.calib_btn, calib_btn_event_cb, LV_EVENT_CLICKED, NULL);

    // Create label for the button
    lv_obj_t *btn_label = lv_label_create(turn_indicator_state.calib_btn);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_align(btn_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
    lv_label_set_text(btn_label, ""); // Start empty
    lv_obj_center(btn_label);

    ESP_LOGI(TAG, "Drew initial turn indicator markings");

    // Create update timer (60ms interval as requested)
    turn_indicator_state.update_timer = lv_timer_create(update_turn_indicator, UPDATE_INTERVAL_MS, NULL);
    if (turn_indicator_state.update_timer)
    {
        ESP_LOGI(TAG, "Created update timer with %dms interval", UPDATE_INTERVAL_MS);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to create update timer");
    }

    ESP_LOGI(TAG, "Turn indicator initialization complete");
    return container;
}

void turn_indicator_set_visible(bool visible)
{
    turn_indicator_state.is_visible = visible;
    ESP_LOGI(TAG, "Visibility set to: %s", visible ? "visible" : "hidden");

    // If becoming visible, force UI update on next timer call
    if (visible)
    {
        turn_indicator_state.last_displayed_turn_rate = 999.0f;
        turn_indicator_state.last_displayed_gravity_angle = 999.0f;
        turn_indicator_state.last_displayed_turn_time_180 = 999.0f;
    }
    else
    {
        // Hide calibration button when component becomes invisible
        lv_obj_set_style_bg_opa(turn_indicator_state.calib_btn, LV_OPA_0, LV_PART_MAIN);
        // Clear any text
        lv_obj_t *label = lv_obj_get_child(turn_indicator_state.calib_btn, 0);
        if (label)
        {
            lv_label_set_text(label, "");
        }
        // Reset calibration state
        turn_indicator_state.calibrating = false;
    }
}
