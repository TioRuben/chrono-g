#include "turn_indicator.h"
#include "main.h"
#include "imu.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "TURN_INDICATOR";

// Update rate: 60ms as requested
#define UPDATE_INTERVAL_MS 60

// Component state
static struct
{
    float turn_rate;
    float gravity_angle;
    float last_displayed_turn_rate;
    float last_displayed_gravity_angle;
    bool is_visible;
    lv_timer_t *update_timer;
    lv_obj_t *turn_rate_label;
    lv_obj_t *gravity_angle_label;
    QueueHandle_t imu_queue;
} turn_indicator_state = {
    .turn_rate = 0.0f,
    .gravity_angle = 0.0f,
    .last_displayed_turn_rate = 999.0f,     // Initialize to different value to force first update
    .last_displayed_gravity_angle = 999.0f, // Initialize to different value to force first update
    .is_visible = false,
    .update_timer = NULL,
    .turn_rate_label = NULL,
    .gravity_angle_label = NULL,
    .imu_queue = NULL};

/**
 * @brief Calculate gravity vector angle between Z and Y axis
 * Returns angle in degrees where 0° means gravity vector is fully aligned with Z axis
 */
static float calculate_gravity_vector_angle(float accel_y_mg, float accel_z_mg)
{
    // Calculate angle between gravity vector and Z axis using Y and Z components
    // atan2(y, z) gives angle from Z axis towards Y axis
    float angle_rad = atan2f(fabsf(accel_y_mg), fabsf(accel_z_mg));
    float angle_deg = angle_rad * 180.0f / M_PI;

    return angle_deg;
}

/**
 * @brief Update turn indicator display with latest IMU data
 */
static void update_turn_indicator(lv_timer_t *timer)
{
    // Skip updates when not visible
    if (!turn_indicator_state.is_visible)
    {
        return;
    }

    // Skip if labels not ready
    if (!turn_indicator_state.turn_rate_label || !turn_indicator_state.gravity_angle_label)
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
        // Calculate turn rate (Z-axis gyroscope)
        turn_indicator_state.turn_rate = imu_calculate_turn_rate(
            imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);

        // Calculate gravity vector angle
        turn_indicator_state.gravity_angle = calculate_gravity_vector_angle(
            imu_data.accel_y, imu_data.accel_z);
    }

    // Check if UI updates are needed (only update if significant change)
    bool need_turn_rate_update = fabsf(turn_indicator_state.turn_rate - turn_indicator_state.last_displayed_turn_rate) >= 0.1f;
    bool need_gravity_angle_update = fabsf(turn_indicator_state.gravity_angle - turn_indicator_state.last_displayed_gravity_angle) >= 0.5f;

    // Update turn rate display
    if (need_turn_rate_update)
    {
        char turn_rate_buf[32];
        snprintf(turn_rate_buf, sizeof(turn_rate_buf), "%.1f°/s", turn_indicator_state.turn_rate);
        lv_label_set_text(turn_indicator_state.turn_rate_label, turn_rate_buf);
        turn_indicator_state.last_displayed_turn_rate = turn_indicator_state.turn_rate;
    }

    // Update gravity angle display
    if (need_gravity_angle_update)
    {
        char gravity_angle_buf[32];
        snprintf(gravity_angle_buf, sizeof(gravity_angle_buf), "%.1f°", turn_indicator_state.gravity_angle);
        lv_label_set_text(turn_indicator_state.gravity_angle_label, gravity_angle_buf);
        turn_indicator_state.last_displayed_gravity_angle = turn_indicator_state.gravity_angle;
    }
}

lv_obj_t *turn_indicator_init(lv_obj_t *parent)
{
    ESP_LOGI(TAG, "Initializing turn indicator component");

    // Get IMU queue handle
    turn_indicator_state.imu_queue = get_imu_queue();
    ESP_LOGI(TAG, "IMU queue handle: %p", turn_indicator_state.imu_queue);
    if (!turn_indicator_state.imu_queue)
    {
        ESP_LOGW(TAG, "IMU queue not available yet, will try to get it later");
    }

    // Create main container
    lv_obj_t *page = lv_obj_create(parent);
    lv_obj_set_size(page, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(page, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(page, 0, 0);
    lv_obj_set_style_pad_all(page, 0, 0);

    ESP_LOGI(TAG, "Created main container");

    // Create container for side-by-side layout
    lv_obj_t *content_container = lv_obj_create(page);
    lv_obj_set_size(content_container, lv_pct(90), lv_pct(60));
    lv_obj_set_style_bg_color(content_container, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(content_container, 0, 0);
    lv_obj_set_style_pad_all(content_container, 10, 0);
    lv_obj_center(content_container);

    // Set flex layout for side-by-side arrangement
    lv_obj_set_flex_flow(content_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(content_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Create turn rate section (left side)
    lv_obj_t *turn_rate_container = lv_obj_create(content_container);
    lv_obj_set_size(turn_rate_container, lv_pct(45), lv_pct(100));
    lv_obj_set_style_bg_color(turn_rate_container, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(turn_rate_container, 0, 0);
    lv_obj_set_style_pad_all(turn_rate_container, 5, 0);

    // Set flex layout for vertical arrangement
    lv_obj_set_flex_flow(turn_rate_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(turn_rate_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Turn rate title
    lv_obj_t *turn_rate_title = lv_label_create(turn_rate_container);
    lv_label_set_text(turn_rate_title, "Turn Rate");
    lv_obj_set_style_text_color(turn_rate_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(turn_rate_title, &lv_font_montserrat_24, 0);

    // Turn rate value label
    turn_indicator_state.turn_rate_label = lv_label_create(turn_rate_container);
    lv_label_set_text(turn_indicator_state.turn_rate_label, "0.0°/s");
    lv_obj_set_style_text_color(turn_indicator_state.turn_rate_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(turn_indicator_state.turn_rate_label, &lv_font_montserrat_34, 0);

    ESP_LOGI(TAG, "Created turn rate label: %p", turn_indicator_state.turn_rate_label);

    // Create gravity angle section (right side)
    lv_obj_t *gravity_angle_container = lv_obj_create(content_container);
    lv_obj_set_size(gravity_angle_container, lv_pct(45), lv_pct(100));
    lv_obj_set_style_bg_color(gravity_angle_container, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(gravity_angle_container, 0, 0);
    lv_obj_set_style_pad_all(gravity_angle_container, 5, 0);

    // Set flex layout for vertical arrangement
    lv_obj_set_flex_flow(gravity_angle_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(gravity_angle_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Gravity angle title
    lv_obj_t *gravity_angle_title = lv_label_create(gravity_angle_container);
    lv_label_set_text(gravity_angle_title, "Gravity Angle");
    lv_obj_set_style_text_color(gravity_angle_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(gravity_angle_title, &lv_font_montserrat_24, 0);

    // Gravity angle value label
    turn_indicator_state.gravity_angle_label = lv_label_create(gravity_angle_container);
    lv_label_set_text(turn_indicator_state.gravity_angle_label, "0.0°");
    lv_obj_set_style_text_color(turn_indicator_state.gravity_angle_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(turn_indicator_state.gravity_angle_label, &lv_font_montserrat_34, 0);

    ESP_LOGI(TAG, "Created gravity angle label: %p", turn_indicator_state.gravity_angle_label);

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
    return page;
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
    }
}
