#include "turn_indicator.h"
#include "main.h"
#include "imu.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "TURN_INDICATOR";

// Update rate: 60ms as requested
#define UPDATE_INTERVAL_MS 250 // 60

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
} turn_indicator_state = {
    .turn_rate = 0.0f,
    .gravity_angle = 0.0f,
    .turn_time_180 = 0.0f,
    .last_displayed_turn_rate = 999.0f,     // Initialize to different value to force first update
    .last_displayed_gravity_angle = 999.0f, // Initialize to different value to force first update
    .last_displayed_turn_time_180 = 999.0f, // Initialize to different value to force first update
    .is_visible = false,
    .update_timer = NULL,
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
 * @brief Calculate time required to complete a 180° turn at current turn rate
 * Returns time in seconds, or 0 if turn rate is too low
 */
static float calculate_180_turn_time(float turn_rate_deg_per_sec)
{
    // If turn rate is too low (less than 0.1°/s), return 0 to indicate "infinity"
    if (fabsf(turn_rate_deg_per_sec) < 0.1f)
    {
        return 0.0f;
    }

    // Calculate time = angle / rate = 180° / (deg/s) = seconds
    return 180.0f / fabsf(turn_rate_deg_per_sec);
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

        // Calculate 180° turn time
        turn_indicator_state.turn_time_180 = calculate_180_turn_time(turn_indicator_state.turn_rate);
    }

    // TODO: Update canvas drawing based on turn_rate and gravity_angle
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

    // Load image turn_coord_img
    lv_obj_t *aircraft_img = lv_img_create(parent);
    lv_img_set_src(aircraft_img, &aircraft_turn_img);
    lv_obj_set_size(aircraft_img, LV_PCT(100), LV_PCT(100));
    lv_obj_align(aircraft_img, LV_ALIGN_CENTER, 0, 0);

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
}
