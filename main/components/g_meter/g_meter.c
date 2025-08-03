#include "g_meter.h"
#include "main.h"
#include "imu.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include <math.h>
#include <stdio.h>

LV_FONT_DECLARE(lv_font_g_meter_96);

static const char *TAG = "G_METER";

// Aircraft G limits for color coding
#define G_LIMIT_MIN -1.0f
#define G_LIMIT_MAX 3.0f

// Update rate: 8 FPS = 125ms (further reduced to prevent tearing)
#define UPDATE_INTERVAL_MS 250

// Component state
static struct
{
    float current_g;
    float min_g;
    float max_g;
    float last_displayed_g;     // Track last displayed value to prevent unnecessary updates
    float last_displayed_min_g; // Track last displayed min value
    float last_displayed_max_g; // Track last displayed max value
    bool is_visible;
    lv_timer_t *update_timer;
    lv_obj_t *g_value_label;
    lv_obj_t *minmax_label;
    lv_obj_t *reset_btn;
    QueueHandle_t imu_queue;
} g_meter_state = {
    .current_g = 1.0f,
    .min_g = 1.0f,
    .max_g = 1.0f,
    .last_displayed_g = 0.0f,     // Initialize to different value to force first update
    .last_displayed_min_g = 0.0f, // Initialize to different value to force first update
    .last_displayed_max_g = 0.0f, // Initialize to different value to force first update
    .is_visible = false,
    .update_timer = NULL,
    .g_value_label = NULL,
    .minmax_label = NULL,
    .reset_btn = NULL,
    .imu_queue = NULL};

/**
 * @brief Calculate color based on G-force value
 * Simple threshold-based color coding
 */
static lv_color_t calculate_g_color(float g_value)
{
    if (g_value <= G_LIMIT_MIN || g_value >= G_LIMIT_MAX)
    {
        // Below minimum or above maximum limit - red
        return lv_color_hex(0xFF0000);
    }
    else if (g_value >= 0.6f && g_value <= 1.4f)
    {
        // Normal 1G range - white
        return lv_color_hex(0xFFFFFF);
    }
    else
    {
        // Warning range - orange
        return lv_color_hex(0xFF8000);
    }
}

/**
 * @brief Update G-meter display with latest IMU data
 */
static void update_g_meter(lv_timer_t *timer)
{
    // Skip updates when not visible
    if (!g_meter_state.is_visible)
    {
        return;
    }

    // Skip if labels not ready
    if (!g_meter_state.g_value_label || !g_meter_state.minmax_label)
    {
        return;
    }

    // Get IMU queue if not available
    if (!g_meter_state.imu_queue)
    {
        g_meter_state.imu_queue = get_imu_queue();
        if (!g_meter_state.imu_queue)
        {
            return;
        }
    }

    // Process IMU data if available
    imu_data_t imu_data;
    if (xQueueReceive(g_meter_state.imu_queue, &imu_data, 0) == pdTRUE)
    {
        float g_force = imu_calculate_g_load_factor(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        g_meter_state.current_g = g_force;

        // Update min/max values
        if (g_force < g_meter_state.min_g)
            g_meter_state.min_g = g_force;
        if (g_force > g_meter_state.max_g)
            g_meter_state.max_g = g_force;
    }

    // Check if UI updates are needed (only update if significant change)
    bool need_g_update = fabsf(g_meter_state.current_g - g_meter_state.last_displayed_g) >= 0.05f;
    bool need_minmax_update = fabsf(g_meter_state.min_g - g_meter_state.last_displayed_min_g) >= 0.05f ||
                              fabsf(g_meter_state.max_g - g_meter_state.last_displayed_max_g) >= 0.05f;

    // Update main G value display
    if (need_g_update)
    {
        char g_buf[16];
        snprintf(g_buf, sizeof(g_buf), "%+.1fG", g_meter_state.current_g);
        lv_label_set_text(g_meter_state.g_value_label, g_buf);
        lv_obj_set_style_text_color(g_meter_state.g_value_label, calculate_g_color(g_meter_state.current_g), 0);
        g_meter_state.last_displayed_g = g_meter_state.current_g;
    }

    // Update min/max display
    if (need_minmax_update)
    {
        char minmax_buf[64];
        snprintf(minmax_buf, sizeof(minmax_buf), "Min: %+.1f    Max: %+.1f", g_meter_state.min_g, g_meter_state.max_g);
        lv_label_set_text(g_meter_state.minmax_label, minmax_buf);
        g_meter_state.last_displayed_min_g = g_meter_state.min_g;
        g_meter_state.last_displayed_max_g = g_meter_state.max_g;
    }
}

/**
 * @brief Reset button event handler
 */
static void reset_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED)
    {
        ESP_LOGI(TAG, "Reset button clicked - resetting min/max values");

        // Reset min/max to current value
        g_meter_state.min_g = g_meter_state.current_g;
        g_meter_state.max_g = g_meter_state.current_g;

        // Update display immediately if visible
        if (g_meter_state.is_visible && g_meter_state.minmax_label != NULL)
        {
            char minmax_buf[64];
            snprintf(minmax_buf, sizeof(minmax_buf), "Min: %+.1f    Max: %+.1f", g_meter_state.min_g, g_meter_state.max_g);
            lv_label_set_text(g_meter_state.minmax_label, minmax_buf);
            g_meter_state.last_displayed_min_g = g_meter_state.min_g;
            g_meter_state.last_displayed_max_g = g_meter_state.max_g;

            // Force immediate refresh
            lv_obj_invalidate(g_meter_state.minmax_label);
        }
    }
}

lv_obj_t *g_meter_init(lv_obj_t *parent)
{
    ESP_LOGI(TAG, "Initializing G-meter component");

    // Get IMU queue handle
    g_meter_state.imu_queue = get_imu_queue();
    ESP_LOGI(TAG, "IMU queue handle: %p", g_meter_state.imu_queue);
    if (!g_meter_state.imu_queue)
    {
        ESP_LOGW(TAG, "IMU queue not available yet, will try to get it later");
        // Don't return NULL, continue with UI creation
    }

    // Create main container
    lv_obj_t *page = lv_obj_create(parent);
    lv_obj_set_size(page, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(page, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(page, 0, 0);
    lv_obj_set_style_pad_all(page, 0, 0);

    ESP_LOGI(TAG, "Created main container");

    // Create main G value label (centered)
    g_meter_state.g_value_label = lv_label_create(page);
    lv_label_set_text(g_meter_state.g_value_label, "+1.0G");
    lv_obj_set_style_text_font(g_meter_state.g_value_label, &lv_font_g_meter_96, 0);
    lv_obj_set_style_text_color(g_meter_state.g_value_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(g_meter_state.g_value_label);

    ESP_LOGI(TAG, "Created main G value label: %p", g_meter_state.g_value_label);

    // Create single centered min/max label (positioned higher above the reset button)
    g_meter_state.minmax_label = lv_label_create(page);
    lv_label_set_text(g_meter_state.minmax_label, "Min: +1.0    Max: +1.0");
    lv_obj_set_style_text_font(g_meter_state.minmax_label, &lv_font_montserrat_34, 0);
    lv_obj_set_style_text_color(g_meter_state.minmax_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(g_meter_state.minmax_label, LV_ALIGN_BOTTOM_MID, 0, -130);

    ESP_LOGI(TAG, "Created minmax label: %p", g_meter_state.minmax_label);

    // Create button container (same pattern as stopwatches)
    lv_obj_t *btn_container = lv_obj_create(page);
    lv_obj_set_size(btn_container, lv_pct(100), lv_pct(25));
    lv_obj_align(btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(btn_container, LV_OPA_0, 0);
    lv_obj_set_style_pad_all(btn_container, 0, 0);
    lv_obj_set_style_border_width(btn_container, 0, 0);

    // Create reset button (same height as stopwatch buttons)
    g_meter_state.reset_btn = lv_btn_create(btn_container);
    lv_obj_set_size(g_meter_state.reset_btn, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(g_meter_state.reset_btn, lv_color_hex(0x808080), LV_PART_MAIN);
    lv_obj_set_style_radius(g_meter_state.reset_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_meter_state.reset_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_meter_state.reset_btn, 0, LV_PART_MAIN);

    // Create reset button label
    lv_obj_t *reset_label = lv_label_create(g_meter_state.reset_btn);
    lv_label_set_text(reset_label, "RESET");
    lv_obj_center(reset_label);
    lv_obj_set_style_text_font(reset_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(reset_label, lv_color_hex(0xFFFFFF), 0);

    // Add reset button event handler
    lv_obj_add_event_cb(g_meter_state.reset_btn, reset_btn_event_cb, LV_EVENT_CLICKED, NULL);

    // Create update timer (30 FPS)
    g_meter_state.update_timer = lv_timer_create(update_g_meter, UPDATE_INTERVAL_MS, NULL);

    ESP_LOGI(TAG, "G-meter component initialized successfully");
    ESP_LOGI(TAG, "Final state - g_value_label: %p, minmax_label: %p",
             g_meter_state.g_value_label, g_meter_state.minmax_label);
    return page;
}

void g_meter_set_visible(bool visible)
{
    g_meter_state.is_visible = visible;

    if (visible)
    {
        // When becoming visible, update display immediately to sync with current state
        ESP_LOGI(TAG, "G-meter becoming visible - updating display");

        // Check if labels are initialized before updating
        if (g_meter_state.g_value_label == NULL ||
            g_meter_state.minmax_label == NULL)
        {
            ESP_LOGW(TAG, "G-meter labels not initialized yet, skipping update");
            return;
        }

        // Update all displays
        char g_buf[16];
        snprintf(g_buf, sizeof(g_buf), "%+.1fG", g_meter_state.current_g);
        lv_label_set_text(g_meter_state.g_value_label, g_buf);

        lv_color_t g_color = calculate_g_color(g_meter_state.current_g);
        lv_obj_set_style_text_color(g_meter_state.g_value_label, g_color, 0);

        // Update the last displayed values to current
        g_meter_state.last_displayed_g = g_meter_state.current_g;
        g_meter_state.last_displayed_min_g = g_meter_state.min_g;
        g_meter_state.last_displayed_max_g = g_meter_state.max_g;

        char minmax_buf[64];
        snprintf(minmax_buf, sizeof(minmax_buf), "Min: %+.1f    Max: %+.1f", g_meter_state.min_g, g_meter_state.max_g);
        lv_label_set_text(g_meter_state.minmax_label, minmax_buf);

        // Force immediate refresh
        lv_obj_invalidate(g_meter_state.g_value_label);
        lv_obj_invalidate(g_meter_state.minmax_label);
    }
    else
    {
        ESP_LOGI(TAG, "G-meter becoming invisible - skipping UI updates");
    }
}
