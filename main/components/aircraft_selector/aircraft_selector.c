#include "aircraft_selector.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "AircraftSelector";

// Structure to hold selector data
typedef struct
{
    aircraft_selection_cb_t selection_cb;
    lv_obj_t *container;
} aircraft_selector_data_t;

/**
 * @brief Event handler for aircraft selection buttons
 */
static void aircraft_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code != LV_EVENT_CLICKED)
        return;

    lv_obj_t *btn = lv_event_get_target(e);
    aircraft_selector_data_t *data = (aircraft_selector_data_t *)lv_event_get_user_data(e);

    if (!data || !data->selection_cb)
        return;

    // Get the aircraft ID from button user data
    const char *aircraft_id = (const char *)lv_obj_get_user_data(btn);

    ESP_LOGI(TAG, "Aircraft selected: %s", aircraft_id);

    // Call the selection callback
    data->selection_cb(aircraft_id);
}

lv_obj_t *aircraft_selector_init(lv_obj_t *parent, aircraft_selection_cb_t selection_cb)
{
    // Allocate data structure
    aircraft_selector_data_t *data = malloc(sizeof(aircraft_selector_data_t));
    if (!data)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for selector data");
        return NULL;
    }

    data->selection_cb = selection_cb;

    // Create main container that fills the screen
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(container, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);

    data->container = container;

    // Store data in container for cleanup
    lv_obj_set_user_data(container, data);

    // Create title label
    lv_obj_t *title_label = lv_label_create(container);
    lv_label_set_text(title_label, "Select Callsign");
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_38, 0);
    lv_obj_set_style_text_color(title_label, lv_color_white(), 0);
    lv_obj_align(title_label, LV_ALIGN_CENTER, 0, 0);

    // Create EC-HH4 button (top half)
    lv_obj_t *ec_btn = lv_btn_create(container);
    lv_obj_set_size(ec_btn, LV_PCT(100), LV_PCT(40));
    lv_obj_align(ec_btn, LV_ALIGN_CENTER, 0, LV_PCT(-30));
    lv_obj_set_style_radius(ec_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ec_btn, 0, LV_PART_MAIN);

    // EC-HH4 button label
    lv_obj_t *ec_label = lv_label_create(ec_btn);
    lv_label_set_text(ec_label, "EC-HH4");
    lv_obj_set_style_text_font(ec_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(ec_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(ec_label);

    // Store aircraft ID in button user data
    lv_obj_set_user_data(ec_btn, (void *)"EC-HH4");
    lv_obj_add_event_cb(ec_btn, aircraft_btn_event_cb, LV_EVENT_CLICKED, data);

    // Create AMMH4 button (bottom half)
    lv_obj_t *ammh4_btn = lv_btn_create(container);
    lv_obj_set_size(ammh4_btn, LV_PCT(100), LV_PCT(40));
    lv_obj_align(ammh4_btn, LV_ALIGN_CENTER, 0, LV_PCT(30));
    lv_obj_set_style_radius(ammh4_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ammh4_btn, 0, LV_PART_MAIN);

    // AMMH4 button label
    lv_obj_t *ammh4_label = lv_label_create(ammh4_btn);
    lv_label_set_text(ammh4_label, "AMMH4");
    lv_obj_set_style_text_font(ammh4_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(ammh4_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(ammh4_label);

    // Store aircraft ID in button user data
    lv_obj_set_user_data(ammh4_btn, (void *)"AMMH4");
    lv_obj_add_event_cb(ammh4_btn, aircraft_btn_event_cb, LV_EVENT_CLICKED, data);

    ESP_LOGI(TAG, "Aircraft selector initialized");

    return container;
}

void aircraft_selector_cleanup(lv_obj_t *selector)
{
    if (!selector)
        return;

    // Free allocated data
    aircraft_selector_data_t *data = (aircraft_selector_data_t *)lv_obj_get_user_data(selector);
    if (data)
    {
        free(data);
    }

    // Delete the selector object
    lv_obj_del(selector);

    ESP_LOGI(TAG, "Aircraft selector cleaned up");
}
