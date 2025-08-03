#include "aircraft_header.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "AircraftHeader";

// Structure to hold header data
typedef struct
{
    lv_obj_t *label;
} aircraft_header_data_t;

lv_obj_t *aircraft_header_init(lv_obj_t *parent, const char *aircraft_id)
{
    // Allocate data structure
    aircraft_header_data_t *data = malloc(sizeof(aircraft_header_data_t));
    if (!data)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for header data");
        return NULL;
    }

    // Create header container with white background
    lv_obj_t *header_container = lv_obj_create(parent);
    lv_obj_set_size(header_container, LV_PCT(100), 50);      // 100% width, 50px height
    lv_obj_align(header_container, LV_ALIGN_TOP_MID, 0, 10); // 10px from top
    lv_obj_set_style_bg_color(header_container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(header_container, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(header_container, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(header_container, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_radius(header_container, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_all(header_container, 5, LV_PART_MAIN);
    lv_obj_clear_flag(header_container, LV_OBJ_FLAG_SCROLLABLE);

    // Create label with black text
    lv_obj_t *label = lv_label_create(header_container);
    lv_label_set_text(label, aircraft_id ? aircraft_id : "UNKNOWN");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_38, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0x000000), 0); // Black text
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);

    data->label = label;

    // Store data in container for cleanup
    lv_obj_set_user_data(header_container, data);

    ESP_LOGI(TAG, "Aircraft header initialized with: %s", aircraft_id ? aircraft_id : "UNKNOWN");

    return header_container;
}

void aircraft_header_set_text(lv_obj_t *header, const char *aircraft_id)
{
    if (!header || !aircraft_id)
        return;

    aircraft_header_data_t *data = (aircraft_header_data_t *)lv_obj_get_user_data(header);
    if (!data || !data->label)
        return;

    lv_label_set_text(data->label, aircraft_id);

    ESP_LOGI(TAG, "Aircraft header updated to: %s", aircraft_id);
}

void aircraft_header_cleanup(lv_obj_t *header)
{
    if (!header)
        return;

    // Free allocated data
    aircraft_header_data_t *data = (aircraft_header_data_t *)lv_obj_get_user_data(header);
    if (data)
    {
        free(data);
    }

    // Delete the header object
    lv_obj_del(header);

    ESP_LOGI(TAG, "Aircraft header cleaned up");
}
