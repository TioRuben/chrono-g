#include "artificial_horizon.h"
#include "esp_log.h"
#include "qmi8658.h"
#include "lvgl.h"

const char *TAG = "ArtificialHorizon";

void artificial_horizon_init(lv_obj_t *parent, qmi8658_dev_t *imu_dev)
{
    ESP_LOGI(TAG, "Initializing artificial horizon");
}

// Function to de-initialize the artificial horizon if needed
void artificial_horizon_deinit(void)
{
}

void artificial_horizon_set_visible(bool visible)
{
    ESP_LOGI(TAG, "Setting artificial horizon visibility to %s", visible ? "true" : "false");
    // Here you would typically show/hide the LVGL object representing the horizon
    // For example: lv_obj_set_hidden(horizon_obj, !visible);
}