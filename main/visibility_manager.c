#include "visibility_manager.h"
#include "esp_log.h"

static const char *TAG = "VisibilityManager";

// Visibility state for each tile
static struct
{
    bool tile_visible[TILE_COUNT];
    tile_index_t active_tile;
} visibility_state = {
    .tile_visible = {true, false, false}, // Start with first tile visible
    .active_tile = TILE_CYAN_STOPWATCH};

void visibility_manager_init(void)
{
    ESP_LOGI(TAG, "Visibility manager initialized");

    // Initialize all tiles as not visible except the first one
    for (int i = 0; i < TILE_COUNT; i++)
    {
        visibility_state.tile_visible[i] = (i == TILE_CYAN_STOPWATCH);
    }
    visibility_state.active_tile = TILE_CYAN_STOPWATCH;
}

void visibility_manager_set_tile_visible(tile_index_t tile_index, bool visible)
{
    if (tile_index >= TILE_COUNT)
    {
        ESP_LOGW(TAG, "Invalid tile index: %d", tile_index);
        return;
    }

    bool previous_state = visibility_state.tile_visible[tile_index];
    visibility_state.tile_visible[tile_index] = visible;

    if (visible)
    {
        // Set all other tiles as not visible when one becomes visible
        for (int i = 0; i < TILE_COUNT; i++)
        {
            if (i != tile_index)
            {
                visibility_state.tile_visible[i] = false;
            }
        }
        visibility_state.active_tile = tile_index;
    }

    if (previous_state != visible)
    {
        const char *tile_names[] = {"Cyan Stopwatch", "Yellow Stopwatch", "G-meter", "Turn Indicator"};
        ESP_LOGI(TAG, "Tile '%s' visibility changed: %s",
                 tile_names[tile_index], visible ? "visible" : "hidden");
    }
}

bool visibility_manager_is_tile_visible(tile_index_t tile_index)
{
    if (tile_index >= TILE_COUNT)
    {
        ESP_LOGW(TAG, "Invalid tile index: %d", tile_index);
        return false;
    }

    return visibility_state.tile_visible[tile_index];
}

tile_index_t visibility_manager_get_active_tile(void)
{
    return visibility_state.active_tile;
}
