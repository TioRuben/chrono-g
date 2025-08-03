#ifndef VISIBILITY_MANAGER_H
#define VISIBILITY_MANAGER_H

#include <stdbool.h>

/**
 * @brief Tile indices for the tileview
 */
typedef enum {
    TILE_CYAN_STOPWATCH = 0,
    TILE_YELLOW_STOPWATCH = 1,
    TILE_ARTIFICIAL_HORIZON = 2,
    TILE_COUNT = 3
} tile_index_t;

/**
 * @brief Initialize the visibility manager
 */
void visibility_manager_init(void);

/**
 * @brief Set the visibility state of a specific tile
 * 
 * @param tile_index The tile index to update
 * @param visible True if the tile is visible, false otherwise
 */
void visibility_manager_set_tile_visible(tile_index_t tile_index, bool visible);

/**
 * @brief Check if a specific tile is currently visible
 * 
 * @param tile_index The tile index to check
 * @return true if the tile is visible, false otherwise
 */
bool visibility_manager_is_tile_visible(tile_index_t tile_index);

/**
 * @brief Get the currently active tile index
 * 
 * @return tile_index_t The currently active tile
 */
tile_index_t visibility_manager_get_active_tile(void);

#endif // VISIBILITY_MANAGER_H
