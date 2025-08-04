#ifndef TURN_INDICATOR_H
#define TURN_INDICATOR_H

#include "lvgl.h"
#include <stdbool.h>

/**
 * @brief Initialize the turn indicator page
 * 
 * @param parent The parent object in which to create the turn indicator
 * @return lv_obj_t* The created page object
 */
lv_obj_t* turn_indicator_init(lv_obj_t *parent);

/**
 * @brief Set the visibility state of the turn indicator
 * This controls whether UI updates are performed
 * 
 * @param visible True to enable UI updates, false to skip them
 */
void turn_indicator_set_visible(bool visible);

#endif // TURN_INDICATOR_H
