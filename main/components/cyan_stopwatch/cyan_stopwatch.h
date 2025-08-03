#ifndef CYAN_STOPWATCH_H
#define CYAN_STOPWATCH_H

#include "lvgl.h"
#include <stdbool.h>

/**
 * @brief Initialize the cyan stopwatch page
 * 
 * @param parent The parent object in which to create the stopwatch
 * @return lv_obj_t* The created page object
 */
lv_obj_t* cyan_stopwatch_init(lv_obj_t *parent);

/**
 * @brief Set the visibility state of the cyan stopwatch
 * This controls whether UI updates are performed
 * 
 * @param visible True to enable UI updates, false to skip them
 */
void cyan_stopwatch_set_visible(bool visible);

#endif // CYAN_STOPWATCH_H
