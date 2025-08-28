#ifndef MAGENTA_STOPWATCH_H
#define MAGENTA_STOPWATCH_H

#include "lvgl.h"
#include <stdbool.h>

/**
 * @brief Initialize the magenta stopwatch page
 *
 * @param parent The parent object in which to create the stopwatch
 * @return lv_obj_t* The created page object
 */
lv_obj_t *magenta_stopwatch_init(lv_obj_t *parent);

/**
 * @brief Set the visibility state of the magenta stopwatch
 * This controls whether UI updates are performed
 *
 * @param visible True to enable UI updates, false to skip them
 */
void magenta_stopwatch_set_visible(bool visible);

#endif // MAGENTA_STOPWATCH_H
