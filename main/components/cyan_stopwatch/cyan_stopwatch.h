#ifndef CYAN_STOPWATCH_H
#define CYAN_STOPWATCH_H

#include "lvgl.h"

/**
 * @brief Initialize the cyan stopwatch page
 * 
 * @param parent The parent object in which to create the stopwatch
 * @return lv_obj_t* The created page object
 */
lv_obj_t* cyan_stopwatch_init(lv_obj_t *parent);

#endif // CYAN_STOPWATCH_H
