#ifndef G_METER_H
#define G_METER_H

#include "lvgl.h"
#include <stdbool.h>

/**
 * @brief Initialize the G-meter page
 * 
 * @param parent The parent object in which to create the G-meter
 * @return lv_obj_t* The created page object
 */
lv_obj_t* g_meter_init(lv_obj_t *parent);

/**
 * @brief Set the visibility state of the G-meter
 * This controls whether UI updates are performed
 * 
 * @param visible True to enable UI updates, false to skip them
 */
void g_meter_set_visible(bool visible);

#endif // G_METER_H
