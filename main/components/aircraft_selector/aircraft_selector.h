#ifndef AIRCRAFT_SELECTOR_H
#define AIRCRAFT_SELECTOR_H

#include "lvgl.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Aircraft selection callback function type
 * 
 * @param aircraft_id Selected aircraft identifier ("EC-HH4" or "AMMH4")
 */
typedef void (*aircraft_selection_cb_t)(const char *aircraft_id);

/**
 * @brief Initialize and show aircraft selection screen
 * 
 * @param parent Parent object to create the selection screen in
 * @param selection_cb Callback function called when user makes selection
 * @return lv_obj_t* Pointer to the created selection screen object
 */
lv_obj_t *aircraft_selector_init(lv_obj_t *parent, aircraft_selection_cb_t selection_cb);

/**
 * @brief Clean up and remove the aircraft selection screen
 * 
 * @param selector Pointer to the selection screen object
 */
void aircraft_selector_cleanup(lv_obj_t *selector);

#ifdef __cplusplus
}
#endif

#endif // AIRCRAFT_SELECTOR_H
