#ifndef AIRCRAFT_HEADER_H
#define AIRCRAFT_HEADER_H

#include "lvgl.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize aircraft header label
 * 
 * @param parent Parent object to create the header in
 * @param aircraft_id Initial aircraft identifier to display
 * @return lv_obj_t* Pointer to the created header object
 */
lv_obj_t *aircraft_header_init(lv_obj_t *parent, const char *aircraft_id);

/**
 * @brief Update the aircraft identifier displayed in the header
 * 
 * @param header Pointer to the header object
 * @param aircraft_id New aircraft identifier to display
 */
void aircraft_header_set_text(lv_obj_t *header, const char *aircraft_id);

/**
 * @brief Clean up and remove the aircraft header
 * 
 * @param header Pointer to the header object
 */
void aircraft_header_cleanup(lv_obj_t *header);

#ifdef __cplusplus
}
#endif

#endif // AIRCRAFT_HEADER_H
