#ifndef ARTIFICIAL_HORIZON_H
#define ARTIFICIAL_HORIZON_H

#include "qmi8658.h"
#include "lvgl.h"

void artificial_horizon_init(lv_obj_t *parent, qmi8658_dev_t *imu_dev);
void artificial_horizon_deinit(void);
void artificial_horizon_set_visible(bool visible);

#endif // ARTIFICIAL_HORIZON_H