#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the IMU data queue handle
 * 
 * This function provides access to the global IMU data queue that contains
 * the latest IMU readings from the sensor task.
 * 
 * @return QueueHandle_t Handle to the IMU queue, or NULL if not initialized
 */
QueueHandle_t get_imu_queue(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
