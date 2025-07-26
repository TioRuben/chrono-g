#ifndef ARTIFICIAL_HORIZON_H
#define ARTIFICIAL_HORIZON_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"    // For lv_obj_t and LVGL types
#include "qmi8658.h" // For qmi8658_dev_t and IMU data types
#include <math.h>    // For M_PI if not already defined

// Utility macro for converting radians to degrees
#ifndef RT_RAD_TO_DEG
#define RT_RAD_TO_DEG(x) ((x) * (180.0f / M_PI))
#endif

    /**
     * @brief Initializes and creates the Artificial Horizon display on an LVGL parent object.
     *
     * This function sets up the LVGL canvas for the artificial horizon, initializes the
     * Extended Kalman Filter (EKF), and starts a FreeRTOS task to read IMU data
     * and update the EKF state. It also creates an LVGL timer to periodically
     * refresh the display based on the EKF's estimated angles.
     *
     * @param parent A pointer to the LVGL parent object where the artificial horizon
     * canvas will be created. This should typically be an LVGL screen
     * or a container object.
     * @param imu_dev A pointer to the initialized QMI8658 device structure. This
     * device will be used to read accelerometer and gyroscope data.
     * @return A pointer to the created LVGL container object for the artificial horizon,
     * or NULL if initialization fails.
     */
    lv_obj_t *artificial_horizon_init(lv_obj_t *parent, qmi8658_dev_t *imu_dev);

    /**
     * @brief De-initializes and cleans up resources used by the Artificial Horizon.
     *
     * This function stops the LVGL display update timer, deletes the FreeRTOS EKF task,
     * frees allocated canvas buffers, and deletes the FreeRTOS angle queue.
     * It should be called when the artificial horizon page is no longer needed
     * to prevent memory leaks and resource conflicts.
     */
    void artificial_horizon_deinit(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // ARTIFICIAL_HORIZON_H