#include "artificial_horizon.h"
#include "esp_log.h"
#include "esp_timer.h" // For precise time measurement in EKF task
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "Artificial Horizon";

// Display and Drawing Parameters
#define DISPLAY_REFRESH_PERIOD_MS 66     // ~30 FPS refresh rate for display
#define CANVAS_SIZE 120                  // Size of the square canvas (will draw a circle within it)
#define HORIZON_RADIUS (CANVAS_SIZE / 2) // Radius of the artificial horizon circle
#define CENTER_X (CANVAS_SIZE / 2)
#define CENTER_Y (CANVAS_SIZE / 2)

#define SKY_COLOR lv_color_hex(0x87CEEB)    // Light blue for sky
#define GROUND_COLOR lv_color_hex(0x8B4513) // Saddle brown for ground
#define LINE_COLOR lv_color_hex(0xFFFFFF)   // White for reference lines
#define REFERENCE_LINE_WIDTH 40             // Length of the main horizontal reference line

// EKF state and parameters
typedef struct
{
    float pitch;   // Pitch angle in radians
    float roll;    // Roll angle in radians
    float bias_gx; // Gyro bias X
    float bias_gy; // Gyro bias Y
    float P[4][4]; // Error covariance matrix
} ekf_state_t;

// Structure to hold estimated angles from EKF task
typedef struct
{
    float pitch;
    float roll;
} estimated_angles_t;

static struct
{
    lv_obj_t *canvas;
    lv_timer_t *display_update_timer;
    qmi8658_dev_t *imu_dev;
    ekf_state_t ekf;
    lv_color_t *canvas_buf;
    QueueHandle_t angle_queue;        // Queue to send angles from EKF task to LVGL task
    TaskHandle_t imu_ekf_task_handle; // Handle for the IMU/EKF task
    estimated_angles_t last_angles;   // Store last known angles
} ah_state = {0};

// EKF parameters (tuned for common MPU/IMU, may need further fine-tuning)
// Q_angle and Q_bias represent the uncertainty in the process model
static const float Q_angle = 0.005f; // Increased slightly for more responsiveness, might need tuning
static const float Q_bias = 0.0005f; // Reduced bias noise, less drift
// R_measure represents the uncertainty in the accelerometer measurements
static const float R_measure = 0.05f; // Increased slightly as accelerometers can be noisy

// Initialize EKF
static void init_ekf(ekf_state_t *ekf)
{
    ekf->pitch = 0.0f;
    ekf->roll = 0.0f;
    ekf->bias_gx = 0.0f;
    ekf->bias_gy = 0.0f;

    // Initialize P matrix (error covariance)
    // High initial values reflect high uncertainty
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            ekf->P[i][j] = (i == j) ? 100.0f : 0.0f; // Increased initial covariance
        }
    }
}

// EKF prediction step
static void ekf_predict(ekf_state_t *ekf, float gx, float gy, float dt)
{
    // State prediction: angle_new = angle_old + (gyro - bias) * dt
    ekf->pitch += (gx - ekf->bias_gx) * dt;
    ekf->roll += (gy - ekf->bias_gy) * dt;

    // Jacobian of state transition F
    // F = [1 0 -dt 0]
    //   [0 1 0 -dt]
    //   [0 0 1 0]
    //   [0 0 0 1]
    float F[4][4] = {
        {1, 0, -dt, 0},
        {0, 1, 0, -dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};

    // Process noise covariance matrix Q
    // Q represents the uncertainty added to the state by the process model
    float Q[4][4] = {
        {Q_angle * dt, 0, 0, 0}, // Q_angle scaled by dt for consistency
        {0, Q_angle * dt, 0, 0},
        {0, 0, Q_bias * dt, 0}, // Q_bias scaled by dt
        {0, 0, 0, Q_bias * dt}};

    // Update P: P = F*P*F' + Q
    float P_temp[4][4]; // For F*P
    float P_new[4][4];  // For (F*P)*F' + Q

    // P_temp = F * P
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P_temp[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                P_temp[i][j] += F[i][k] * ekf->P[k][j];
            }
        }
    }

    // P_new = P_temp * F' + Q
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P_new[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                P_new[i][j] += P_temp[i][k] * F[j][k]; // F[j][k] is F_transpose[k][j]
            }
            ekf->P[i][j] = P_new[i][j] + Q[i][j];
        }
    }
}

// EKF update step
static void ekf_update(ekf_state_t *ekf, float ax, float ay, float az)
{
    // Calculate pitch and roll from accelerometer (measurement model h(x))
    // Rotated 90 degrees clockwise:
    // - Roll from Z axis, with 90-degree offset (subtract π/2 from the angle)
    // - Pitch from Y axis
    float roll_m = atan2f(az, ax) - (M_PI / 2.0f); // Roll from accel (Z axis), shifted by 90°
    float pitch_m = atan2f(-ay, -az);              // Pitch from accel (Y axis)

    // Wrap roll angle to [-π, π]
    if (roll_m > M_PI)
        roll_m -= 2.0f * M_PI;
    if (roll_m < -M_PI)
        roll_m += 2.0f * M_PI;

    // Measurement vector z
    float z[2] = {pitch_m, roll_m};

    // Innovation (measurement residual) y = z - h(x_predicted)
    float y[2] = {z[0] - ekf->pitch, z[1] - ekf->roll};

    // Measurement Jacobian H
    // H = [d(pitch_m)/dpitch d(pitch_m)/droll d(pitch_m)/dbias_gx d(pitch_m)/dbias_gy]
    //   [d(roll_m)/dpitch  d(roll_m)/droll  d(roll_m)/dbias_gx  d(roll_m)/dbias_gy]
    // Since pitch_m and roll_m only depend on accelerometer data, not state (pitch, roll, biases),
    // H is simply the identity matrix for the angle components.
    float H[2][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0}};

    // Measurement noise covariance matrix R
    // R represents the uncertainty in the sensor measurements
    float R[2][2] = {
        {R_measure, 0},
        {0, R_measure}};

    // Calculate S = H*P*H' + R (Innovation covariance)
    float HP[2][4]; // For H*P
    // H*P
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            HP[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                HP[i][j] += H[i][k] * ekf->P[k][j];
            }
        }
    }

    float S[2][2]; // For HP*H' + R
    // HP*H' + R
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            S[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                S[i][j] += HP[i][k] * H[j][k]; // H[j][k] is H_transpose[k][j]
            }
            S[i][j] += R[i][j];
        }
    }

    // Calculate K = P*H'*S^-1 (Kalman Gain)
    float det_S = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabs(det_S) < 1e-9) // Prevent division by zero or very small numbers
    {
        ESP_LOGW(TAG, "Singular S matrix in EKF update. Skipping update.");
        return;
    }

    float S_inv[2][2] = {
        {S[1][1] / det_S, -S[0][1] / det_S},
        {-S[1][0] / det_S, S[0][0] / det_S}};

    float PHt[4][2]; // For P*H'
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            PHt[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                PHt[i][j] += ekf->P[i][k] * H[j][k]; // H[j][k] is H_transpose[k][j]
            }
        }
    }

    float K[4][2]; // Kalman Gain K = PHt * S_inv
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            K[i][j] = 0;
            for (int l = 0; l < 2; l++)
            {
                K[i][j] += PHt[i][l] * S_inv[l][j];
            }
        }
    }

    // Update state: x = x_predicted + K*y
    ekf->pitch += K[0][0] * y[0] + K[0][1] * y[1];
    ekf->roll += K[1][0] * y[0] + K[1][1] * y[1];
    ekf->bias_gx += K[2][0] * y[0] + K[2][1] * y[1];
    ekf->bias_gy += K[3][0] * y[0] + K[3][1] * y[1];

    // Update P matrix: P = (I - K*H) * P_predicted
    float I_KH[4][4];
    float KH[4][4]; // K*H
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            KH[i][j] = 0;
            for (int l = 0; l < 2; l++)
            {
                KH[i][j] += K[i][l] * H[l][j];
            }
            I_KH[i][j] = (i == j) - KH[i][j];
        }
    }

    float P_temp[4][4]; // For (I - K*H) * P_predicted
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P_temp[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                P_temp[i][j] += I_KH[i][k] * ekf->P[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            ekf->P[i][j] = P_temp[i][j];
        }
    }
}

// IMU reading and EKF task
static void imu_ekf_task(void *arg)
{
    qmi8658_dev_t *imu_dev = (qmi8658_dev_t *)arg;
    init_ekf(&ah_state.ekf); // Initialize EKF state in this task context

    uint64_t last_time_us = esp_timer_get_time();
    qmi8658_data_t data;
    bool ready;

    while (1)
    {
        uint64_t current_time_us = esp_timer_get_time();
        float dt_actual = (current_time_us - last_time_us) / 1000000.0f;
        last_time_us = current_time_us;

        // Read IMU data
        if (qmi8658_is_data_ready(imu_dev, &ready) == ESP_OK && ready)
        {
            if (qmi8658_read_sensor_data(imu_dev, &data) == ESP_OK)
            {
                // For 90-degree clockwise rotation:
                // - Roll still uses gyroZ but needs to be negated due to the rotation
                // - Pitch still uses gyroY
                ekf_predict(&ah_state.ekf, -data.gyroZ, data.gyroY, dt_actual);
                ekf_update(&ah_state.ekf, data.accelX, data.accelY, data.accelZ);

                // Send updated angles to LVGL task
                estimated_angles_t angles = {
                    .pitch = ah_state.ekf.pitch,
                    .roll = ah_state.ekf.roll};
                // Don't block if queue is full, just drop the old data, LVGL will get the newest next time.
                xQueueOverwrite(ah_state.angle_queue, &angles);

                // ESP_LOGI(TAG, "Pitch: %.2f deg, Roll: %.2f deg",
                //         RT_RAD_TO_DEG(ah_state.ekf.pitch), RT_RAD_TO_DEG(ah_state.ekf.roll));
            }
        }
        // Adjust this delay to match your desired IMU read rate.
        // For 100Hz, delay 10ms. For 200Hz, delay 5ms.
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Draw horizon using LVGL 9.3.0 compatible approach
static void draw_horizon(estimated_angles_t angles)
{
    if (!ah_state.canvas)
        return;

    // Clear canvas with black background
    lv_canvas_fill_bg(ah_state.canvas, lv_color_hex(0x000000), LV_OPA_COVER);

    float roll_rad = angles.roll;
    float pitch_rad = angles.pitch;

    // Scale pitch to pixels based on HORIZON_RADIUS
    float pitch_offset_y = pitch_rad * (CANVAS_SIZE / M_PI);

    // Calculate horizon line parameters
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);

    // Draw the horizon by checking each pixel
    for (int x = 0; x < CANVAS_SIZE; x++)
    {
        for (int y = 0; y < CANVAS_SIZE; y++)
        {
            // Check if pixel is within the circular display area
            int dx = x - CENTER_X;
            int dy = y - CENTER_Y;
            float distance = sqrtf(dx * dx + dy * dy);

            if (distance <= HORIZON_RADIUS)
            {
                // Transform pixel coordinates relative to horizon
                float x_rel = dx;
                float y_rel = dy;

                // Rotate coordinates by negative roll angle and apply pitch offset
                float y_rot = -x_rel * sin_roll + y_rel * cos_roll - pitch_offset_y;

                // Determine if this pixel is sky or ground
                lv_color_t pixel_color;
                if (y_rot < 0)
                {
                    pixel_color = SKY_COLOR;
                }
                else
                {
                    pixel_color = GROUND_COLOR;
                }

                lv_canvas_set_px(ah_state.canvas, x, y, pixel_color, LV_OPA_COVER);
            }
        }
    }

    // Draw reference lines using LVGL 9.3.0 canvas API
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = LINE_COLOR;
    line_dsc.width = 2;

    // Initialize layer for drawing
    lv_layer_t layer;
    lv_canvas_init_layer(ah_state.canvas, &layer);

    // Draw horizontal reference line
    line_dsc.p1.x = CENTER_X - REFERENCE_LINE_WIDTH / 2;
    line_dsc.p1.y = CENTER_Y;
    line_dsc.p2.x = CENTER_X + REFERENCE_LINE_WIDTH / 2;
    line_dsc.p2.y = CENTER_Y;
    lv_draw_line(&layer, &line_dsc);

    // Draw vertical reference line
    line_dsc.p1.x = CENTER_X;
    line_dsc.p1.y = CENTER_Y - REFERENCE_LINE_WIDTH / 2;
    line_dsc.p2.x = CENTER_X;
    line_dsc.p2.y = CENTER_Y + REFERENCE_LINE_WIDTH / 2;
    lv_draw_line(&layer, &line_dsc);

    // Finish the layer to apply the changes
    lv_canvas_finish_layer(ah_state.canvas, &layer);

    // Invalidate the canvas to trigger redraw
    lv_obj_invalidate(ah_state.canvas);
}

// Timer callback to update artificial horizon display
static void display_update_timer_cb(lv_timer_t *timer)
{
    estimated_angles_t angles;
    // Try to get the latest angles from the queue
    if (xQueueReceive(ah_state.angle_queue, &angles, 0) == pdPASS)
    {
        ah_state.last_angles = angles; // Store the latest angles
        draw_horizon(angles);
    }
    else
    {
        // No new data, redraw with last known angles for smooth display
        draw_horizon(ah_state.last_angles);
    }
}

lv_obj_t *artificial_horizon_init(lv_obj_t *parent, qmi8658_dev_t *imu_dev)
{
    // Create main container
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(container, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);

    // Store IMU device handle
    ah_state.imu_dev = imu_dev;

    // Initialize last known angles to zero
    ah_state.last_angles.pitch = 0.0f;
    ah_state.last_angles.roll = 0.0f;

    // Create FreeRTOS queue for EKF results
    ah_state.angle_queue = xQueueCreate(1, sizeof(estimated_angles_t)); // Queue of size 1 (only latest value matters)
    if (ah_state.angle_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create angle queue");
        lv_obj_del(container);
        return NULL;
    }

    // Create canvas for artificial horizon
    // Allocate buffer for the canvas: CANVAS_SIZE x CANVAS_SIZE
    ah_state.canvas_buf = (lv_color_t *)malloc(CANVAS_SIZE * CANVAS_SIZE * sizeof(lv_color_t));
    if (!ah_state.canvas_buf)
    {
        ESP_LOGE(TAG, "Failed to allocate canvas buffer");
        vQueueDelete(ah_state.angle_queue);
        lv_obj_del(container);
        return NULL;
    }

    ah_state.canvas = lv_canvas_create(container);
    lv_canvas_set_buffer(ah_state.canvas, ah_state.canvas_buf, CANVAS_SIZE, CANVAS_SIZE, LV_COLOR_FORMAT_NATIVE);
    lv_obj_align(ah_state.canvas, LV_ALIGN_CENTER, 0, 0);
    // Initial fill (will be overwritten by draw_horizon)
    lv_canvas_fill_bg(ah_state.canvas, lv_color_hex(0x000000), LV_OPA_COVER);

    // Create the IMU/EKF FreeRTOS task
    BaseType_t xReturned = xTaskCreate(
        imu_ekf_task,                 // Task function
        "IMU_EKF_Task",               // Task name
        4096,                         // Stack size (increase if needed)
        (void *)imu_dev,              // Parameter to pass
        5,                            // Priority (higher than LVGL task normally)
        &ah_state.imu_ekf_task_handle // Task handle
    );
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create IMU_EKF_Task");
        free(ah_state.canvas_buf);
        vQueueDelete(ah_state.angle_queue);
        lv_obj_del(container);
        return NULL;
    }

    // Create display update timer for LVGL
    ah_state.display_update_timer = lv_timer_create(display_update_timer_cb, DISPLAY_REFRESH_PERIOD_MS, NULL);
    if (ah_state.display_update_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create display update timer");
        vTaskDelete(ah_state.imu_ekf_task_handle);
        free(ah_state.canvas_buf);
        vQueueDelete(ah_state.angle_queue);
        lv_obj_del(container);
        return NULL;
    }

    return container;
}

// Function to de-initialize the artificial horizon if needed
void artificial_horizon_deinit(void)
{
    if (ah_state.display_update_timer)
    {
        lv_timer_del(ah_state.display_update_timer);
        ah_state.display_update_timer = NULL;
    }
    if (ah_state.imu_ekf_task_handle)
    {
        vTaskDelete(ah_state.imu_ekf_task_handle);
        ah_state.imu_ekf_task_handle = NULL;
    }
    if (ah_state.canvas_buf)
    {
        free(ah_state.canvas_buf);
        ah_state.canvas_buf = NULL;
    }
    if (ah_state.angle_queue)
    {
        vQueueDelete(ah_state.angle_queue);
        ah_state.angle_queue = NULL;
    }
    // LVGL objects are typically deleted by their parent or when the screen changes.
    // If `container` is the only child of a screen, deleting the screen will delete it.
    // If it's part of a tileview, it will be cleaned up when the tileview is destroyed.
    // Avoid `lv_obj_del(ah_state.canvas)` here unless you manage its parentage carefully.
}