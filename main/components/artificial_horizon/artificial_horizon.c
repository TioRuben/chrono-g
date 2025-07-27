#include "artificial_horizon.h"
#include "esp_log.h"
#include "esp_timer.h" // For precise time measurement
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

static const char *TAG = "Artificial Horizon";

// Display and Drawing Parameters
#define DISPLAY_REFRESH_PERIOD_MS 40 // 25 FPS refresh rate for display

// Horizon colors
#define SKY_COLOR lv_color_hex(0x87CEEB)           // Light blue for sky
#define GROUND_COLOR lv_color_hex(0x8B4513)        // Saddle brown for ground
#define LINE_COLOR lv_color_hex(0xFFFFFF)          // White for reference lines
#define AIRCRAFT_LINE_COLOR lv_color_hex(0xFFFF00) // Yellow for aircraft lines
#define REFERENCE_LINE_WIDTH 80                    // Length of the main horizontal reference line

// Horizon parameters
#define HORIZON_LINE_WIDTH 3                     // Width of the white horizon line
#define G_COLOR_NORMAL lv_color_hex(0xFFFFFF)    // White for normal G (0.5-2.0G)
#define G_COLOR_WARNING lv_color_hex(0xFFA500)   // Orange for moderate G (2.0-4.0G)
#define G_COLOR_DANGER lv_color_hex(0xFF0000)    // Red for extreme G (>4.0G or <0.5G)
#define G_BUTTON_BG_COLOR lv_color_hex(0x0066CC) // Blue background for G button

// Standard gravity constant for converting m/s² to G units
#define STANDARD_GRAVITY 9.80665f // m/s² (standard gravity)

// Motion threshold parameters (in radians) - optimized for reduced sensitivity
#define MOTION_THRESHOLD (1.0f * M_PI / 180.0f) // Increased to 1.0 degree for less frequent updates
#define MOTION_THRESHOLD_SQR (MOTION_THRESHOLD * MOTION_THRESHOLD)

// ============================================================================
// FILTER TUNING PARAMETERS - Affect sensor data smoothness and responsiveness
// ============================================================================

// Low-pass filter parameters - Control sensor data smoothness
// LOWER alpha = More filtering (smoother but slower response)
// HIGHER alpha = Less filtering (faster response but more noise)
#define ACCEL_FILTER_ALPHA 0.3f // Accelerometer filter strength (0.1-0.8)
                                // Reduce if horizon is too jumpy from vibration
                                // Increase if horizon response is too slow

#define GYRO_FILTER_ALPHA 0.85f // Gyroscope filter strength (0.7-0.95)
                                // Keep high to preserve gyro responsiveness
                                // Reduce slightly if gyro noise causes jitter

// ============================================================================
// EKF TUNING PARAMETERS - Adjust these to fine-tune artificial horizon behavior
// ============================================================================

// Process noise parameters - Control how much the filter trusts the gyroscope prediction
// LOWER values = More trust in gyroscope, slower adaptation to accelerometer
// HIGHER values = Less trust in gyroscope, faster adaptation but more noise
static const float Q_angle = 0.0006f; // Angle process noise (rad²) - Controls pitch/roll prediction trust
                                      // Range: 0.0005-0.01 | Lower = smoother but slower response
                                      // If cross-coupling persists, try REDUCING this value

static const float Q_bias = 0.00005f; // Bias process noise (rad/s)² - Controls gyro bias adaptation speed
                                      // Range: 0.00001-0.0005 | Lower = more stable bias estimation
                                      // Increase if gyro drift is visible over time

// Cross-correlation factor - Controls coupling between pitch and roll axes
static const float Q_cross_factor = 0.001f; // Multiplier for cross-correlation terms (0.0-1.0)
                                            // 0.0 = No cross-correlation (independent axes)
                                            // 1.0 = Full cross-correlation
                                            // REDUCE this if cross-coupling is too strong

// Measurement noise parameter - Controls how much the filter trusts accelerometer measurements
// LOWER values = More trust in accelerometer, faster convergence but more sensitive to vibration
// HIGHER values = Less trust in accelerometer, slower convergence but more stable
static const float R_measure = 0.01f; // Accelerometer measurement noise (rad²)
                                      // Range: 0.05-0.5 | Increase if horizon is too jumpy
                                      // Decrease if horizon response is too slow

// ============================================================================
// CROSS-COUPLING TROUBLESHOOTING GUIDE
// ============================================================================
//
// PROBLEM: Pitch movements cause unwanted roll (and vice versa)
//
// SOLUTIONS (try in order):
//
// 1. REDUCE Q_cross_factor (start with 0.1, try 0.0 for completely independent axes)
//    - This is the most direct way to reduce cross-coupling
//
// 2. REDUCE Q_angle (try 0.001, 0.0008, 0.0005)
//    - Makes the filter trust gyroscope predictions more
//    - Reduces sensitivity to accelerometer noise that can cause coupling
//
// 3. INCREASE R_measure (try 0.2, 0.3, 0.4)
//    - Makes the filter trust accelerometer measurements less
//    - Reduces the impact of accelerometer cross-axis contamination
//
// 4. REDUCE ACCEL_FILTER_ALPHA (try 0.2, 0.1)
//    - More aggressive accelerometer filtering
//    - Smooths out vibrations that can cause cross-coupling
//
// 5. INCREASE motion threshold (try 1.5°, 2.0°)
//    - Reduces display updates for small movements
//    - Can mask minor cross-coupling effects
//
// ADVANCED: If cross-coupling persists, check sensor mounting alignment
// ============================================================================

// Global state structure
static struct
{
    lv_obj_t *container;       // Main container
    lv_obj_t *brown_square;    // Brown square object (ground)
    lv_obj_t *aircraft_line_h; // Horizontal aircraft line
    lv_obj_t *aircraft_line_v; // Vertical aircraft line
    lv_timer_t *display_update_timer;
    qmi8658_dev_t *imu_dev;
    ekf_state_t ekf;
    QueueHandle_t angle_queue;        // Queue to send angles from EKF task to LVGL task
    TaskHandle_t imu_ekf_task_handle; // Handle for the IMU/EKF task
    estimated_angles_t last_angles;   // Store last known angles
    filtered_data_t filtered_data;    // Store filtered sensor data
    bool is_visible;                  // Track if the artificial horizon screen is visible
    int container_width;              // Container width for calculations
    int container_height;             // Container height for calculations
    struct
    {
        float current_g;  // Current G force
        float max_g;      // Maximum G force since last reset
        lv_obj_t *button; // Button for G-force display and reset
        lv_obj_t *label;  // Label inside the button for G-force display
    } g_data;
} ah_state = {0};

// Initialize EKF
static void init_ekf(ekf_state_t *ekf)
{
    ekf->pitch = 0.0f;
    ekf->roll = 0.0f;
    ekf->bias_gx = 0.0f;
    ekf->bias_gy = 0.0f;

    // Initialize P matrix (error covariance) - optimized values
    // Lower initial values reflect better initial confidence
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j)
            {
                // Different initial uncertainties for different states
                if (i < 2)
                {
                    ekf->P[i][j] = 0.1f; // Lower initial uncertainty for angles (0.1 rad²)
                }
                else
                {
                    ekf->P[i][j] = 0.01f; // Very low initial uncertainty for biases (0.01 (rad/s)²)
                }
            }
            else
            {
                ekf->P[i][j] = 0.0f; // No initial cross-correlation
            }
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
    // Cross-correlation terms help handle coupling between pitch and roll
    float Q_cross = Q_cross_factor * Q_angle * dt; // Configurable cross-correlation strength
    float Q[4][4] = {
        {Q_angle * dt, Q_cross, 0, 0}, // Q_angle scaled by dt for consistency
        {Q_cross, Q_angle * dt, 0, 0}, // Cross-correlation between pitch and roll
        {0, 0, Q_bias * dt, 0},        // Q_bias scaled by dt
        {0, 0, 0, Q_bias * dt}};       // Gyro bias covariance

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
    // Calculate pitch and roll from accelerometer for aircraft instrument panel mounting:
    // Based on actual accelerometer readings analysis:
    // Level flight shows: ax=-1.842, ay=9.884, az=-0.218
    // This means: Y-axis points DOWN (toward gravity), not up!
    //
    // Corrected device coordinate system:
    // - Z-axis: Points away from pilot (forward flight direction) - ROLL axis
    // - X-axis: Points left - PITCH axis
    // - Y-axis: Points DOWN (toward gravity in level flight) - opposite to our assumption

    // In level flight: ax≈0, ay≈+9.8 (toward gravity), az≈0
    // Pitch (nose up/down): rotation around Z-axis (forward flight direction)
    // When nose pitches up: X component becomes more negative relative to gravity
    float pitch_m = atan2f(-ax, ay);

    // Roll (left/right bank): rotation around X-axis (left-pointing axis)
    // When banking left: Z component becomes more negative relative to gravity
    float roll_m = atan2f(-az, ay);

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

// Check if motion exceeds threshold
static inline bool motion_exceeds_threshold(estimated_angles_t *current, estimated_angles_t *previous)
{
    // Calculate squared differences (avoid sqrt for performance)
    float pitch_diff = current->pitch - previous->pitch;
    float roll_diff = current->roll - previous->roll;
    float squared_diff = pitch_diff * pitch_diff + roll_diff * roll_diff;

    return squared_diff > MOTION_THRESHOLD_SQR;
}

// Apply low-pass filter to sensor data
// Calculate G-force from accelerometer data
static float calculate_g_force(filtered_data_t *data)
{
    // Calculate the magnitude of the acceleration vector in m/s²
    float accel_magnitude = sqrtf(data->accel_x * data->accel_x +
                                  data->accel_y * data->accel_y +
                                  data->accel_z * data->accel_z);

    // Convert from m/s² to G units (load factor)
    return accel_magnitude / STANDARD_GRAVITY;
}

// Update G-force and calculate max G
static void update_g_force(float current_g)
{
    ah_state.g_data.current_g = current_g;

    // Update max G if current is higher
    if (current_g > ah_state.g_data.max_g)
    {
        ah_state.g_data.max_g = current_g;
    }
}

static void apply_low_pass_filter(qmi8658_data_t *raw_data, filtered_data_t *filtered)
{
    // Apply filter to accelerometer data (more aggressive filtering)
    filtered->accel_x = (ACCEL_FILTER_ALPHA * raw_data->accelX) + ((1.0f - ACCEL_FILTER_ALPHA) * filtered->accel_x);
    filtered->accel_y = (ACCEL_FILTER_ALPHA * raw_data->accelY) + ((1.0f - ACCEL_FILTER_ALPHA) * filtered->accel_y);
    filtered->accel_z = (ACCEL_FILTER_ALPHA * raw_data->accelZ) + ((1.0f - ACCEL_FILTER_ALPHA) * filtered->accel_z);

    // Apply filter to gyroscope data (less aggressive filtering)
    filtered->gyro_x = (GYRO_FILTER_ALPHA * raw_data->gyroX) + ((1.0f - GYRO_FILTER_ALPHA) * filtered->gyro_x);
    filtered->gyro_y = (GYRO_FILTER_ALPHA * raw_data->gyroY) + ((1.0f - GYRO_FILTER_ALPHA) * filtered->gyro_y);
    filtered->gyro_z = (GYRO_FILTER_ALPHA * raw_data->gyroZ) + ((1.0f - GYRO_FILTER_ALPHA) * filtered->gyro_z);
}

// G-force button click event handler
static void g_button_click_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        // Reset max G to current G
        ah_state.g_data.max_g = ah_state.g_data.current_g;
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
        // Adaptive delay based on visibility
        TickType_t task_delay = ah_state.is_visible ? pdMS_TO_TICKS(20) : pdMS_TO_TICKS(100);

        uint64_t current_time_us = esp_timer_get_time();
        float dt_actual = (current_time_us - last_time_us) / 1000000.0f;
        last_time_us = current_time_us;

        // Read IMU data
        if (qmi8658_is_data_ready(imu_dev, &ready) == ESP_OK && ready)
        {
            if (qmi8658_read_sensor_data(imu_dev, &data) == ESP_OK)
            {
                // Apply low-pass filtering to sensor data
                apply_low_pass_filter(&data, &ah_state.filtered_data);

                // Calculate and update G-force
                float current_g = calculate_g_force(&ah_state.filtered_data);
                update_g_force(current_g);

                // Only run EKF if visible (G-force still updates for logging purposes)
                if (ah_state.is_visible)
                {
                    // Correct axis mapping for aircraft instrument panel mounting:
                    // Device mounted vertically, Z-axis forward (roll), X-axis left (pitch), Y-axis down (yaw)
                    // - Roll: rotation around Z-axis (forward), use gyroZ
                    // - Pitch: rotation around X-axis (left), use gyroX
                    // Note: Gyro readings are in rad/s, EKF expects rad/s
                    ekf_predict(&ah_state.ekf, ah_state.filtered_data.gyro_x, ah_state.filtered_data.gyro_z, dt_actual);
                    ekf_update(&ah_state.ekf, ah_state.filtered_data.accel_x, ah_state.filtered_data.accel_y, ah_state.filtered_data.accel_z);

                    // Check if we should update the display
                    estimated_angles_t new_angles = {
                        .roll = ah_state.ekf.pitch,  // EKF pitch becomes display roll (banking)
                        .pitch = ah_state.ekf.roll}; // EKF roll becomes display pitch (nose up/down)

                    // Only send update if motion exceeds threshold
                    if (motion_exceeds_threshold(&new_angles, &ah_state.last_angles))
                    {
                        // Don't block if queue is full, just drop the old data
                        xQueueOverwrite(ah_state.angle_queue, &new_angles);
                        // Update last angles for next comparison
                        ah_state.last_angles.pitch = new_angles.pitch;
                        ah_state.last_angles.roll = new_angles.roll;
                    }
                }
            }
        }

        vTaskDelay(task_delay);
    }
}

// Update horizon using LVGL object transformations
static void update_horizon(estimated_angles_t angles)
{
    if (!ah_state.brown_square || !ah_state.is_visible)
        return;

    // Get roll and pitch angles (already in radians) - corrected mapping
    float roll_rad = angles.roll;   // Roll for banking display
    float pitch_rad = angles.pitch; // Pitch for nose up/down display

    // Calculate pitch offset in pixels (inverted for correct artificial horizon behavior)
    // When nose goes up (positive pitch), horizon should move down (negative offset)
    float pitch_offset_pixels = -pitch_rad * (ah_state.container_height / 2.0f);

    // Get brown square position for calculations
    int brown_square_y = (ah_state.container_height / 2) + (int)pitch_offset_pixels;
    int brown_square_x = -96; // Centered horizontally with overflow for rotation

    bsp_display_lock(0);
    // Position brown square (centered horizontally, adjusted vertically for pitch)
    lv_obj_set_pos(ah_state.brown_square, brown_square_x, brown_square_y);

    // Set rotation around center of container
    lv_obj_set_style_transform_pivot_x(ah_state.brown_square, ah_state.container_width / 2, 0);
    lv_obj_set_style_transform_pivot_y(ah_state.brown_square, ah_state.container_height / 2, 0);

    // Apply roll rotation (convert radians to decidegrees: rad * 180/π * 10)
    int roll_decidegrees = (int)(roll_rad * 1800.0f / M_PI);

    // lv_obj_set_style_transform_angle(ah_state.brown_square, roll_decidegrees, 0);
    lv_obj_set_style_transform_angle(ah_state.brown_square, roll_decidegrees, 0);

    bsp_display_unlock();
}

// Timer callback to update artificial horizon display
static void display_update_timer_cb(lv_timer_t *timer)
{
    // Skip updates if screen is not visible
    if (!ah_state.is_visible)
        return;

    static estimated_angles_t last_drawn_angles = {0};
    estimated_angles_t new_angles;

    // Try to get the latest angles from the queue
    if (xQueueReceive(ah_state.angle_queue, &new_angles, 0) == pdPASS)
    {
        // Only redraw if motion exceeds threshold
        if (motion_exceeds_threshold(&new_angles, &last_drawn_angles))
        {
            update_horizon(new_angles);
            last_drawn_angles.pitch = new_angles.pitch;
            last_drawn_angles.roll = new_angles.roll;
        }
    }

    // Update G-force display (always update when visible)
    char g_text[32];
    snprintf(g_text, sizeof(g_text), "%.1fG [%.1fG]",
             ah_state.g_data.current_g, ah_state.g_data.max_g);
    lv_label_set_text(ah_state.g_data.label, g_text);

    // Update text color based on G-force level (aircraft load factor thresholds)
    lv_color_t color;
    if (ah_state.g_data.current_g < 0.5f || ah_state.g_data.current_g > 4.0f)
    {
        color = G_COLOR_DANGER; // Red for extreme G (below 0.5G or above 4G)
    }
    else if (ah_state.g_data.current_g < 0.8f || ah_state.g_data.current_g > 2.0f)
    {
        color = G_COLOR_WARNING; // Orange for moderate G (0.5-0.8G or 2.0-4.0G)
    }
    else
    {
        color = G_COLOR_NORMAL; // White for normal G (0.8-2.0G)
    }
    lv_obj_set_style_text_color(ah_state.g_data.label, color, 0);
}

lv_obj_t *artificial_horizon_init(lv_obj_t *parent, qmi8658_dev_t *imu_dev)
{
    // Log memory status before initialization
    ESP_LOGI(TAG, "Memory status before initialization:");
    ESP_LOGI(TAG, "Free internal: %d bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Free PSRAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Largest free internal block: %d bytes", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    // Create main container with sky blue background
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(container, SKY_COLOR, LV_PART_MAIN); // Sky blue background
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);

    // Store container reference and get dimensions
    ah_state.container = container;

    // Force LVGL to layout the container first to get accurate dimensions
    lv_obj_update_layout(container);

    ah_state.container_width = lv_obj_get_width(container);
    ah_state.container_height = lv_obj_get_height(container);

    ESP_LOGI(TAG, "Container dimensions: %dx%d", ah_state.container_width, ah_state.container_height);

    // If dimensions are still 0, use fallback values
    if (ah_state.container_width == 0 || ah_state.container_height == 0)
    {
        ah_state.container_width = 466;  // Your mentioned screen width
        ah_state.container_height = 466; // Assume square screen
        ESP_LOGW(TAG, "Using fallback dimensions: %dx%d", ah_state.container_width, ah_state.container_height);
    }

    // Store IMU device handle
    ah_state.imu_dev = imu_dev;

    // Initialize last known angles to zero
    ah_state.last_angles.pitch = 0.0f;
    ah_state.last_angles.roll = 0.0f;

    // Initialize G-force data
    ah_state.g_data.current_g = 1.0f;
    ah_state.g_data.max_g = 1.0f;

    // Create FreeRTOS queue for EKF results
    ah_state.angle_queue = xQueueCreate(1, sizeof(estimated_angles_t)); // Queue of size 1 (only latest value matters)
    if (ah_state.angle_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create angle queue");
        lv_obj_del(container);
        return NULL;
    }

    // Calculate brown square size based on container width
    int brown_square_size = ah_state.container_width * 1.4142; // sqrt(2) for diagonal coverage
    ESP_LOGI(TAG, "Brown square size: %d", brown_square_size);
    bsp_display_lock(0); // Lock display to prevent flickering during initialization
    // Create brown square object (ground)
    ah_state.brown_square = lv_obj_create(container);
    lv_obj_set_size(ah_state.brown_square, brown_square_size, brown_square_size);
    lv_obj_set_style_bg_color(ah_state.brown_square, GROUND_COLOR, LV_PART_MAIN);   // Brown background
    lv_obj_set_style_border_color(ah_state.brown_square, LINE_COLOR, LV_PART_MAIN); // White border
    lv_obj_set_style_border_width(ah_state.brown_square, HORIZON_LINE_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_border_side(ah_state.brown_square, LV_BORDER_SIDE_TOP, LV_PART_MAIN); // Only top border
    lv_obj_set_style_pad_all(ah_state.brown_square, 0, LV_PART_MAIN);
    lv_obj_clear_flag(ah_state.brown_square, LV_OBJ_FLAG_SCROLLABLE);

    // Position brown square so its top edge is at container center when pitch = 0
    int initial_y = (ah_state.container_height / 2);
    int initial_x = -(brown_square_size - ah_state.container_width) / 2;

    ESP_LOGI(TAG, "Brown square initial position: (%d, %d)", initial_x, initial_y);

    lv_obj_set_pos(ah_state.brown_square, initial_x, initial_y);

    // Create aircraft symbol lines (inverted T)
    // Horizontal line (aircraft wings)
    ah_state.aircraft_line_h = lv_line_create(container);
    static lv_point_precise_t line_h_points[] = {
        {0, 0},
        {REFERENCE_LINE_WIDTH, 0}};
    lv_line_set_points(ah_state.aircraft_line_h, line_h_points, 2);
    lv_obj_set_style_line_color(ah_state.aircraft_line_h, AIRCRAFT_LINE_COLOR, LV_PART_MAIN);
    lv_obj_set_style_line_width(ah_state.aircraft_line_h, 8, LV_PART_MAIN);
    lv_obj_align(ah_state.aircraft_line_h, LV_ALIGN_CENTER, 0, 0);

    // Vertical line (aircraft center)
    ah_state.aircraft_line_v = lv_line_create(container);
    static lv_point_precise_t line_v_points[] = {
        {0, -15},
        {0, 20}};
    lv_line_set_points(ah_state.aircraft_line_v, line_v_points, 2);
    lv_obj_set_style_line_color(ah_state.aircraft_line_v, AIRCRAFT_LINE_COLOR, LV_PART_MAIN);
    lv_obj_set_style_line_width(ah_state.aircraft_line_v, 5, LV_PART_MAIN);
    lv_obj_align(ah_state.aircraft_line_v, LV_ALIGN_CENTER, 0, 0);

    // Create G-force button (occupies bottom fourth of container)
    ah_state.g_data.button = lv_btn_create(container);
    lv_obj_set_size(ah_state.g_data.button, LV_PCT(100), LV_PCT(20)); // Full width, 20% height
    lv_obj_align(ah_state.g_data.button, LV_ALIGN_BOTTOM_MID, 0, 0);  // Bottom center
    lv_obj_set_style_bg_color(ah_state.g_data.button, G_BUTTON_BG_COLOR, LV_PART_MAIN);
    lv_obj_set_style_border_width(ah_state.g_data.button, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(ah_state.g_data.button, g_button_click_cb, LV_EVENT_CLICKED, NULL);

    // Create G-force label inside the button
    ah_state.g_data.label = lv_label_create(ah_state.g_data.button);
    lv_obj_set_style_text_font(ah_state.g_data.label, &lv_font_montserrat_38, 0);
    lv_obj_set_style_text_color(ah_state.g_data.label, G_COLOR_NORMAL, 0);
    lv_label_set_text(ah_state.g_data.label, "1.0G [1.0G]");
    lv_obj_center(ah_state.g_data.label); // Center the label in the button
    bsp_display_unlock();

    // Create the IMU/EKF FreeRTOS task with optimized stack size
    BaseType_t xReturned = xTaskCreate(
        imu_ekf_task,                 // Task function
        "IMU_EKF_Task",               // Task name
        2048 * 4,                     // Reduced stack size from 4096 to 2048 (saves 2KB per task)
        (void *)imu_dev,              // Parameter to pass
        5,                            // Priority (higher than LVGL task normally)
        &ah_state.imu_ekf_task_handle // Task handle
    );
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create IMU_EKF_Task");
        vQueueDelete(ah_state.angle_queue);
        lv_obj_del(container);
        return NULL;
    }

    // Initialize visibility state
    ah_state.is_visible = true; // Assume visible initially

    // Force an initial horizon update to position the brown square correctly
    estimated_angles_t initial_angles = {0.0f, 0.0f};
    update_horizon(initial_angles);

    // Create display update timer for LVGL
    ah_state.display_update_timer = lv_timer_create(display_update_timer_cb, DISPLAY_REFRESH_PERIOD_MS, NULL);
    if (ah_state.display_update_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create display update timer");
        vTaskDelete(ah_state.imu_ekf_task_handle);
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
    if (ah_state.angle_queue)
    {
        vQueueDelete(ah_state.angle_queue);
        ah_state.angle_queue = NULL;
    }

    // LVGL objects are typically deleted by their parent or when the screen changes.
    // If `container` is the only child of a screen, deleting the screen will delete it.
    // If it's part of a tileview, it will be cleaned up when the tileview is destroyed.
    // Avoid `lv_obj_del(ah_state.container)` here unless you manage its parentage carefully.
}

// Function to set visibility state for performance optimization
void artificial_horizon_set_visible(bool visible)
{
    ah_state.is_visible = visible;
}

// Function to check if artificial horizon is visible
bool artificial_horizon_is_visible(void)
{
    return ah_state.is_visible;
}