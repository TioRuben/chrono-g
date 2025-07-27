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

// ESP-DSP includes
#include "ekf_imu13states.h"
#include "esp_dsp.h"

static const char *TAG = "Artificial Horizon ESP-DSP";

// Display and Drawing Parameters
#define DISPLAY_REFRESH_PERIOD_MS 17 // 25 FPS refresh rate for display

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
// ESP-DSP EKF PARAMETERS - Much simpler configuration than custom implementation
// ============================================================================

// Sensor noise parameters for ESP-DSP EKF
#define ACCEL_NOISE_VARIANCE 0.01f // Accelerometer measurement noise (m/s²)²
#define GYRO_NOISE_VARIANCE 0.001f // Gyroscope measurement noise (rad/s)²
#define MAGN_NOISE_VARIANCE 0.1f   // Magnetometer measurement noise (µT)²

// Low-pass filter parameters - Control sensor data smoothness
#define ACCEL_FILTER_ALPHA 0.3f // Accelerometer filter strength (0.1-0.8)
#define GYRO_FILTER_ALPHA 0.85f // Gyroscope filter strength (0.7-0.95)

// Global state structure - simplified with ESP-DSP
static struct
{
    lv_obj_t *container;       // Main container
    lv_obj_t *brown_square;    // Brown square object (ground)
    lv_obj_t *aircraft_line_h; // Horizontal aircraft line
    lv_obj_t *aircraft_line_v; // Vertical aircraft line
    lv_timer_t *display_update_timer;
    qmi8658_dev_t *imu_dev;
    ekf_imu13states *ekf_filter;      // ESP-DSP EKF filter
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

// Initialize ESP-DSP EKF
static esp_err_t init_esp_dsp_ekf(void)
{
    // Create EKF instance
    ah_state.ekf_filter = new ekf_imu13states();
    if (!ah_state.ekf_filter)
    {
        ESP_LOGE(TAG, "Failed to create ESP-DSP EKF instance");
        return ESP_FAIL;
    }

    // Initialize the EKF
    ah_state.ekf_filter->Init();

    ESP_LOGI(TAG, "ESP-DSP EKF initialized successfully");
    return ESP_OK;
}

// Apply low-pass filter to sensor data
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

// Calculate G-force from accelerometer data
static float calculate_g_force(filtered_data_t *data)
{
    float accel_magnitude = sqrtf(data->accel_x * data->accel_x +
                                  data->accel_y * data->accel_y +
                                  data->accel_z * data->accel_z);
    return accel_magnitude / STANDARD_GRAVITY;
}

// Update G-force and calculate max G
static void update_g_force(float current_g)
{
    ah_state.g_data.current_g = current_g;
    if (current_g > ah_state.g_data.max_g)
    {
        ah_state.g_data.max_g = current_g;
    }
}

// Check if motion exceeds threshold
static inline bool motion_exceeds_threshold(estimated_angles_t *current, estimated_angles_t *previous)
{
    float pitch_diff = current->pitch - previous->pitch;
    float roll_diff = current->roll - previous->roll;
    float squared_diff = pitch_diff * pitch_diff + roll_diff * roll_diff;
    return squared_diff > MOTION_THRESHOLD_SQR;
}

// G-force button click event handler
static void g_button_click_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        ah_state.g_data.max_g = ah_state.g_data.current_g;
    }
}

// Convert quaternion to Euler angles (pitch, roll)
static void quaternion_to_euler(float q[4], float *pitch, float *roll)
{
    // Convert quaternion to Euler angles using ESP-DSP utility functions
    dspm::Mat quat_mat = ekf::quat2eul(q);

    // Extract pitch and roll (assuming standard aircraft convention)
    *pitch = quat_mat(1, 0); // Pitch (rotation around Y-axis)
    *roll = quat_mat(0, 0);  // Roll (rotation around X-axis)
}

// IMU reading and ESP-DSP EKF task
static void imu_ekf_task(void *arg)
{
    qmi8658_dev_t *imu_dev = (qmi8658_dev_t *)arg;

    uint64_t last_time_us = esp_timer_get_time();
    qmi8658_data_t data;
    bool ready;

    // ESP-DSP EKF measurement noise covariance matrix (diagonal)
    float R[6] = {
        ACCEL_NOISE_VARIANCE, ACCEL_NOISE_VARIANCE, ACCEL_NOISE_VARIANCE, // Accelerometer X, Y, Z
        MAGN_NOISE_VARIANCE, MAGN_NOISE_VARIANCE, MAGN_NOISE_VARIANCE     // Magnetometer X, Y, Z (if available)
    };

    while (1)
    {
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

                // Only run EKF if visible
                if (ah_state.is_visible && ah_state.ekf_filter)
                {
                    // Prepare gyroscope data for ESP-DSP EKF (rad/s)
                    float gyro_data[3] = {
                        ah_state.filtered_data.gyro_x,
                        ah_state.filtered_data.gyro_y,
                        ah_state.filtered_data.gyro_z};

                    // Prepare accelerometer data for ESP-DSP EKF (m/s²)
                    float accel_data[3] = {
                        ah_state.filtered_data.accel_x,
                        ah_state.filtered_data.accel_y,
                        ah_state.filtered_data.accel_z};

                    // For now, we'll use zero magnetometer data (can be enhanced later)
                    float magn_data[3] = {0.0f, 0.0f, 0.0f};

                    // Process EKF prediction step with gyroscope data
                    ah_state.ekf_filter->Process(gyro_data, dt_actual);

                    // Update with accelerometer and magnetometer measurements
                    ah_state.ekf_filter->UpdateRefMeasurement(accel_data, magn_data, R);

                    // Extract attitude quaternion from EKF state
                    float quaternion[4];
                    // Access the quaternion from EKF state vector (first 4 elements)
                    for (int i = 0; i < 4; i++)
                    {
                        quaternion[i] = ah_state.ekf_filter->X(i, 0);
                    }

                    // Convert quaternion to pitch and roll
                    float pitch, roll;
                    quaternion_to_euler(quaternion, &pitch, &roll);

                    estimated_angles_t new_angles = {
                        .pitch = pitch,
                        .roll = roll};

                    // Only send update if motion exceeds threshold
                    if (motion_exceeds_threshold(&new_angles, &ah_state.last_angles))
                    {
                        xQueueOverwrite(ah_state.angle_queue, &new_angles);
                        ah_state.last_angles = new_angles;
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

    float roll_rad = angles.roll;
    float pitch_rad = angles.pitch;

    // Calculate pitch offset in pixels
    float pitch_offset_pixels = -pitch_rad * (ah_state.container_height / 2.0f);

    int brown_square_y = (ah_state.container_height / 2) + (int)pitch_offset_pixels;
    int brown_square_x = -96;

    bsp_display_lock(0);
    lv_obj_set_pos(ah_state.brown_square, brown_square_x, brown_square_y);

    lv_obj_set_style_transform_pivot_x(ah_state.brown_square, ah_state.container_width / 2, 0);
    lv_obj_set_style_transform_pivot_y(ah_state.brown_square, 0, 0);

    int roll_decidegrees = (int)(roll_rad * 1800.0f / M_PI);
    lv_obj_set_style_transform_angle(ah_state.brown_square, roll_decidegrees, 0);

    bsp_display_unlock();
}

// Timer callback to update artificial horizon display
static void display_update_timer_cb(lv_timer_t *timer)
{
    if (!ah_state.is_visible)
        return;

    static estimated_angles_t last_drawn_angles = {0};
    estimated_angles_t new_angles;

    if (xQueueReceive(ah_state.angle_queue, &new_angles, 0) == pdPASS)
    {
        if (motion_exceeds_threshold(&new_angles, &last_drawn_angles))
        {
            update_horizon(new_angles);
            last_drawn_angles = new_angles;
        }
    }

    // Update G-force display
    char g_text[32];
    snprintf(g_text, sizeof(g_text), "%.1fg [%.1fg]",
             ah_state.g_data.current_g, ah_state.g_data.max_g);
    lv_label_set_text(ah_state.g_data.label, g_text);

    // Update text color based on G-force level
    lv_color_t color;
    if (ah_state.g_data.current_g < 0.5f || ah_state.g_data.current_g > 4.0f)
    {
        color = G_COLOR_DANGER;
    }
    else if (ah_state.g_data.current_g < 0.8f || ah_state.g_data.current_g > 2.0f)
    {
        color = G_COLOR_WARNING;
    }
    else
    {
        color = G_COLOR_NORMAL;
    }
    lv_obj_set_style_text_color(ah_state.g_data.label, color, 0);
}

lv_obj_t *artificial_horizon_init(lv_obj_t *parent, qmi8658_dev_t *imu_dev)
{
    ESP_LOGI(TAG, "Initializing Artificial Horizon with ESP-DSP EKF");

    // Create main container with sky blue background
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(container, SKY_COLOR, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);

    ah_state.container = container;
    lv_obj_update_layout(container);

    ah_state.container_width = lv_obj_get_width(container);
    ah_state.container_height = lv_obj_get_height(container);

    if (ah_state.container_width == 0 || ah_state.container_height == 0)
    {
        ah_state.container_width = 466;
        ah_state.container_height = 466;
        ESP_LOGW(TAG, "Using fallback dimensions: %dx%d", ah_state.container_width, ah_state.container_height);
    }

    // Store IMU device handle
    ah_state.imu_dev = imu_dev;

    // Initialize ESP-DSP EKF
    if (init_esp_dsp_ekf() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize ESP-DSP EKF");
        lv_obj_del(container);
        return NULL;
    }

    // Initialize angles and G-force data
    ah_state.last_angles.pitch = 0.0f;
    ah_state.last_angles.roll = 0.0f;
    ah_state.g_data.current_g = 1.0f;
    ah_state.g_data.max_g = 1.0f;

    // Create FreeRTOS queue
    ah_state.angle_queue = xQueueCreate(1, sizeof(estimated_angles_t));
    if (ah_state.angle_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create angle queue");
        delete ah_state.ekf_filter;
        lv_obj_del(container);
        return NULL;
    }

    // Calculate brown square size
    int brown_square_size = ah_state.container_width * 2;

    bsp_display_lock(0);

    // Create brown square object (ground)
    ah_state.brown_square = lv_obj_create(container);
    lv_obj_set_size(ah_state.brown_square, brown_square_size, brown_square_size);
    lv_obj_set_style_bg_color(ah_state.brown_square, GROUND_COLOR, LV_PART_MAIN);
    lv_obj_set_style_border_color(ah_state.brown_square, LINE_COLOR, LV_PART_MAIN);
    lv_obj_set_style_border_width(ah_state.brown_square, HORIZON_LINE_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_border_side(ah_state.brown_square, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ah_state.brown_square, 0, LV_PART_MAIN);
    lv_obj_clear_flag(ah_state.brown_square, LV_OBJ_FLAG_SCROLLABLE);

    int initial_y = (ah_state.container_height / 2);
    int initial_x = -(brown_square_size - ah_state.container_width) / 2;
    lv_obj_set_pos(ah_state.brown_square, initial_x, initial_y);

    // Create aircraft symbol lines
    ah_state.aircraft_line_h = lv_line_create(container);
    static lv_point_precise_t line_h_points[] = {{0, 0}, {REFERENCE_LINE_WIDTH, 0}};
    lv_line_set_points(ah_state.aircraft_line_h, line_h_points, 2);
    lv_obj_set_style_line_color(ah_state.aircraft_line_h, AIRCRAFT_LINE_COLOR, LV_PART_MAIN);
    lv_obj_set_style_line_width(ah_state.aircraft_line_h, 8, LV_PART_MAIN);
    lv_obj_align(ah_state.aircraft_line_h, LV_ALIGN_CENTER, 0, 0);

    ah_state.aircraft_line_v = lv_line_create(container);
    static lv_point_precise_t line_v_points[] = {{0, -15}, {0, 20}};
    lv_line_set_points(ah_state.aircraft_line_v, line_v_points, 2);
    lv_obj_set_style_line_color(ah_state.aircraft_line_v, AIRCRAFT_LINE_COLOR, LV_PART_MAIN);
    lv_obj_set_style_line_width(ah_state.aircraft_line_v, 5, LV_PART_MAIN);
    lv_obj_align(ah_state.aircraft_line_v, LV_ALIGN_CENTER, 0, 0);

    // Create G-force button
    ah_state.g_data.button = lv_btn_create(container);
    lv_obj_set_size(ah_state.g_data.button, LV_PCT(100), LV_PCT(20));
    lv_obj_align(ah_state.g_data.button, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(ah_state.g_data.button, G_BUTTON_BG_COLOR, LV_PART_MAIN);
    lv_obj_set_style_border_width(ah_state.g_data.button, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(ah_state.g_data.button, g_button_click_cb, LV_EVENT_CLICKED, NULL);

    ah_state.g_data.label = lv_label_create(ah_state.g_data.button);
    lv_obj_set_style_text_font(ah_state.g_data.label, &lv_font_montserrat_38, 0);
    lv_obj_set_style_text_color(ah_state.g_data.label, G_COLOR_NORMAL, 0);
    lv_label_set_text(ah_state.g_data.label, "1.0g [1.0g]");
    lv_obj_center(ah_state.g_data.label);

    bsp_display_unlock();

    // Create the IMU/EKF FreeRTOS task
    BaseType_t xReturned = xTaskCreate(
        imu_ekf_task,
        "IMU_EKF_ESP_DSP_Task",
        4096, // Increased stack size for C++ ESP-DSP operations
        (void *)imu_dev,
        5,
        &ah_state.imu_ekf_task_handle);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create IMU_EKF_Task");
        vQueueDelete(ah_state.angle_queue);
        delete ah_state.ekf_filter;
        lv_obj_del(container);
        return NULL;
    }

    ah_state.is_visible = true;

    // Force initial horizon update
    estimated_angles_t initial_angles = {0.0f, 0.0f};
    update_horizon(initial_angles);

    // Create display update timer
    ah_state.display_update_timer = lv_timer_create(display_update_timer_cb, DISPLAY_REFRESH_PERIOD_MS, NULL);
    if (ah_state.display_update_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create display update timer");
        vTaskDelete(ah_state.imu_ekf_task_handle);
        vQueueDelete(ah_state.angle_queue);
        delete ah_state.ekf_filter;
        lv_obj_del(container);
        return NULL;
    }

    ESP_LOGI(TAG, "Artificial Horizon with ESP-DSP EKF initialized successfully");
    return container;
}

// Function to de-initialize the artificial horizon
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
    if (ah_state.ekf_filter)
    {
        delete ah_state.ekf_filter;
        ah_state.ekf_filter = NULL;
    }
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
