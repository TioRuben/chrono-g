#include <assert.h>
#include "display_port.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_sh8601.h"
#include "esp_lcd_touch_cst9217.h"
#include "bsp/esp32_s3_touch_amoled_1_75.h" // for pins and constants
#include "esp_timer.h"

static const char *TAG = "app_display";

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFE, (uint8_t[]){0x20}, 1, 0},
    {0x19, (uint8_t[]){0x10}, 1, 0},
    {0x1C, (uint8_t[]){0xA0}, 1, 0},

    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xD7}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 600},
    {0x11, NULL, 0, 600},
    {0x29, NULL, 0, 0},
};

static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)lv_indev_get_user_data(indev);
    // Read latest touch sample from controller
    esp_lcd_touch_read_data(tp);
    uint16_t x, y;
    uint8_t touched;
    esp_lcd_touch_get_coordinates(tp, &x, &y, NULL, &touched, 1);
    if (touched)
    {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// Periodic wake for LVGL input processing
static esp_timer_handle_t s_touch_wake_timer = NULL;
static void touch_wake_timer_cb(void *arg)
{
    (void)arg;
    lvgl_port_task_wake(LVGL_PORT_EVENT_TOUCH, NULL);
}

lv_display_t *app_display_start(const lvgl_port_cfg_t *port_cfg,
                                int buf_height,
                                bool double_buf,
                                int pclk_mhz,
                                bool enable_touch)
{
    ESP_RETURN_ON_FALSE(port_cfg != NULL, NULL, TAG, "port_cfg is NULL");
    if (buf_height <= 0)
        buf_height = 116;
    if (pclk_mhz <= 0)
        pclk_mhz = 80;

    // Init LVGL port (creates tick + task)
    {
        esp_err_t __err = lvgl_port_init(port_cfg);
        if (__err != ESP_OK)
        {
            ESP_LOGE(TAG, "lvgl_port_init failed: %d", (int)__err);
            return NULL;
        }
    }

    // Pause LVGL timers during display creation to avoid early flushes
    lv_timer_enable(false);

    // SPI bus for SH8601 QSPI
    // Configure SPI with a larger max transfer size to reduce the number of queued chunks
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(BSP_LCD_PCLK,
                                                                 BSP_LCD_DATA0,
                                                                 BSP_LCD_DATA1,
                                                                 BSP_LCD_DATA2,
                                                                 BSP_LCD_DATA3,
                                                                 24 * 1024); // max transfer size (bytes)
    ESP_ERROR_CHECK(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));

    // QSPI IO config with adjustable pclk
    esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(BSP_LCD_CS, NULL, NULL);
    io_config.pclk_hz = pclk_mhz * 1000 * 1000;
    // Use a deeper queue so large refreshes don't fail to enqueue
    io_config.trans_queue_depth = 6;

    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &io_handle));

    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };

    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    esp_lcd_panel_set_gap(panel_handle, 0x06, 0);
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Try full-frame double buffering in PSRAM (direct mode) first to minimize tearing
    lv_display_t *disp = NULL;
    do
    {
        const uint32_t full_buffer_pixels = BSP_LCD_H_RES * BSP_LCD_V_RES;
        lvgl_port_display_cfg_t direct_cfg = {
            .io_handle = io_handle,
            .panel_handle = panel_handle,
            .buffer_size = full_buffer_pixels,
            .double_buffer = true,
            .trans_size = 48 * 1024, // SRAM bounce for PSRAM
            .hres = BSP_LCD_H_RES,
            .vres = BSP_LCD_V_RES,
            .monochrome = false,
            .color_format = LV_COLOR_FORMAT_RGB565,
            .rotation = {
                .swap_xy = false,
                .mirror_x = false,
                .mirror_y = false,
            },
            .flags = {
                .buff_dma = true,
                .buff_spiram = true,
                .sw_rotate = 1,
                .swap_bytes = 1,
                .full_refresh = 1,
                .direct_mode = 0,
            },
        };

        disp = lvgl_port_add_disp(&direct_cfg);
        if (disp)
        {
            // Lower pclk for PSRAM-backed full frames to reduce SPI pressure
            if (pclk_mhz > 40)
            {
                // io_config.pclk_hz = 40 * 1000 * 1000;
            }
            ESP_LOGI(TAG, "LVGL display ready: FULL, double buffer(s), PSRAM, DMA, direct_mode (pclk=%dMHz)", (int)(io_config.pclk_hz / 1000000));
            break;
        }
        else
        {
            ESP_LOGW(TAG, "LVGL full-screen double buffer in PSRAM failed, falling back to partial buffers");
        }
    } while (0);

    // Try multiple buffer configurations (auto-fallback) until LVGL accepts
    const int heights_try[] = {buf_height, 232, 160, 128, 116, 96, 80, 64};
    const size_t heights_cnt = sizeof(heights_try) / sizeof(heights_try[0]);

    for (size_t hi = 0; hi < heights_cnt && disp == NULL; ++hi)
    {
        int h_try = heights_try[hi];
        if (h_try <= 0)
            continue;

        // 1) Internal RAM, DMA, double (preferred)
        for (int pass = 0; pass < 4 && disp == NULL; ++pass)
        {
            bool use_double = double_buf;
            bool use_spiram = false;
            bool use_dma = true;

            // Try order: internal double -> internal single -> PSRAM single -> PSRAM double
            if (pass == 1)
            {
                use_double = false;
            }
            if (pass == 2)
            {
                use_double = false;
                use_spiram = true;
            }
            if (pass == 3)
            {
                use_double = true;
                use_spiram = true;
            }

            const uint32_t buffer_size = BSP_LCD_H_RES * h_try; // pixels
            lvgl_port_display_cfg_t try_cfg = {
                .io_handle = io_handle,
                .panel_handle = panel_handle,
                .buffer_size = buffer_size,
                .double_buffer = use_double,
                .trans_size = use_spiram ? (24 * 1024) : 0, // larger SRAM bounce for PSRAM
                .hres = BSP_LCD_H_RES,
                .vres = BSP_LCD_V_RES,
                .monochrome = false,
                .color_format = LV_COLOR_FORMAT_RGB565,
                .rotation = {
                    .swap_xy = false,
                    .mirror_x = false,
                    .mirror_y = false,
                },
                .flags = {
                    .buff_dma = use_dma,
                    .buff_spiram = use_spiram,
                    .sw_rotate = 1,
                    .swap_bytes = 1,
                    .full_refresh = 0,
                    .direct_mode = 0,
                },
            };

            disp = lvgl_port_add_disp(&try_cfg);
            if (disp)
            {
                // If buffers landed in PSRAM, drop panel pclk to 40MHz to reduce SPI pressure
                if (use_spiram && pclk_mhz > 40)
                {
                    io_config.pclk_hz = 40 * 1000 * 1000;
                }
                ESP_LOGI(TAG, "LVGL display ready: H=%d, %s buffer(s), %s, %s",
                         h_try,
                         use_double ? "double" : "single",
                         use_spiram ? "PSRAM" : "internal",
                         use_dma ? "DMA" : "no-DMA");
                break;
            }
            else
            {
                ESP_LOGW(TAG, "LVGL display config failed, retrying... (H=%d, %s, %s)",
                         h_try,
                         use_double ? "double" : "single",
                         use_spiram ? "PSRAM" : "internal");
            }
        }
    }

    if (!disp)
    {
        ESP_LOGE(TAG, "All LVGL display configurations failed");
        return NULL;
    }

    if (enable_touch)
    {
        // Reuse the BSP's shared I2C bus to avoid double-acquire conflicts with IMU
        i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();

        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
        tp_io_config.scl_speed_hz = 400000; // fast mode
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle));

        const esp_lcd_touch_config_t tp_cfg = {
            .x_max = BSP_LCD_H_RES,
            .y_max = BSP_LCD_V_RES,
            .rst_gpio_num = BSP_LCD_TOUCH_RST,
            .int_gpio_num = BSP_LCD_TOUCH_INT,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                // Align touch to display rotated 270 deg in main:
                // No axis swap; mirror both axes to match physical orientation
                .swap_xy = 0,
                .mirror_x = 1,
                .mirror_y = 1,
            },
        };
        esp_lcd_touch_handle_t tp = NULL;
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst9217(tp_io_handle, &tp_cfg, &tp));

        // Wire touch into LVGL using the port helper (handles IRQ/poll wake-ups)
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = disp,
            .handle = tp,
            .scale = {.x = 1.0f, .y = 1.0f},
        };
        lv_indev_t *indev = lvgl_port_add_touch(&touch_cfg);
        if (!indev)
        {
            ESP_LOGW(TAG, "lvgl_port_add_touch failed; falling back to manual input device");
            indev = lv_indev_create();
            lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
            lv_indev_set_disp(indev, disp);
            lv_indev_set_user_data(indev, tp);
            lv_indev_set_read_cb(indev, lvgl_touch_read_cb);
        }

        // Start a periodic timer to wake LVGL task for input handling
        const esp_timer_create_args_t targs = {
            .callback = &touch_wake_timer_cb,
            .name = "lv_touch_wake",
        };
        if (esp_timer_create(&targs, &s_touch_wake_timer) == ESP_OK)
        {
            esp_timer_start_periodic(s_touch_wake_timer, 10 * 1000); // 10 ms
        }
        else
        {
            ESP_LOGW(TAG, "Failed to create touch wake timer; touch may be less responsive");
        }
    }

    // Re-enable LVGL timers now that the display and optional touch are ready
    lv_timer_enable(true);

    ESP_LOGI(TAG, "Display initialized: %dx%d, pclk=%dMHz",
             BSP_LCD_H_RES, BSP_LCD_V_RES, pclk_mhz);

    return disp;
}
