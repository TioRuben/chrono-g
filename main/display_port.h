#pragma once

#include "lvgl.h"
#include "esp_lvgl_port.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * Initialize display (SH8601 via QSPI) and LVGL with DMA double buffers and optional touch.
     *
     * @param port_cfg       LVGL port task/tick configuration
     * @param buf_height     Draw buffer height in lines (e.g., 116)
     * @param double_buf     true to allocate 2 draw buffers
     * @param pclk_mhz       Panel QSPI pixel clock in MHz (e.g., 80)
     * @param enable_touch   true to initialize CST9217 touch and add LVGL input device
     * @return lv_display_t* LVGL display handle or NULL on error
     */
    lv_display_t *app_display_start(const lvgl_port_cfg_t *port_cfg,
                                    int buf_height,
                                    bool double_buf,
                                    int pclk_mhz,
                                    bool enable_touch);

#ifdef __cplusplus
}
#endif
