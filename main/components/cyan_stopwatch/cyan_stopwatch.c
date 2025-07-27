#include "cyan_stopwatch.h"
#include "esp_log.h"
#include "stdio.h"

// Component state
static struct
{
    uint32_t seconds_counted;
    bool stopwatch_running;
    lv_timer_t *timer;
    lv_obj_t *time_label;
    lv_obj_t *btn;
    lv_obj_t *reset_btn;
    lv_obj_t *btn_container;
} cyan_state = {0};

// Convert seconds to HH:MM:SS format
static void format_time(char *buf, size_t buf_size, uint32_t total_seconds)
{
    uint32_t hours = total_seconds / 3600;
    uint32_t minutes = (total_seconds % 3600) / 60;
    uint32_t seconds = total_seconds % 60;
    snprintf(buf, buf_size, "%02u:%02u:%02u", (unsigned int)hours, (unsigned int)minutes, (unsigned int)seconds);
}

// Timer callback to update stopwatch
static void update_stopwatch(lv_timer_t *timer)
{
    if (cyan_state.stopwatch_running)
    {
        cyan_state.seconds_counted++;
        char buf[16]; // Restored to 16 bytes to accommodate format (max 14 bytes needed + null terminator)
        format_time(buf, sizeof(buf), cyan_state.seconds_counted);
        lv_label_set_text(cyan_state.time_label, buf);
    }
}

// Button event handler
static void btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);

    // Check if it's a click event
    if (code != LV_EVENT_CLICKED)
        return;

    if (target == cyan_state.btn)
    {
        if (!cyan_state.stopwatch_running)
        {
            // Starting the stopwatch
            cyan_state.stopwatch_running = true;
            lv_timer_resume(cyan_state.timer);
            lv_obj_set_style_bg_color(cyan_state.btn, lv_color_hex(0x770000), LV_PART_MAIN);
            lv_label_set_text(lv_obj_get_child(cyan_state.btn, 0), "STOP");
            lv_obj_set_size(cyan_state.btn, lv_pct(100), lv_pct(100));
            if (cyan_state.reset_btn)
            {
                lv_obj_add_flag(cyan_state.reset_btn, LV_OBJ_FLAG_HIDDEN);
            }
        }
        else
        {
            // Stopping the stopwatch
            cyan_state.stopwatch_running = false;
            lv_timer_pause(cyan_state.timer);
            lv_obj_set_style_bg_color(cyan_state.btn, lv_color_hex(0x007700), LV_PART_MAIN);
            lv_label_set_text(lv_obj_get_child(cyan_state.btn, 0), "         START");
            lv_obj_set_size(cyan_state.btn, lv_pct(50), lv_pct(100));

            if (cyan_state.seconds_counted > 0)
            {
                // Show reset button
                if (!cyan_state.reset_btn)
                {
                    // Create reset button
                    cyan_state.reset_btn = lv_btn_create(cyan_state.btn_container);
                    lv_obj_set_size(cyan_state.reset_btn, lv_pct(50), lv_pct(100));
                    lv_obj_set_style_bg_color(cyan_state.reset_btn, lv_color_hex(0x808080), LV_PART_MAIN);
                    lv_obj_set_style_radius(cyan_state.reset_btn, 0, LV_PART_MAIN);
                    lv_obj_set_style_border_width(cyan_state.reset_btn, 0, LV_PART_MAIN);
                    lv_obj_set_style_pad_all(cyan_state.reset_btn, 0, LV_PART_MAIN);
                    lv_obj_t *reset_label = lv_label_create(cyan_state.reset_btn);
                    lv_label_set_text(reset_label, "RESET         ");
                    lv_obj_set_style_text_align(reset_label, LV_TEXT_ALIGN_LEFT, 0);
                    lv_obj_set_style_text_font(reset_label, &lv_font_montserrat_24, 0);
                    lv_obj_center(reset_label);
                    lv_obj_add_event_cb(cyan_state.reset_btn, btn_event_cb, LV_EVENT_CLICKED, NULL);
                }
                else
                {
                    lv_obj_clear_flag(cyan_state.reset_btn, LV_OBJ_FLAG_HIDDEN);
                }
            }
        }
    }
    else if (target == cyan_state.reset_btn)
    {
        // Reset the stopwatch
        cyan_state.seconds_counted = 0;
        lv_label_set_text(cyan_state.time_label, "00:00:00");
        lv_obj_del(cyan_state.reset_btn);
        cyan_state.reset_btn = NULL;
        cyan_state.stopwatch_running = false;
        lv_timer_pause(cyan_state.timer);
        lv_obj_set_style_bg_color(cyan_state.btn, lv_color_hex(0x007700), LV_PART_MAIN);
        lv_obj_set_size(cyan_state.btn, lv_pct(100), lv_pct(100));
        lv_label_set_text(lv_obj_get_child(cyan_state.btn, 0), "START");
    }
}

lv_obj_t *cyan_stopwatch_init(lv_obj_t *parent)
{
    // Create main container
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(container, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);

    // Create time display label with cyan color
    cyan_state.time_label = lv_label_create(container);
    lv_obj_set_style_text_font(cyan_state.time_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(cyan_state.time_label, lv_color_hex(0x00FFFF), 0); // Cyan color
    lv_label_set_text(cyan_state.time_label, "00:00:00");
    lv_obj_align(cyan_state.time_label, LV_ALIGN_CENTER, 0, 0);

    // Create button container
    cyan_state.btn_container = lv_obj_create(container);
    lv_obj_set_size(cyan_state.btn_container, lv_pct(100), lv_pct(25));
    lv_obj_align(cyan_state.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(cyan_state.btn_container, LV_OPA_0, 0);
    lv_obj_set_style_pad_all(cyan_state.btn_container, 0, 0);
    lv_obj_set_flex_flow(cyan_state.btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cyan_state.btn_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_border_width(cyan_state.btn_container, 0, 0);
    lv_obj_set_style_pad_all(cyan_state.btn_container, 0, LV_PART_MAIN);

    // Create start/stop button
    cyan_state.btn = lv_btn_create(cyan_state.btn_container);
    lv_obj_set_size(cyan_state.btn, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(cyan_state.btn, lv_color_hex(0x007700), LV_PART_MAIN);
    lv_obj_set_style_radius(cyan_state.btn, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(cyan_state.btn, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(cyan_state.btn, 0, LV_PART_MAIN);
    lv_obj_t *btn_label = lv_label_create(cyan_state.btn);
    lv_label_set_text(btn_label, "START");
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_24, 0);
    lv_obj_center(btn_label);
    lv_obj_add_event_cb(cyan_state.btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    // Create and start the timer for stopwatch updates
    cyan_state.timer = lv_timer_create(update_stopwatch, 1000, NULL);
    lv_timer_pause(cyan_state.timer);

    return container;
}
