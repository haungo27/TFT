#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_lcd_panel_ops.h"
#include "sd_ota.h"
#include "fb.h"

void ui_draw_init(void);
void ui_draw_fill_color(esp_lcd_panel_handle_t panel_handle, uint16_t color);
void ui_draw_test_pattern(esp_lcd_panel_handle_t panel_handle);
void ui_draw_main_screen(esp_lcd_panel_handle_t panel_handle, uint32_t uptime_sec, bool full_clear);
void ui_draw_settings(esp_lcd_panel_handle_t panel_handle, fb_render_mode_t mode, bool full_clear);
void ui_draw_sd_list(esp_lcd_panel_handle_t panel_handle, const sd_file_list_t *list, bool full_clear);
void ui_draw_sd_idle(esp_lcd_panel_handle_t panel_handle, bool full_clear);
void ui_draw_status_bar(esp_lcd_panel_handle_t panel_handle, bool mcp_ok, bool sd_ok, bool touch_ok,
						uint16_t tx, uint16_t ty, uint32_t touch_age_ms,
						int enc_delta, bool enc_pressed, uint32_t enc_age_ms);

void ui_draw_touch_cursor(esp_lcd_panel_handle_t panel_handle, uint16_t x, uint16_t y);
