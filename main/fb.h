#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

typedef enum {
	FB_RENDER_DIRECT = 0,
	FB_RENDER_FB = 1,
	FB_RENDER_FULL = 2,
} fb_render_mode_t;

void fb_init(void);
bool fb_is_enabled(void);
void fb_set_render_mode(fb_render_mode_t mode);
fb_render_mode_t fb_get_render_mode(void);
void fb_set_io_handle(esp_lcd_panel_io_handle_t io_handle);
void fb_request_full_refresh(void);
bool fb_take_full_refresh_request(void);
uint16_t *fb_get_draw_buffer(void);
void fb_draw_fill_rect(int x0, int y0, int w, int h, uint16_t color);
void fb_draw_blit(int x0, int y0, int w, int h, const uint16_t *src);
void fb_present(esp_lcd_panel_handle_t panel_handle);
