#include "fb.h"

#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "tft_config.h"

static const char *TAG = "TFT_FB";

static uint16_t *fb_draw = NULL;
static uint16_t *fb_show = NULL;
static uint16_t *fb_dma = NULL;
static bool fb_enabled = false;
static volatile bool fb_force_full_refresh = false;
static fb_render_mode_t fb_render_mode = FB_RENDER_FB;

static void fb_fill_rect(uint16_t *fb, int x0, int y0, int w, int h, uint16_t color) {
    if (w <= 0 || h <= 0) {
        return;
    }
    if (x0 < 0) {
        w += x0;
        x0 = 0;
    }
    if (y0 < 0) {
        h += y0;
        y0 = 0;
    }
    if (x0 + w > LCD_H_RES) {
        w = LCD_H_RES - x0;
    }
    if (y0 + h > LCD_V_RES) {
        h = LCD_V_RES - y0;
    }
    if (w <= 0 || h <= 0) {
        return;
    }
    for (int y = y0; y < y0 + h; ++y) {
        uint16_t *row = fb + y * LCD_H_RES + x0;
        for (int x = 0; x < w; ++x) {
            row[x] = color;
        }
    }
}

static void fb_blit(uint16_t *fb, int x0, int y0, int w, int h, const uint16_t *src) {
    if (w <= 0 || h <= 0) {
        return;
    }
    if (x0 < 0 || y0 < 0 || x0 + w > LCD_H_RES || y0 + h > LCD_V_RES) {
        return;
    }
    for (int y = 0; y < h; ++y) {
        memcpy(fb + (y0 + y) * LCD_H_RES + x0, src + y * w, (size_t)w * sizeof(uint16_t));
    }
}

void fb_init(void) {
    size_t fb_size = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
    fb_draw = (uint16_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    fb_show = (uint16_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    fb_dma = (uint16_t *)heap_caps_malloc(LCD_H_RES * FB_LINES * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    if (!fb_draw || !fb_show || !fb_dma) {
        if (fb_draw) {
            heap_caps_free(fb_draw);
            fb_draw = NULL;
        }
        if (fb_show) {
            heap_caps_free(fb_show);
            fb_show = NULL;
        }
        if (fb_dma) {
            heap_caps_free(fb_dma);
            fb_dma = NULL;
        }
        fb_enabled = false;
        ESP_LOGW(TAG, "FB init failed, using direct draw");
        return;
    }

    fb_enabled = true;
    ESP_LOGI(TAG, "FB init ok: PSRAM double buffer + DMA line buffer");
}

bool fb_is_enabled(void) {
    return fb_enabled && fb_render_mode != FB_RENDER_DIRECT;
}

void fb_set_render_mode(fb_render_mode_t mode) {
    fb_render_mode = mode;
}

fb_render_mode_t fb_get_render_mode(void) {
    return fb_render_mode;
}

void fb_request_full_refresh(void) {
    fb_force_full_refresh = true;
}

bool fb_take_full_refresh_request(void) {
    if (!fb_force_full_refresh) {
        return false;
    }
    fb_force_full_refresh = false;
    return true;
}

uint16_t *fb_get_draw_buffer(void) {
    if (!fb_is_enabled()) {
        return NULL;
    }
    return fb_draw;
}

void fb_draw_fill_rect(int x0, int y0, int w, int h, uint16_t color) {
    if (!fb_is_enabled()) {
        return;
    }
    fb_fill_rect(fb_draw, x0, y0, w, h, color);
}

void fb_draw_blit(int x0, int y0, int w, int h, const uint16_t *src) {
    if (!fb_is_enabled()) {
        return;
    }
    fb_blit(fb_draw, x0, y0, w, h, src);
}

void fb_present(esp_lcd_panel_handle_t panel_handle) {
    if (!fb_is_enabled()) {
        return;
    }
    for (int y = 0; y < LCD_V_RES; y += FB_LINES) {
        int h = LCD_V_RES - y;
        if (h > FB_LINES) {
            h = FB_LINES;
        }
        memcpy(fb_dma, fb_draw + y * LCD_H_RES, (size_t)LCD_H_RES * h * sizeof(uint16_t));
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y + h, fb_dma);
    }
    uint16_t *tmp = fb_show;
    fb_show = fb_draw;
    fb_draw = tmp;
}
