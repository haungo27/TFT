#include "fb.h"

#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tft_config.h"

static const char *TAG = "TFT_FB";

static uint16_t *fb_draw = NULL;
static uint16_t *fb_show = NULL;
static uint16_t *fb_dma = NULL;
static bool fb_enabled = false;
static volatile bool fb_force_full_refresh = false;
static fb_render_mode_t fb_render_mode = FB_RENDER_FB;
static esp_lcd_panel_io_handle_t fb_io_handle = NULL;
static SemaphoreHandle_t fb_tx_sem = NULL;
static bool fb_tx_cb_installed = false;

static bool fb_on_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    (void)panel_io;
    (void)edata;
    (void)user_ctx;
    BaseType_t high_task_wakeup = pdFALSE;
    if (fb_tx_sem != NULL) {
        xSemaphoreGiveFromISR(fb_tx_sem, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

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

void fb_set_io_handle(esp_lcd_panel_io_handle_t io_handle) {
    fb_io_handle = io_handle;
    if (fb_io_handle == NULL || fb_tx_cb_installed) {
        return;
    }
    if (fb_tx_sem == NULL) {
        fb_tx_sem = xSemaphoreCreateBinary();
    }
    if (fb_tx_sem == NULL) {
        ESP_LOGW(TAG, "FB tx semaphore alloc failed");
        return;
    }

    esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = fb_on_color_trans_done,
    };
    if (esp_lcd_panel_io_register_event_callbacks(fb_io_handle, &cbs, NULL) == ESP_OK) {
        fb_tx_cb_installed = true;
    }
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
        if (fb_tx_sem != NULL && fb_tx_cb_installed) {
            xSemaphoreTake(fb_tx_sem, portMAX_DELAY);
        }
    }
    uint16_t *tmp = fb_show;
    fb_show = fb_draw;
    fb_draw = tmp;
}
