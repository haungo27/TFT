#include "ui_draw.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/temperature_sensor.h"
#include "esp_heap_caps.h"
#include "tft_config.h"
#include "fb.h"

#if __has_include("lvgl.h")
#include "lvgl.h"
#define TFT_HAS_LVGL 1
#else
#define TFT_HAS_LVGL 0
#endif

static temperature_sensor_handle_t temp_handle;
static bool temp_ready = false;

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static void temp_sensor_init(void) {
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&cfg, &temp_handle) != ESP_OK) {
        return;
    }
    if (temperature_sensor_enable(temp_handle) != ESP_OK) {
        return;
    }
    temp_ready = true;
}

static bool temp_sensor_read(float *out_celsius) {
    if (!temp_ready) {
        return false;
    }
    return temperature_sensor_get_celsius(temp_handle, out_celsius) == ESP_OK;
}

static int lvgl_fps_avg(void) {
#if TFT_HAS_LVGL && defined(LV_USE_PERF_MONITOR) && LV_USE_PERF_MONITOR
    if (lv_disp_get_default() == NULL) {
        return -1;
    }
    return (int)lv_refr_get_fps_avg();
#else
    return -1;
#endif
}

static int lvgl_cpu_usage_percent(void) {
#if TFT_HAS_LVGL && defined(LV_USE_PERF_MONITOR) && LV_USE_PERF_MONITOR
    if (lv_disp_get_default() == NULL) {
        return -1;
    }
    uint32_t idle = lv_timer_get_idle();
    if (idle > 100) {
        idle = 100;
    }
    return (int)(100 - idle);
#else
    return -1;
#endif
}

static int cpu_usage_percent(void) {
#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1) && (INCLUDE_xTaskGetSystemState == 1)
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    TaskStatus_t *tasks = (TaskStatus_t *)pvPortMalloc(task_count * sizeof(TaskStatus_t));
    if (!tasks) {
        return -1;
    }
    uint32_t total_time = 0;
    UBaseType_t count = uxTaskGetSystemState(tasks, task_count, &total_time);
    uint32_t idle_time = 0;
    for (UBaseType_t i = 0; i < count; ++i) {
        if (strncmp(tasks[i].pcTaskName, "IDLE", 4) == 0) {
            idle_time += tasks[i].ulRunTimeCounter;
        }
    }
    vPortFree(tasks);
    if (total_time == 0) {
        return -1;
    }
    int idle_pct = (int)((idle_time * 100U) / total_time);
    int used_pct = 100 - idle_pct;
    if (used_pct < 0) used_pct = 0;
    if (used_pct > 100) used_pct = 100;
    return used_pct;
#else
    return -1;
#endif
}

static void to_upper(char *dst, const char *src, size_t max_len) {
    size_t i = 0;
    for (; i + 1 < max_len && src[i] != '\0'; ++i) {
        char c = src[i];
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - 'a' + 'A');
        }
        dst[i] = c;
    }
    dst[i] = '\0';
}

static void draw_rect(esp_lcd_panel_handle_t panel_handle, int x0, int y0, int w, int h, uint16_t color) {
    if (fb_is_enabled()) {
        fb_draw_fill_rect(x0, y0, w, h, color);
        return;
    }
    static uint16_t line[LCD_H_RES];
    if (w <= 0 || h <= 0) {
        return;
    }
    if (w > LCD_H_RES) {
        w = LCD_H_RES;
    }
    for (int x = 0; x < w; ++x) {
        line[x] = color;
    }
    for (int y = y0; y < y0 + h; ++y) {
        esp_lcd_panel_draw_bitmap(panel_handle, x0, y, x0 + w, y + 1, line);
    }
}

static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},
    {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},
    {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14},
    {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},
    {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},
    {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},
    {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},
    {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},
    {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},
    {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},
    {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},
    {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63},
    {0x03,0x04,0x78,0x04,0x03},
    {0x61,0x51,0x49,0x45,0x43},
    {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},
    {0x00,0x41,0x41,0x7F,0x00},
    {0x04,0x02,0x01,0x02,0x04},
    {0x80,0x80,0x80,0x80,0x80},
    {0x00,0x03,0x05,0x00,0x00},
};

static void draw_char(esp_lcd_panel_handle_t panel_handle, int x, int y, char c, uint16_t fg, uint16_t bg) {
    if (c < 32 || c > 96) {
        c = '?';
    }
    const uint8_t *glyph = font5x7[c - 32];
    uint16_t buffer[6 * 8];
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 6; ++col) {
            uint16_t color = bg;
            if (col < 5 && (glyph[col] & (1 << row))) {
                color = fg;
            }
            buffer[row * 6 + col] = color;
        }
    }
    if (fb_is_enabled()) {
        fb_draw_blit(x, y, 6, 8, buffer);
    } else {
        esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 6, y + 8, buffer);
    }
}

static void draw_text(esp_lcd_panel_handle_t panel_handle, int x, int y, const char *text, uint16_t fg, uint16_t bg) {
    int cursor = x;
    char upper[128];
    to_upper(upper, text, sizeof(upper));
    for (size_t i = 0; upper[i] != '\0'; ++i) {
        draw_char(panel_handle, cursor, y, upper[i], fg, bg);
        cursor += 6;
    }
}

static void draw_button(esp_lcd_panel_handle_t panel_handle, int x, int y, int w, int h, const char *label, bool active) {
    uint16_t bg = active ? rgb565(0, 120, 200) : rgb565(20, 20, 20);
    uint16_t fg = rgb565(240, 240, 240);
    draw_rect(panel_handle, x, y, w, h, bg);
    int text_x = x + 6;
    int text_y = y + (h - 8) / 2;
    draw_text(panel_handle, text_x, text_y, label, fg, bg);
}

void ui_draw_init(void) {
    temp_sensor_init();
}

void ui_draw_fill_color(esp_lcd_panel_handle_t panel_handle, uint16_t color) {
    if (fb_is_enabled()) {
        fb_draw_fill_rect(0, 0, LCD_H_RES, LCD_V_RES, color);
        return;
    }
    static uint16_t line[LCD_H_RES];
    for (int x = 0; x < LCD_H_RES; ++x) {
        line[x] = color;
    }

    for (int y = 0; y < LCD_V_RES; ++y) {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y + 1, line);
    }
}

void ui_draw_test_pattern(esp_lcd_panel_handle_t panel_handle) {
    if (fb_is_enabled()) {
        uint16_t *fb = fb_get_draw_buffer();
        if (fb) {
            for (int y = 0; y < LCD_V_RES; ++y) {
                uint16_t color = rgb565(0, 0, 0);
                if (y < (LCD_V_RES / 3)) {
                    color = rgb565(255, 0, 0);
                } else if (y < (2 * LCD_V_RES / 3)) {
                    color = rgb565(0, 255, 0);
                } else {
                    color = rgb565(0, 0, 255);
                }
                uint16_t *row = fb + y * LCD_H_RES;
                for (int x = 0; x < LCD_H_RES; ++x) {
                    row[x] = color;
                }
            }
        }
    } else {
        static uint16_t line[LCD_H_RES];
        for (int y = 0; y < LCD_V_RES; ++y) {
            uint16_t color = rgb565(0, 0, 0);
            if (y < (LCD_V_RES / 3)) {
                color = rgb565(255, 0, 0);
            } else if (y < (2 * LCD_V_RES / 3)) {
                color = rgb565(0, 255, 0);
            } else {
                color = rgb565(0, 0, 255);
            }
            for (int x = 0; x < LCD_H_RES; ++x) {
                line[x] = color;
            }
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y + 1, line);
        }
    }

    enum { FLOWER_SIZE = 80 };
    static uint16_t flower[FLOWER_SIZE * FLOWER_SIZE];
    uint16_t background = rgb565(10, 10, 10);
    uint16_t petal = rgb565(255, 105, 180);
    uint16_t center = rgb565(255, 215, 0);

    for (int i = 0; i < FLOWER_SIZE * FLOWER_SIZE; ++i) {
        flower[i] = background;
    }

    int cx = FLOWER_SIZE / 2;
    int cy = FLOWER_SIZE / 2;
    int r = 14;
    int petal_r = 12;
    int offsets[5][2] = {
        {0, -22},
        {21, -7},
        {13, 20},
        {-13, 20},
        {-21, -7},
    };

    for (int p = 0; p < 5; ++p) {
        int pcx = cx + offsets[p][0];
        int pcy = cy + offsets[p][1];
        for (int y = 0; y < FLOWER_SIZE; ++y) {
            for (int x = 0; x < FLOWER_SIZE; ++x) {
                int dx = x - pcx;
                int dy = y - pcy;
                if ((dx * dx + dy * dy) <= (petal_r * petal_r)) {
                    flower[y * FLOWER_SIZE + x] = petal;
                }
            }
        }
    }

    for (int y = 0; y < FLOWER_SIZE; ++y) {
        for (int x = 0; x < FLOWER_SIZE; ++x) {
            int dx = x - cx;
            int dy = y - cy;
            if ((dx * dx + dy * dy) <= (r * r)) {
                flower[y * FLOWER_SIZE + x] = center;
            }
        }
    }

    int x0 = (LCD_H_RES - FLOWER_SIZE) / 2;
    int y0 = (LCD_V_RES - FLOWER_SIZE) / 2;
    if (fb_is_enabled()) {
        fb_draw_blit(x0, y0, FLOWER_SIZE, FLOWER_SIZE, flower);
    } else {
        esp_lcd_panel_draw_bitmap(panel_handle, x0, y0, x0 + FLOWER_SIZE, y0 + FLOWER_SIZE, flower);
    }
}

void ui_draw_main_screen(esp_lcd_panel_handle_t panel_handle, uint32_t uptime_sec, bool full_clear) {
    const int header_y = 36;
    if (full_clear) {
        ui_draw_fill_color(panel_handle, rgb565(0, 0, 0));
    } else {
        draw_rect(panel_handle, 0, 32, LCD_H_RES, LCD_V_RES - 32, rgb565(0, 0, 0));
    }

    draw_text(panel_handle, 8, header_y, "MAIN", rgb565(255, 255, 255), rgb565(0, 0, 0));

    uint32_t hours = uptime_sec / 3600U;
    uint32_t minutes = (uptime_sec % 3600U) / 60U;
    uint32_t seconds = uptime_sec % 60U;
    char time_buf[32];
    snprintf(time_buf, sizeof(time_buf), "UPTIME %02lu:%02lu:%02lu", (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
    draw_text(panel_handle, 8, header_y + 20, time_buf, rgb565(180, 220, 255), rgb565(0, 0, 0));

    draw_button(panel_handle, 40, 96, 160, 40, "SD OTA", false);
    draw_button(panel_handle, 40, 150, 160, 40, "SETTINGS", false);
}

void ui_draw_settings(esp_lcd_panel_handle_t panel_handle, fb_render_mode_t mode, bool full_clear) {
    const int header_y = 36;
    if (full_clear) {
        ui_draw_fill_color(panel_handle, rgb565(0, 0, 0));
    } else {
        draw_rect(panel_handle, 0, 32, LCD_H_RES, LCD_V_RES - 32, rgb565(0, 0, 0));
    }

    draw_text(panel_handle, 8, header_y, "SETTINGS", rgb565(255, 255, 255), rgb565(0, 0, 0));
    draw_text(panel_handle, 8, header_y + 14, "RENDER MODE", rgb565(180, 180, 180), rgb565(0, 0, 0));

    draw_button(panel_handle, 24, 80, 192, 28, "DIRECT DRAW", mode == FB_RENDER_DIRECT);
    draw_button(panel_handle, 24, 116, 192, 28, "FB + DMA", mode == FB_RENDER_FB);
    draw_button(panel_handle, 24, 152, 192, 28, "FULL REFRESH", mode == FB_RENDER_FULL);
    draw_button(panel_handle, 8, LCD_V_RES - 30, 80, 22, "BACK", false);
}

void ui_draw_sd_list(esp_lcd_panel_handle_t panel_handle, const sd_file_list_t *list, bool full_clear) {
    const int refresh_x = LCD_H_RES - 92;
    const int refresh_y = LCD_V_RES - 30;
    const int refresh_w = 84;
    const int refresh_h = 22;
    const int header_y = 36;
    const int list_y0 = 52;
    if (full_clear) {
        ui_draw_fill_color(panel_handle, rgb565(0, 0, 0));
    } else {
        draw_rect(panel_handle, 0, 32, LCD_H_RES, LCD_V_RES - 32, rgb565(0, 0, 0));
    }
    draw_text(panel_handle, 8, header_y, "SELECT OTA FILE", rgb565(255, 255, 255), rgb565(0, 0, 0));
    int y = list_y0;
    for (int i = 0; i < list->count; ++i) {
        bool active = (i == list->selected);
        draw_button(panel_handle, 8, y, LCD_H_RES - 16, 24, list->names[i], active);
        y += 28;
    }
    draw_button(panel_handle, 8, LCD_V_RES - 30, 80, 22, "BACK", false);
    draw_button(panel_handle, refresh_x, refresh_y, refresh_w, refresh_h, "REFRESH", false);
}

void ui_draw_sd_idle(esp_lcd_panel_handle_t panel_handle, bool full_clear) {
    const int refresh_x = LCD_H_RES - 92;
    const int refresh_y = LCD_V_RES - 30;
    const int refresh_w = 84;
    const int refresh_h = 22;
    const int header_y = 36;
    if (full_clear) {
        ui_draw_fill_color(panel_handle, rgb565(0, 0, 0));
    } else {
        draw_rect(panel_handle, 0, 32, LCD_H_RES, LCD_V_RES - 32, rgb565(0, 0, 0));
    }
    draw_text(panel_handle, 8, header_y, "SD OTA", rgb565(255, 255, 255), rgb565(0, 0, 0));
    draw_text(panel_handle, 8, header_y + 12, "TAP TO SCAN", rgb565(180, 180, 180), rgb565(0, 0, 0));
    draw_button(panel_handle, 40, 96, 160, 40, "LOAD SD CARD", false);
    draw_button(panel_handle, 8, LCD_V_RES - 30, 80, 22, "BACK", false);
    draw_button(panel_handle, refresh_x, refresh_y, refresh_w, refresh_h, "REFRESH", false);
}

void ui_draw_status_bar(esp_lcd_panel_handle_t panel_handle, bool mcp_ok, bool sd_ok, bool touch_ok,
                        uint16_t tx, uint16_t ty, uint32_t touch_age_ms,
                        int enc_delta, bool enc_pressed, uint32_t enc_age_ms) {
    uint16_t bg = rgb565(8, 8, 12);
    uint16_t fg = rgb565(220, 220, 220);
    char line[96];
    int cpu = cpu_usage_percent();
    int lvgl_cpu = lvgl_cpu_usage_percent();
    int fps = lvgl_fps_avg();
#ifdef CONFIG_SPIRAM
    uint32_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
#else
    uint32_t free_psram = 0;
#endif
    float temp_c = 0.0f;
    bool temp_ok = temp_sensor_read(&temp_c);
    char cpu_buf[12];
    char lvgl_buf[12];
    char fps_buf[12];
    char temp_buf[12];

    if (cpu >= 0) {
        snprintf(cpu_buf, sizeof(cpu_buf), "%d", cpu);
    } else {
        snprintf(cpu_buf, sizeof(cpu_buf), "--");
    }
    if (lvgl_cpu >= 0) {
        snprintf(lvgl_buf, sizeof(lvgl_buf), "%d", lvgl_cpu);
    } else {
        snprintf(lvgl_buf, sizeof(lvgl_buf), "--");
    }
    if (fps >= 0) {
        snprintf(fps_buf, sizeof(fps_buf), "%d", fps);
    } else {
        snprintf(fps_buf, sizeof(fps_buf), "--");
    }
    if (temp_ok) {
        snprintf(temp_buf, sizeof(temp_buf), "%d", (int)(temp_c + 0.5f));
    } else {
        snprintf(temp_buf, sizeof(temp_buf), "--");
    }

    draw_rect(panel_handle, 0, 0, LCD_H_RES, 16, bg);
    snprintf(line, sizeof(line), "M:%s S:%s T:%s CPU:%s%% L:%s%%", mcp_ok ? "OK" : "--", sd_ok ? "OK" : "--", touch_ok ? "OK" : "--", cpu_buf, lvgl_buf);
    draw_text(panel_handle, 4, 4, line, fg, bg);

    draw_rect(panel_handle, 0, 16, LCD_H_RES, 16, bg);
    if (touch_ok && touch_age_ms <= 1000U) {
        snprintf(line, sizeof(line), "FPS:%s PS:%luk XY:%u,%u", fps_buf, (unsigned long)free_psram, (unsigned)tx, (unsigned)ty);
    } else if (enc_age_ms <= 1000U) {
        snprintf(line, sizeof(line), "FPS:%s PS:%luk ENC:%d%s", fps_buf, (unsigned long)free_psram, enc_delta, enc_pressed ? "*" : "");
    } else {
        snprintf(line, sizeof(line), "FPS:%s PS:%luk TMP:%sC", fps_buf, (unsigned long)free_psram, temp_buf);
    }
    draw_text(panel_handle, 4, 20, line, fg, bg);
}

void ui_draw_touch_cursor(esp_lcd_panel_handle_t panel_handle, uint16_t x, uint16_t y) {
    const uint16_t color = rgb565(255, 240, 120);
    const int size = 6;
    int left = (int)x - size;
    int right = (int)x + size;
    int top = (int)y - size;
    int bottom = (int)y + size;

    if (left < 0) left = 0;
    if (right >= LCD_H_RES) right = LCD_H_RES - 1;
    if (top < 0) top = 0;
    if (bottom >= LCD_V_RES) bottom = LCD_V_RES - 1;

    draw_rect(panel_handle, left, (int)y, (right - left) + 1, 1, color);
    draw_rect(panel_handle, (int)x, top, 1, (bottom - top) + 1, color);
}
