#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "tft_config.h"
#include "fb.h"
#include "mcp23017.h"
#include "sd_ota.h"
#include "touch_xpt2046.h"
#include "ui_draw.h"

static const char *TAG = "TFT_FULL_S3";

#define LCD_HOST SPI2_HOST

typedef enum {
    UI_SCREEN_HOME = 0,
    UI_SCREEN_SD_IDLE = 1,
    UI_SCREEN_SD_LIST = 2,
    UI_SCREEN_SETTINGS = 3,
} ui_screen_t;

static void control_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "FFB control task on core %d", xPortGetCoreID());
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void ui_task(void *arg) {
    (void)arg;
    spi_bus_config_t buscfg = {
        .sclk_io_num = TFT_SCLK,
        .mosi_io_num = TFT_MOSI,
        .miso_io_num = TFT_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = TFT_DC,
        .cs_gpio_num = TFT_CS,
        .pclk_hz = TFT_SPI_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = TFT_SPI_MODE,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TFT_RST,
        .rgb_ele_order = CONFIG_TFT_COLOR_ORDER_BGR ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, CONFIG_TFT_INVERT_COLOR);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    fb_init();
    ui_draw_init();

    fb_render_mode_t render_mode = FB_RENDER_DIRECT;
    fb_set_render_mode(render_mode);

#if CONFIG_TFT_TEST_PATTERN
    ui_draw_fill_color(panel_handle, 0);
    ui_draw_test_pattern(panel_handle);
    fb_present(panel_handle);
#endif

#if TFT_TOUCH_ENABLED
    ESP_ERROR_CHECK(touch_init());
#endif

    ESP_LOGI(TAG, "System ready");

#if CONFIG_TFT_OTA_FROM_SDMMC
    bool redraw = true;
    bool sd_ok = false;
#endif
#if TFT_TOUCH_ENABLED && CONFIG_TFT_OTA_FROM_SDMMC
    sd_file_list_t file_list = {0};
    ui_screen_t screen = UI_SCREEN_HOME;
    bool last_pressed = false;
    uint32_t last_touch_ms = 0;
    uint32_t last_touch_time_ms = 0;
    uint16_t last_tx = 0;
    uint16_t last_ty = 0;
    bool touch_seen = false;
    bool ui_full_clear = true;
    uint32_t last_uptime_sec = 0;
    bool encoder_ready = false;
    bool encoder_attempted = false;
    int encoder_last_delta = 0;
    bool encoder_last_pressed = false;
    uint32_t last_encoder_time_ms = 0;
#endif
#if CONFIG_TFT_OTA_FROM_SDMMC && !TFT_TOUCH_ENABLED
    sd_file_list_t encoder_list = {0};
    bool encoder_ready = false;
    bool encoder_ui_list = false;
    int encoder_last_delta = 0;
    bool encoder_last_pressed = false;
    bool ui_full_clear = true;
#endif

    while (1) {
#if TFT_TOUCH_ENABLED && CONFIG_TFT_OTA_TOUCH_TRIGGER && CONFIG_TFT_OTA_FROM_SDMMC
        static uint32_t hold_start_ms = 0;
        uint16_t x = 0;
        uint16_t y = 0;
        bool pressed = touch_get_xy(&x, &y);
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (pressed) {
            last_tx = x;
            last_ty = y;
            touch_seen = true;
            last_touch_time_ms = now_ms;
        }

        if (!encoder_attempted) {
            encoder_attempted = true;
            if (mcp23017_init() == ESP_OK) {
                encoder_ready = true;
            }
        }
        if (encoder_ready) {
            int delta = 0;
            bool enc_pressed = false;
            if (encoder_poll(&delta, &enc_pressed)) {
                if (delta != 0 || enc_pressed) {
                    encoder_last_delta = delta;
                    encoder_last_pressed = enc_pressed;
                    last_encoder_time_ms = now_ms;
                }
            }
        }

        uint32_t uptime_sec = (uint32_t)(now_ms / 1000U);
        if (screen == UI_SCREEN_HOME && uptime_sec != last_uptime_sec) {
            last_uptime_sec = uptime_sec;
            redraw = true;
        }

        if (render_mode == FB_RENDER_FULL) {
            redraw = true;
            ui_full_clear = true;
        }

        if (redraw) {
            if (screen == UI_SCREEN_HOME) {
                ui_draw_main_screen(panel_handle, uptime_sec, ui_full_clear);
            } else if (screen == UI_SCREEN_SD_IDLE) {
                ui_draw_sd_idle(panel_handle, ui_full_clear);
            } else if (screen == UI_SCREEN_SD_LIST) {
                ui_draw_sd_list(panel_handle, &file_list, ui_full_clear);
            } else if (screen == UI_SCREEN_SETTINGS) {
                ui_draw_settings(panel_handle, render_mode, ui_full_clear);
            }
            uint32_t touch_age_ms = touch_seen ? (now_ms - last_touch_time_ms) : 0xFFFFFFFFU;
            uint32_t enc_age_ms = (last_encoder_time_ms > 0) ? (now_ms - last_encoder_time_ms) : 0xFFFFFFFFU;
            ui_draw_status_bar(panel_handle, encoder_ready, sd_ok, touch_seen, last_tx, last_ty, touch_age_ms,
                               encoder_last_delta, encoder_last_pressed, enc_age_ms);
            if (touch_seen && touch_age_ms <= 1000U) {
                ui_draw_touch_cursor(panel_handle, last_tx, last_ty);
            }
            fb_present(panel_handle);
            redraw = false;
            ui_full_clear = false;
        }
        if (fb_take_full_refresh_request()) {
            ui_full_clear = true;
            redraw = true;
        }

        if (pressed) {
            last_touch_ms = now_ms;
        }

        bool tapped = pressed && !last_pressed;
        last_pressed = pressed;

        if (tapped) {
            int tx = x;
            int ty = y;
            bool hit_refresh = (tx >= (LCD_H_RES - 92) && tx <= (LCD_H_RES - 8) && ty >= (LCD_V_RES - 30) && ty <= (LCD_V_RES - 8));

            if (screen == UI_SCREEN_HOME) {
                bool hit_sd_ota = (tx >= 40 && tx <= 200 && ty >= 96 && ty <= 136);
                bool hit_settings = (tx >= 40 && tx <= 200 && ty >= 150 && ty <= 190);
                if (hit_sd_ota) {
                    screen = UI_SCREEN_SD_IDLE;
                    ui_full_clear = true;
                    redraw = true;
                } else if (hit_settings) {
                    screen = UI_SCREEN_SETTINGS;
                    ui_full_clear = true;
                    redraw = true;
                }
            } else if (screen == UI_SCREEN_SETTINGS) {
                bool hit_back = (ty >= LCD_V_RES - 30 && ty <= LCD_V_RES - 8 && tx >= 8 && tx <= 88);
                bool hit_direct = (tx >= 24 && tx <= 216 && ty >= 80 && ty <= 108);
                bool hit_fb = (tx >= 24 && tx <= 216 && ty >= 116 && ty <= 144);
                bool hit_full = (tx >= 24 && tx <= 216 && ty >= 152 && ty <= 180);
                if (hit_back) {
                    screen = UI_SCREEN_HOME;
                    ui_full_clear = true;
                    redraw = true;
                } else if (hit_direct) {
                    render_mode = FB_RENDER_DIRECT;
                    fb_set_render_mode(render_mode);
                    ui_full_clear = true;
                    redraw = true;
                } else if (hit_fb) {
                    render_mode = FB_RENDER_FB;
                    fb_set_render_mode(render_mode);
                    ui_full_clear = true;
                    redraw = true;
                } else if (hit_full) {
                    render_mode = FB_RENDER_FULL;
                    fb_set_render_mode(render_mode);
                    ui_full_clear = true;
                    redraw = true;
                }
            } else if (screen == UI_SCREEN_SD_IDLE) {
                bool hit_back = (ty >= LCD_V_RES - 30 && ty <= LCD_V_RES - 8 && tx >= 8 && tx <= 88);
                bool hit_load = (tx >= 40 && tx <= 200 && ty >= 96 && ty <= 136);
                if (hit_refresh) {
                    fb_request_full_refresh();
                    redraw = true;
                    ui_full_clear = true;
                } else if (hit_back) {
                    screen = UI_SCREEN_HOME;
                    ui_full_clear = true;
                    redraw = true;
                } else if (hit_load) {
                    ESP_LOGI(TAG, "Scanning SD card");
                    if (sd_ota_scan(&file_list) == ESP_OK && file_list.count > 0) {
                        file_list.selected = 0;
                        screen = UI_SCREEN_SD_LIST;
                        sd_ok = true;
                        ui_full_clear = true;
                    } else {
                        ESP_LOGW(TAG, "No .bin files found on SD card");
                        sd_ok = false;
                    }
                    redraw = true;
                }
            } else if (screen == UI_SCREEN_SD_LIST) {
                if (ty >= LCD_V_RES - 30 && ty <= LCD_V_RES - 8 && tx >= 8 && tx <= 88) {
                    screen = UI_SCREEN_SD_IDLE;
                    ui_full_clear = true;
                    redraw = true;
                } else if (hit_refresh) {
                    fb_request_full_refresh();
                    redraw = true;
                    ui_full_clear = true;
                } else {
                    int index = (ty - 52) / 28;
                    if (index >= 0 && index < file_list.count) {
                        file_list.selected = index;
                        redraw = true;
                        char path[128];
                        snprintf(path, sizeof(path), "%s/%s", CONFIG_TFT_SDMMC_MOUNT_POINT, file_list.names[index]);
                        ESP_LOGI(TAG, "Starting OTA: %s", path);
                        sd_ota_start(path);
                    }
                }
            }
        }

        if (pressed && x <= CONFIG_TFT_OTA_TOUCH_ZONE_PX && y <= CONFIG_TFT_OTA_TOUCH_ZONE_PX) {
            if (hold_start_ms == 0) {
                hold_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            } else {
                uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                if ((now_ms - hold_start_ms) >= CONFIG_TFT_OTA_TOUCH_HOLD_MS) {
                    ESP_LOGI(TAG, "Touch hold detected, starting SD card OTA");
                    hold_start_ms = 0;
                    sd_ota_start(CONFIG_TFT_OTA_BIN_PATH);
                }
            }
        } else {
            hold_start_ms = 0;
        }
        if (last_touch_ms == 0) {
            last_touch_ms = now_ms;
        } else {
            if ((now_ms - last_touch_ms) >= 10000) {
                ESP_LOGW(TAG, "No touch detected for 10s. Check wiring and touch CS/MISO.");
                last_touch_ms = now_ms;
            }
        }
#elif CONFIG_TFT_OTA_FROM_SDMMC
        if (!encoder_ready) {
            if (mcp23017_init() == ESP_OK) {
                encoder_ready = true;
                redraw = true;
            } else {
                ESP_LOGW(TAG, "MCP23017 init failed");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        int delta = 0;
        bool pressed = false;
        if (encoder_poll(&delta, &pressed)) {
            encoder_last_delta = delta;
            encoder_last_pressed = pressed;
            if (delta != 0 && encoder_ui_list && encoder_list.count > 0) {
                int next = encoder_list.selected + delta;
                if (next < 0) {
                    next = encoder_list.count - 1;
                } else if (next >= encoder_list.count) {
                    next = 0;
                }
                encoder_list.selected = next;
                redraw = true;
            }

            if (pressed) {
                if (!encoder_ui_list) {
                    ESP_LOGI(TAG, "Scanning SD card");
                    if (sd_ota_scan(&encoder_list) == ESP_OK && encoder_list.count > 0) {
                        encoder_list.selected = 0;
                        encoder_ui_list = true;
                        sd_ok = true;
                        ui_full_clear = true;
                    } else {
                        ESP_LOGW(TAG, "No .bin files found on SD card");
                        sd_ok = false;
                    }
                    redraw = true;
                } else if (encoder_list.count > 0) {
                    char path[128];
                    snprintf(path, sizeof(path), "%s/%s", CONFIG_TFT_SDMMC_MOUNT_POINT, encoder_list.names[encoder_list.selected]);
                    ESP_LOGI(TAG, "Starting OTA: %s", path);
                    sd_ota_start(path);
                }
            }
        }

        if (redraw) {
            if (encoder_ui_list) {
                ui_draw_sd_list(panel_handle, &encoder_list, ui_full_clear);
            } else {
                ui_draw_sd_idle(panel_handle, ui_full_clear);
            }
            uint32_t enc_age_ms = 0xFFFFFFFFU;
            if (encoder_last_pressed || encoder_last_delta != 0) {
                enc_age_ms = 0;
            }
            ui_draw_status_bar(panel_handle, encoder_ready, sd_ok, false, (uint16_t)encoder_last_delta,
                               (uint16_t)(encoder_last_pressed ? 1 : 0), 0xFFFFFFFFU,
                               encoder_last_delta, encoder_last_pressed, enc_age_ms);
            fb_present(panel_handle);
            redraw = false;
            ui_full_clear = false;
        }
        if (fb_take_full_refresh_request()) {
            ui_full_clear = true;
            redraw = true;
        }
#endif

#if CONFIG_TFT_OTA_ON_BOOT && CONFIG_TFT_OTA_FROM_SDMMC
        static bool ota_done = false;
        if (!ota_done) {
            ota_done = true;
            ESP_LOGI(TAG, "Starting SD card OTA on boot");
            sd_ota_start(CONFIG_TFT_OTA_BIN_PATH);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void) {
    xTaskCreatePinnedToCore(control_task, "ffb_ctrl", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(ui_task, "ui_task", 8192, NULL, 5, NULL, 0);
    vTaskDelete(NULL);
}
