idf_component_register(SRCS "app_main.c" "fb.c" "mcp23017.c" "sd_ota.c" "touch_xpt2046.c" "ui_draw.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_lcd app_update fatfs sdmmc esp_driver_uart esp_driver_i2c esp_driver_tsens lvgl)