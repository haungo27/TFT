idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_lcd app_update fatfs sdmmc esp_driver_uart esp_driver_i2c)