#include "touch_xpt2046.h"

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "tft_config.h"

#define TOUCH_HOST SPI3_HOST

static const char *TAG = "TFT_TOUCH";
static spi_device_handle_t touch_dev;

static esp_err_t xpt2046_read_cmd(uint8_t cmd, uint16_t *value) {
    uint8_t tx[3] = {cmd, 0x00, 0x00};
    uint8_t rx[3] = {0};
    spi_transaction_t t = {
        .length = 3 * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(touch_dev, &t);
    if (err != ESP_OK) {
        return err;
    }
    *value = (uint16_t)(((rx[1] << 8) | rx[2]) >> 3);
    return ESP_OK;
}

static bool xpt2046_read_raw(uint16_t *raw_x, uint16_t *raw_y, uint16_t *raw_z) {
    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t z1 = 0;
    if (xpt2046_read_cmd(0xD0, &x) != ESP_OK) {
        return false;
    }
    if (xpt2046_read_cmd(0x90, &y) != ESP_OK) {
        return false;
    }
    if (xpt2046_read_cmd(0xB0, &z1) != ESP_OK) {
        return false;
    }
    *raw_x = x;
    *raw_y = y;
    *raw_z = z1;
    return true;
}

esp_err_t touch_init(void) {
    spi_bus_config_t buscfg = {
        .sclk_io_num = TFT_TOUCH_SPI_SCLK,
        .mosi_io_num = TFT_TOUCH_SPI_MOSI,
        .miso_io_num = TFT_TOUCH_SPI_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 3,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = TFT_TOUCH_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = TFT_TOUCH_CS,
        .queue_size = 1,
    };
    ESP_LOGI(TAG, "Touch SPI: sclk=%d mosi=%d miso=%d cs=%d", TFT_TOUCH_SPI_SCLK, TFT_TOUCH_SPI_MOSI, TFT_TOUCH_SPI_MISO, TFT_TOUCH_CS);
    return spi_bus_add_device(TOUCH_HOST, &devcfg, &touch_dev);
}

bool touch_get_xy(uint16_t *x, uint16_t *y) {
    uint16_t rx = 0;
    uint16_t ry = 0;
    uint16_t rz = 0;

    if (!xpt2046_read_raw(&rx, &ry, &rz)) {
        return false;
    }
    if (rz < TFT_TOUCH_Z_THRESHOLD) {
        return false;
    }

    uint16_t raw_x = rx;
    uint16_t raw_y = ry;
    if (TFT_TOUCH_SWAP_XY) {
        uint16_t tmp = raw_x;
        raw_x = raw_y;
        raw_y = tmp;
    }

    int32_t mapped_x = (int32_t)(raw_x - TFT_TOUCH_X_MIN) * LCD_H_RES / (TFT_TOUCH_X_MAX - TFT_TOUCH_X_MIN);
    int32_t mapped_y = (int32_t)(raw_y - TFT_TOUCH_Y_MIN) * LCD_V_RES / (TFT_TOUCH_Y_MAX - TFT_TOUCH_Y_MIN);

    if (mapped_x < 0) mapped_x = 0;
    if (mapped_y < 0) mapped_y = 0;
    if (mapped_x >= LCD_H_RES) mapped_x = LCD_H_RES - 1;
    if (mapped_y >= LCD_V_RES) mapped_y = LCD_V_RES - 1;

    if (TFT_TOUCH_MIRROR_X) {
        mapped_x = (LCD_H_RES - 1) - mapped_x;
    }
    if (TFT_TOUCH_MIRROR_Y) {
        mapped_y = (LCD_V_RES - 1) - mapped_y;
    }

    *x = (uint16_t)mapped_x;
    *y = (uint16_t)mapped_y;
    return true;
}
