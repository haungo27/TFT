#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include <dirent.h>
#include <string.h>
#include <stdbool.h>

#ifndef CONFIG_TFT_SPI_MODE
#define CONFIG_TFT_SPI_MODE 0
#endif
#ifndef CONFIG_TFT_COLOR_ORDER_BGR
#define CONFIG_TFT_COLOR_ORDER_BGR 0
#endif
#ifndef CONFIG_TFT_INVERT_COLOR
#define CONFIG_TFT_INVERT_COLOR 0
#endif
#ifndef CONFIG_TFT_TEST_PATTERN
#define CONFIG_TFT_TEST_PATTERN 1
#endif
#ifndef CONFIG_TFT_I2C_SDA
#define CONFIG_TFT_I2C_SDA 4
#endif
#ifndef CONFIG_TFT_I2C_SCL
#define CONFIG_TFT_I2C_SCL 17
#endif
#ifndef CONFIG_TFT_I2C_FREQ_HZ
#define CONFIG_TFT_I2C_FREQ_HZ 400000
#endif
#ifndef CONFIG_TFT_MCP23017_ADDR
#define CONFIG_TFT_MCP23017_ADDR 0x20
#endif
#ifndef CONFIG_TFT_MCP23017_RST
#define CONFIG_TFT_MCP23017_RST 18
#endif
#ifndef CONFIG_TFT_ENCODER_PIN_A
#define CONFIG_TFT_ENCODER_PIN_A 1
#endif
#ifndef CONFIG_TFT_ENCODER_PIN_B
#define CONFIG_TFT_ENCODER_PIN_B 2
#endif
#ifndef CONFIG_TFT_ENCODER_PIN_SW
#define CONFIG_TFT_ENCODER_PIN_SW 3
#endif
#ifndef CONFIG_TFT_SDMMC_MOUNT_POINT
#define CONFIG_TFT_SDMMC_MOUNT_POINT "/sdcard"
#endif
#ifndef CONFIG_TFT_OTA_BIN_PATH
#define CONFIG_TFT_OTA_BIN_PATH "/sdcard/firmware.bin"
#endif
#ifndef CONFIG_TFT_SDMMC_FREQ_KHZ
#define CONFIG_TFT_SDMMC_FREQ_KHZ 20000
#endif
#ifndef CONFIG_TFT_TOUCH_SPI_MOSI
#define CONFIG_TFT_TOUCH_SPI_MOSI 6
#endif
#ifndef CONFIG_TFT_TOUCH_SPI_MISO
#define CONFIG_TFT_TOUCH_SPI_MISO 5
#endif
#ifndef CONFIG_TFT_TOUCH_SPI_SCLK
#define CONFIG_TFT_TOUCH_SPI_SCLK 7
#endif
#ifndef CONFIG_TFT_TOUCH_X_MIN
#define CONFIG_TFT_TOUCH_X_MIN 200
#endif
#ifndef CONFIG_TFT_TOUCH_X_MAX
#define CONFIG_TFT_TOUCH_X_MAX 3800
#endif
#ifndef CONFIG_TFT_TOUCH_Y_MIN
#define CONFIG_TFT_TOUCH_Y_MIN 200
#endif
#ifndef CONFIG_TFT_TOUCH_Y_MAX
#define CONFIG_TFT_TOUCH_Y_MAX 3800
#endif
#ifndef CONFIG_TFT_TOUCH_SWAP_XY
#define CONFIG_TFT_TOUCH_SWAP_XY 0
#endif
#ifndef CONFIG_TFT_TOUCH_MIRROR_X
#define CONFIG_TFT_TOUCH_MIRROR_X 0
#endif
#ifndef CONFIG_TFT_TOUCH_MIRROR_Y
#define CONFIG_TFT_TOUCH_MIRROR_Y 0
#endif
#ifndef CONFIG_TFT_TOUCH_Z_THRESHOLD
#define CONFIG_TFT_TOUCH_Z_THRESHOLD 50
#endif

static const char *TAG = "TFT_FULL_S3";

// --- SPI pin configuration for ESP32-S3 ---
#define LCD_HOST       SPI2_HOST
#define TFT_MOSI       CONFIG_TFT_SPI_MOSI
#define TFT_SCLK       CONFIG_TFT_SPI_SCLK
#define TFT_CS         CONFIG_TFT_SPI_CS
#define TFT_DC         CONFIG_TFT_SPI_DC
#define TFT_RST        CONFIG_TFT_SPI_RST
#define TOUCH_CS       CONFIG_TFT_TOUCH_CS
#define TFT_MISO       CONFIG_TFT_SPI_MISO
#define TFT_SPI_CLOCK_HZ CONFIG_TFT_SPI_CLOCK_HZ
#define TFT_TOUCH_SPI_CLOCK_HZ CONFIG_TFT_TOUCH_SPI_CLOCK_HZ
#define TFT_SDMMC_CLK_PIN CONFIG_TFT_SDMMC_CLK_PIN
#define TFT_SDMMC_CMD_PIN CONFIG_TFT_SDMMC_CMD_PIN
#define TFT_SDMMC_D0_PIN  CONFIG_TFT_SDMMC_D0_PIN
#define TFT_SDMMC_FREQ_KHZ CONFIG_TFT_SDMMC_FREQ_KHZ
#define TFT_SPI_MODE CONFIG_TFT_SPI_MODE
#define TFT_TOUCH_SPI_MOSI CONFIG_TFT_TOUCH_SPI_MOSI
#define TFT_TOUCH_SPI_MISO CONFIG_TFT_TOUCH_SPI_MISO
#define TFT_TOUCH_SPI_SCLK CONFIG_TFT_TOUCH_SPI_SCLK
#define TFT_TOUCH_X_MIN CONFIG_TFT_TOUCH_X_MIN
#define TFT_TOUCH_X_MAX CONFIG_TFT_TOUCH_X_MAX
#define TFT_TOUCH_Y_MIN CONFIG_TFT_TOUCH_Y_MIN
#define TFT_TOUCH_Y_MAX CONFIG_TFT_TOUCH_Y_MAX
#define TFT_TOUCH_SWAP_XY CONFIG_TFT_TOUCH_SWAP_XY
#define TFT_TOUCH_MIRROR_X CONFIG_TFT_TOUCH_MIRROR_X
#define TFT_TOUCH_MIRROR_Y CONFIG_TFT_TOUCH_MIRROR_Y
#define TFT_TOUCH_Z_THRESHOLD CONFIG_TFT_TOUCH_Z_THRESHOLD
#define TFT_I2C_SDA CONFIG_TFT_I2C_SDA
#define TFT_I2C_SCL CONFIG_TFT_I2C_SCL
#define TFT_I2C_FREQ_HZ CONFIG_TFT_I2C_FREQ_HZ
#define TFT_MCP23017_ADDR CONFIG_TFT_MCP23017_ADDR
#define TFT_MCP23017_RST CONFIG_TFT_MCP23017_RST
#define TFT_ENCODER_PIN_A CONFIG_TFT_ENCODER_PIN_A
#define TFT_ENCODER_PIN_B CONFIG_TFT_ENCODER_PIN_B
#define TFT_ENCODER_PIN_SW CONFIG_TFT_ENCODER_PIN_SW

#define TFT_TOUCH_ENABLED CONFIG_TFT_ENABLE_TOUCH

#if CONFIG_TFT_OTA_FROM_SDMMC
typedef struct {
    int count;
    int selected;
    char names[8][64];
} sd_file_list_t;

static bool ends_with_bin(const char *name) {
    size_t len = strlen(name);
    if (len < 4) {
        return false;
    }
    return (name[len - 4] == '.') && ((name[len - 3] | 0x20) == 'b') && ((name[len - 2] | 0x20) == 'i') && ((name[len - 1] | 0x20) == 'n');
}
#endif

// Kích thước màn hình
#define LCD_H_RES      240
#define LCD_V_RES      320

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

#if CONFIG_TFT_ENABLE_TOUCH
#define TOUCH_HOST SPI3_HOST
static spi_device_handle_t touch_dev;

static esp_err_t touch_spi_init(void) {
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
        .spics_io_num = TOUCH_CS,
        .queue_size = 1,
    };
    return spi_bus_add_device(TOUCH_HOST, &devcfg, &touch_dev);
}

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

static bool touch_get_xy(uint16_t *x, uint16_t *y) {
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
#endif

static void fill_color(esp_lcd_panel_handle_t panel_handle, uint16_t color) {
    static uint16_t line[LCD_H_RES];
    for (int x = 0; x < LCD_H_RES; ++x) {
        line[x] = color;
    }

    for (int y = 0; y < LCD_V_RES; ++y) {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y + 1, line);
    }
}

static void draw_test_pattern(esp_lcd_panel_handle_t panel_handle) {
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
    esp_lcd_panel_draw_bitmap(panel_handle, x0, y0, x0 + FLOWER_SIZE, y0 + FLOWER_SIZE, flower);
}

#if CONFIG_TFT_OTA_FROM_SDMMC
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mcp_dev;

enum {
    MCP23017_IODIRA = 0x00,
    MCP23017_IODIRB = 0x01,
    MCP23017_GPPUA  = 0x0C,
    MCP23017_GPPUB  = 0x0D,
    MCP23017_GPIOA  = 0x12,
    MCP23017_GPIOB  = 0x13,
};

static esp_err_t mcp23017_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(mcp_dev, buf, sizeof(buf), 100);
}

static esp_err_t mcp23017_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(mcp_dev, &reg, 1, val, 1, 100);
}

static esp_err_t mcp23017_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = TFT_I2C_SDA,
        .scl_io_num = TFT_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 4,
        .flags.enable_internal_pullup = 1,
    };

    ESP_LOGI(TAG, "I2C bus cfg: port=%d sda=%d scl=%d freq=%d", bus_cfg.i2c_port, bus_cfg.sda_io_num, bus_cfg.scl_io_num, (int)TFT_I2C_FREQ_HZ);

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TFT_MCP23017_ADDR,
        .scl_speed_hz = TFT_I2C_FREQ_HZ,
    };
    ESP_LOGI(TAG, "MCP23017 addr=0x%02X rst=%d A=%d B=%d SW=%d", dev_cfg.device_address, TFT_MCP23017_RST, TFT_ENCODER_PIN_A, TFT_ENCODER_PIN_B, TFT_ENCODER_PIN_SW);
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &mcp_dev));

    if (TFT_MCP23017_RST >= 0) {
        gpio_set_direction(TFT_MCP23017_RST, GPIO_MODE_OUTPUT);
        gpio_set_level(TFT_MCP23017_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(TFT_MCP23017_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_RETURN_ON_ERROR(mcp23017_write_reg(MCP23017_IODIRA, 0xFF), TAG, "IODIRA write failed");
    ESP_RETURN_ON_ERROR(mcp23017_write_reg(MCP23017_IODIRB, 0xFF), TAG, "IODIRB write failed");
    ESP_RETURN_ON_ERROR(mcp23017_write_reg(MCP23017_GPPUA, 0xFF), TAG, "GPPUA write failed");
    ESP_RETURN_ON_ERROR(mcp23017_write_reg(MCP23017_GPPUB, 0xFF), TAG, "GPPUB write failed");
    return ESP_OK;
}

static esp_err_t mcp23017_read_gpioa(uint8_t *gpioa) {
    return mcp23017_read_reg(MCP23017_GPIOA, gpioa);
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
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x05,0x03,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00}, // )
    {0x14,0x08,0x3E,0x08,0x14}, // *
    {0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x50,0x30,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00}, // ;
    {0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14}, // =
    {0x00,0x41,0x22,0x14,0x08}, // >
    {0x02,0x01,0x51,0x09,0x06}, // ?
    {0x32,0x49,0x79,0x41,0x3E}, // @
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x7F,0x20,0x18,0x20,0x7F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x03,0x04,0x78,0x04,0x03}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x00,0x7F,0x41,0x41,0x00}, // [
    {0x02,0x04,0x08,0x10,0x20}, // backslash
    {0x00,0x41,0x41,0x7F,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04}, // ^
    {0x80,0x80,0x80,0x80,0x80}, // _
    {0x00,0x03,0x05,0x00,0x00}, // `
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
    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 6, y + 8, buffer);
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

static void draw_status_bar(esp_lcd_panel_handle_t panel_handle, bool mcp_ok, bool sd_ok, bool touch_ok, uint16_t tx, uint16_t ty) {
    uint16_t bg = rgb565(8, 8, 12);
    uint16_t fg = rgb565(220, 220, 220);
    char line[64];

    draw_rect(panel_handle, 0, 0, LCD_H_RES, 16, bg);
    snprintf(line, sizeof(line), "MCP:%s SD:%s T:%s", mcp_ok ? "OK" : "--", sd_ok ? "OK" : "--", touch_ok ? "OK" : "--");
    draw_text(panel_handle, 4, 4, line, fg, bg);

    draw_rect(panel_handle, 0, 16, LCD_H_RES, 16, bg);
    snprintf(line, sizeof(line), "X:%u Y:%u", (unsigned)tx, (unsigned)ty);
    draw_text(panel_handle, 4, 20, line, fg, bg);
}

static esp_err_t mount_sdcard(sdmmc_card_t **out_card);
static esp_err_t ota_from_sdcard(const char *path);

static esp_err_t scan_sdcard(sd_file_list_t *list) {
    sdmmc_card_t *card = NULL;
    list->count = 0;
    list->selected = -1;

    esp_err_t err = mount_sdcard(&card);
    if (err != ESP_OK) {
        return err;
    }

    DIR *dir = opendir(CONFIG_TFT_SDMMC_MOUNT_POINT);
    if (!dir) {
        esp_vfs_fat_sdcard_unmount(CONFIG_TFT_SDMMC_MOUNT_POINT, card);
        return ESP_FAIL;
    }

    struct dirent *ent = NULL;
    while ((ent = readdir(dir)) != NULL) {
        if (ent->d_name[0] == '.') {
            continue;
        }
        if (!ends_with_bin(ent->d_name)) {
            continue;
        }
        if (list->count >= (int)(sizeof(list->names) / sizeof(list->names[0]))) {
            break;
        }
        strncpy(list->names[list->count], ent->d_name, sizeof(list->names[list->count]) - 1);
        list->names[list->count][sizeof(list->names[list->count]) - 1] = '\0';
        list->count++;
    }
    closedir(dir);
    esp_vfs_fat_sdcard_unmount(CONFIG_TFT_SDMMC_MOUNT_POINT, card);
    return ESP_OK;
}

static void draw_sd_list(esp_lcd_panel_handle_t panel_handle, const sd_file_list_t *list) {
    fill_color(panel_handle, rgb565(0, 0, 0));
    draw_text(panel_handle, 8, 6, "SELECT OTA FILE", rgb565(255, 255, 255), rgb565(0, 0, 0));
    int y = 24;
    for (int i = 0; i < list->count; ++i) {
        bool active = (i == list->selected);
        draw_button(panel_handle, 8, y, LCD_H_RES - 16, 24, list->names[i], active);
        y += 28;
    }
    draw_button(panel_handle, 8, LCD_V_RES - 30, 80, 22, "BACK", false);
}

static void draw_sd_idle(esp_lcd_panel_handle_t panel_handle) {
    fill_color(panel_handle, rgb565(0, 0, 0));
    draw_text(panel_handle, 8, 6, "SD OTA", rgb565(255, 255, 255), rgb565(0, 0, 0));
    draw_text(panel_handle, 8, 18, "TAP TO SCAN", rgb565(180, 180, 180), rgb565(0, 0, 0));
    draw_button(panel_handle, 40, 80, 160, 40, "LOAD SD CARD", false);
}

static int encoder_delta(uint8_t curr_ab, uint8_t *last_ab) {
    static const int8_t table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    int index = ((*last_ab) << 2) | (curr_ab & 0x03);
    *last_ab = curr_ab & 0x03;
    return table[index];
}

static bool encoder_poll(int *delta, bool *pressed) {
    uint8_t gpioa = 0;
    if (mcp23017_read_gpioa(&gpioa) != ESP_OK) {
        return false;
    }

    uint8_t a = (gpioa >> TFT_ENCODER_PIN_A) & 0x01;
    uint8_t b = (gpioa >> TFT_ENCODER_PIN_B) & 0x01;
    uint8_t sw = (gpioa >> TFT_ENCODER_PIN_SW) & 0x01;

    static uint8_t last_ab = 0;
    static uint8_t last_sw = 1;

    *delta = encoder_delta((uint8_t)((a << 1) | b), &last_ab);
    *pressed = (last_sw == 1 && sw == 0);
    last_sw = sw;

    if (*delta != 0 || *pressed) {
        ESP_LOGI(TAG, "ENC gpioa=0x%02X a=%u b=%u sw=%u delta=%d pressed=%d", gpioa, a, b, sw, *delta, *pressed);
    }
    return true;
}

static esp_err_t mount_sdcard(sdmmc_card_t **out_card) {
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = TFT_SDMMC_FREQ_KHZ;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = TFT_SDMMC_CLK_PIN;
    slot_config.cmd = TFT_SDMMC_CMD_PIN;
    slot_config.d0 = TFT_SDMMC_D0_PIN;
    slot_config.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    gpio_set_pull_mode(TFT_SDMMC_CMD_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(TFT_SDMMC_D0_PIN, GPIO_PULLUP_ONLY);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 16 * 1024,
    };

    ESP_LOGI(TAG, "SDMMC cfg: clk=%d cmd=%d d0=%d freq=%d kHz", TFT_SDMMC_CLK_PIN, TFT_SDMMC_CMD_PIN, TFT_SDMMC_D0_PIN, TFT_SDMMC_FREQ_KHZ);

    sdmmc_card_t *card = NULL;
    esp_err_t err = esp_vfs_fat_sdmmc_mount(CONFIG_TFT_SDMMC_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SDMMC mount failed: %s", esp_err_to_name(err));
        return err;
    }

    *out_card = card;
    return ESP_OK;
}

static esp_err_t ota_from_sdcard(const char *path) {
    sdmmc_card_t *card = NULL;
    esp_err_t err = mount_sdcard(&card);
    if (err != ESP_OK) {
        return err;
    }

    FILE *file = fopen(path, "rb");
    if (!file) {
        ESP_LOGE(TAG, "OTA image not found: %s", path);
        esp_vfs_fat_sdcard_unmount(CONFIG_TFT_SDMMC_MOUNT_POINT, card);
        return ESP_ERR_NOT_FOUND;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition available");
        fclose(file);
        esp_vfs_fat_sdcard_unmount(CONFIG_TFT_SDMMC_MOUNT_POINT, card);
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        fclose(file);
        esp_vfs_fat_sdcard_unmount(CONFIG_TFT_SDMMC_MOUNT_POINT, card);
        return err;
    }

    uint8_t buffer[4096];
    size_t bytes_read = 0;
    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        err = esp_ota_write(ota_handle, buffer, bytes_read);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
            esp_ota_end(ota_handle);
            fclose(file);
            esp_vfs_fat_sdcard_unmount(CONFIG_TFT_SDMMC_MOUNT_POINT, card);
            return err;
        }
    }

    fclose(file);
    esp_vfs_fat_sdcard_unmount(CONFIG_TFT_SDMMC_MOUNT_POINT, card);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA set boot partition failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "OTA successful, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

#endif

void app_main(void) {
    // 1. KHỞI TẠO SPI BUS DÙNG CHUNG
    spi_bus_config_t buscfg = {
        .sclk_io_num = TFT_SCLK,
        .mosi_io_num = TFT_MOSI,
        .miso_io_num = TFT_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 2. KHỞI TẠO MÀN HÌNH (LCD)
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

#if CONFIG_TFT_TEST_PATTERN
    fill_color(panel_handle, rgb565(0, 0, 0));
    draw_test_pattern(panel_handle);
#endif

#if TFT_TOUCH_ENABLED
    ESP_ERROR_CHECK(touch_spi_init());
    ESP_LOGI(TAG, "Touch SPI: sclk=%d mosi=%d miso=%d cs=%d", TFT_TOUCH_SPI_SCLK, TFT_TOUCH_SPI_MOSI, TFT_TOUCH_SPI_MISO, TOUCH_CS);
#endif

    ESP_LOGI(TAG, "He thong da san sang!");

#if CONFIG_TFT_OTA_FROM_SDMMC
    bool redraw = true;
    bool sd_ok = false;
#endif
#if TFT_TOUCH_ENABLED && CONFIG_TFT_OTA_FROM_SDMMC
    sd_file_list_t file_list = {0};
    bool ui_list = false;
    bool last_pressed = false;
    uint32_t last_touch_ms = 0;
    uint16_t last_tx = 0;
    uint16_t last_ty = 0;
    bool touch_seen = false;
#endif
#if CONFIG_TFT_OTA_FROM_SDMMC && !TFT_TOUCH_ENABLED
    sd_file_list_t encoder_list = {0};
    bool encoder_ready = false;
    bool encoder_ui_list = false;
    int encoder_last_delta = 0;
    bool encoder_last_pressed = false;
#endif

    while (1) {
#if TFT_TOUCH_ENABLED && CONFIG_TFT_OTA_TOUCH_TRIGGER && CONFIG_TFT_OTA_FROM_SDMMC
        static uint32_t hold_start_ms = 0;
    uint16_t x = 0;
    uint16_t y = 0;
    bool pressed = touch_get_xy(&x, &y);
    if (pressed) {
        last_tx = x;
        last_ty = y;
            touch_seen = true;
    }

        if (redraw) {
            if (ui_list) {
                draw_sd_list(panel_handle, &file_list);
            } else {
                draw_sd_idle(panel_handle);
            }
            draw_status_bar(panel_handle, true, sd_ok, touch_seen, last_tx, last_ty);
            redraw = false;
        }

        if (pressed) {
            last_touch_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        }

        bool tapped = pressed && !last_pressed;
        last_pressed = pressed;

        if (tapped) {
            int tx = x;
            int ty = y;

            if (!ui_list) {
                ESP_LOGI(TAG, "Scanning SD card");
                if (scan_sdcard(&file_list) == ESP_OK && file_list.count > 0) {
                    file_list.selected = 0;
                    ui_list = true;
                    sd_ok = true;
                } else {
                    ESP_LOGW(TAG, "No .bin files found on SD card");
                    sd_ok = false;
                }
                redraw = true;
            } else {
                if (ty >= LCD_V_RES - 30 && ty <= LCD_V_RES - 8 && tx >= 8 && tx <= 88) {
                    ui_list = false;
                    redraw = true;
                } else {
                    int index = (ty - 24) / 28;
                    if (index >= 0 && index < file_list.count) {
                        file_list.selected = index;
                        redraw = true;
                        char path[128];
                        snprintf(path, sizeof(path), "%s/%s", CONFIG_TFT_SDMMC_MOUNT_POINT, file_list.names[index]);
                        ESP_LOGI(TAG, "Starting OTA: %s", path);
                        ota_from_sdcard(path);
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
                    ota_from_sdcard(CONFIG_TFT_OTA_BIN_PATH);
                }
            }
        } else {
            hold_start_ms = 0;
        }
#if TFT_TOUCH_ENABLED
        if (last_touch_ms == 0) {
            last_touch_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        } else {
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if ((now_ms - last_touch_ms) >= 10000) {
                ESP_LOGW(TAG, "No touch detected for 10s. Check wiring and touch CS/MISO.");
                last_touch_ms = now_ms;
            }
        }
#endif
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
                    if (scan_sdcard(&encoder_list) == ESP_OK && encoder_list.count > 0) {
                        encoder_list.selected = 0;
                        encoder_ui_list = true;
                        sd_ok = true;
                    } else {
                        ESP_LOGW(TAG, "No .bin files found on SD card");
                        sd_ok = false;
                    }
                    redraw = true;
                } else if (encoder_list.count > 0) {
                    char path[128];
                    snprintf(path, sizeof(path), "%s/%s", CONFIG_TFT_SDMMC_MOUNT_POINT, encoder_list.names[encoder_list.selected]);
                    ESP_LOGI(TAG, "Starting OTA: %s", path);
                    ota_from_sdcard(path);
                }
            }
        }

        if (redraw) {
            if (encoder_ui_list) {
                draw_sd_list(panel_handle, &encoder_list);
            } else {
                draw_sd_idle(panel_handle);
            }
            draw_status_bar(panel_handle, encoder_ready, sd_ok, false, (uint16_t)encoder_last_delta, (uint16_t)(encoder_last_pressed ? 1 : 0));
            redraw = false;
        }
#endif

#if CONFIG_TFT_OTA_ON_BOOT && CONFIG_TFT_OTA_FROM_SDMMC
        static bool ota_done = false;
        if (!ota_done) {
            ota_done = true;
            ESP_LOGI(TAG, "Starting SD card OTA on boot");
            ota_from_sdcard(CONFIG_TFT_OTA_BIN_PATH);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}