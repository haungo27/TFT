#include "mcp23017.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tft_config.h"

static const char *TAG = "TFT_MCP";

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mcp_dev;

enum {
    MCP23017_IODIRA = 0x00,
    MCP23017_IODIRB = 0x01,
    MCP23017_GPPUA = 0x0C,
    MCP23017_GPPUB = 0x0D,
    MCP23017_GPIOA = 0x12,
    MCP23017_GPIOB = 0x13,
};

static esp_err_t mcp23017_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(mcp_dev, buf, sizeof(buf), 100);
}

static esp_err_t mcp23017_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(mcp_dev, &reg, 1, val, 1, 100);
}

esp_err_t mcp23017_init(void) {
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

static esp_err_t mcp23017_read_gpiob(uint8_t *gpiob) {
    return mcp23017_read_reg(MCP23017_GPIOB, gpiob);
}

static int encoder_delta(uint8_t curr_ab, uint8_t *last_ab) {
    static const int8_t table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    int index = ((*last_ab) << 2) | (curr_ab & 0x03);
    *last_ab = curr_ab & 0x03;
    return table[index];
}

bool encoder_poll(int *delta, bool *pressed) {
    uint8_t gpioa = 0;
    uint8_t gpiob = 0;
    if (mcp23017_read_gpioa(&gpioa) != ESP_OK) {
        return false;
    }
    if (mcp23017_read_gpiob(&gpiob) != ESP_OK) {
        return false;
    }

    uint16_t gpio = (uint16_t)gpioa | ((uint16_t)gpiob << 8);
    if (TFT_ENCODER_PIN_A > 15 || TFT_ENCODER_PIN_B > 15 || TFT_ENCODER_PIN_SW > 15) {
        return false;
    }

    uint8_t a = (gpio >> TFT_ENCODER_PIN_A) & 0x01;
    uint8_t b = (gpio >> TFT_ENCODER_PIN_B) & 0x01;
    uint8_t sw = (gpio >> TFT_ENCODER_PIN_SW) & 0x01;

    static uint8_t last_ab = 0;
    static uint8_t last_sw = 1;

    *delta = encoder_delta((uint8_t)((a << 1) | b), &last_ab);
    *pressed = (last_sw == 1 && sw == 0);
    last_sw = sw;

    if (*delta != 0 || *pressed) {
        ESP_LOGI(TAG, "ENC gpio=0x%04X a=%u b=%u sw=%u delta=%d pressed=%d", gpio, a, b, sw, *delta, *pressed);
    }
    return true;
}
