#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t touch_init(void);
bool touch_get_xy(uint16_t *x, uint16_t *y);
