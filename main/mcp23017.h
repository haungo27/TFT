#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t mcp23017_init(void);
bool encoder_poll(int *delta, bool *pressed);
