#pragma once

#include "esp_err.h"

typedef struct {
    int count;
    int selected;
    char names[8][64];
} sd_file_list_t;

esp_err_t sd_ota_scan(sd_file_list_t *list);
void sd_ota_start(const char *path);
