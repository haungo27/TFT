#include "sd_ota.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "tft_config.h"

static const char *TAG = "TFT_OTA";

static bool ota_task_active = false;

static bool ends_with_bin(const char *name) {
    size_t len = strlen(name);
    if (len < 4) {
        return false;
    }
    return (name[len - 4] == '.') && ((name[len - 3] | 0x20) == 'b') && ((name[len - 2] | 0x20) == 'i') && ((name[len - 1] | 0x20) == 'n');
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

static void ota_task(void *arg) {
    char *path = (char *)arg;
    ESP_LOGI(TAG, "OTA task start: %s", path);
    ota_from_sdcard(path);
    free(path);
    ota_task_active = false;
    vTaskDelete(NULL);
}

void sd_ota_start(const char *path) {
    if (ota_task_active) {
        ESP_LOGW(TAG, "OTA already running, skip");
        return;
    }
    size_t len = strlen(path) + 1;
    char *copy = (char *)malloc(len);
    if (!copy) {
        ESP_LOGE(TAG, "OTA path alloc failed");
        return;
    }
    memcpy(copy, path, len);
    ota_task_active = true;
    if (xTaskCreate(ota_task, "ota_task", 8192, copy, 5, NULL) != pdPASS) {
        ota_task_active = false;
        free(copy);
        ESP_LOGE(TAG, "OTA task create failed");
    }
}

esp_err_t sd_ota_scan(sd_file_list_t *list) {
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
