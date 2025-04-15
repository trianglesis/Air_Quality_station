#include "local_flash.h"

static const char *TAG = "littlefs";

float littlefs_total = 0;
float littlefs_used = 0;

// Just test
esp_err_t fs_read(void) {
    ESP_LOGI(TAG, "Reading from flashed filesystem example.txt");
    FILE *f = fopen("/littlefs/README.md", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    fclose(f);
    return ESP_OK;
}

esp_err_t fs_setup(void) {
    // 
    esp_vfs_littlefs_conf_t conf = {
        .base_path = LFS_MOUNT_POINT,
        .partition_label = LFS_PARTITION_LABEL,
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        }
        else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    }
    else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    littlefs_total = (total / 1024); // Convert to Kb
    littlefs_used = (used / 1024); // Convert to Kb
    // https://cplusplus.com/reference/cstdio/printf/
    ESP_LOGI(TAG, "Partition size: %.2f/%.2f KB", littlefs_total, littlefs_used);
    return ESP_OK;

}