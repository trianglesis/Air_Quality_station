#include "card_driver.h"

#define EXAMPLE_MAX_CHAR_SIZE    64

static const char *TAG = "SD-SPI";

// Mount path for the partition
const char *base_path = MOUNT_POINT;

uint32_t Flash_Size = 0;
uint32_t SDCard_Size = 0;

// Leave it as in example
esp_err_t s_example_write_file(const char *path, char *data) {
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

// Based on two different examples
void file_sum_test(void) {
    ESP_LOGI(TAG, "Computing README.md MD5 hash and test reading");
    FILE* f = fopen("/sdcard/README.md", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open alice.txt");
    }
    // Read file and compute the digest chunk by chunk
    #define MD5_MAX_LEN 16

    char buf[64];
    mbedtls_md5_context ctx;
    unsigned char digest[MD5_MAX_LEN];

    mbedtls_md5_init(&ctx);
    mbedtls_md5_starts(&ctx);

    size_t read;

    do {
        read = fread((void*) buf, 1, sizeof(buf), f);
        mbedtls_md5_update(&ctx, (unsigned const char*) buf, read);
    } while(read == sizeof(buf));

    mbedtls_md5_finish(&ctx, digest);

    // Create a string of the digest
    char digest_str[MD5_MAX_LEN * 2];

    for (int i = 0; i < MD5_MAX_LEN; i++) {
        sprintf(&digest_str[i * 2], "%02x", (unsigned int)digest[i]);
    }

    // For reference, MD5 should be deeb71f585cbb3ae5f7976d5127faf2a
    ESP_LOGI(TAG, "Computed MD5 hash of alice.txt: %s", digest_str);

    fclose(f);
}

// Based on two different examples
void file_read_test(void) {
    ESP_LOGI(TAG, "Reading README.md");
    // Open for reading README.md
    FILE* f = fopen("/sdcard/README.md", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open hello.txt");
    }
    char buf[64];
    memset(buf, 0, sizeof(buf));
    fread(buf, 1, sizeof(buf), f);
    fclose(f);
    // Display the read contents from the file
    ESP_LOGI(TAG, "Read from README.md: %s", buf);
}

void card_init(void) {
    esp_err_t ret;

    // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/fatfs.html#_CPPv426esp_vfs_fat_mount_config_t
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .max_files = 10,
        .format_if_mount_failed = true,
        .allocation_unit_size = 16 * 1024,
        .use_one_fat = false,
    };

    sdmmc_card_t *card;
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SPI peripheral");
    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();


    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(base_path, &host, &slot_config, &mount_config, &card);
    // Check errors
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. Set FORMAT_IF_MOUNT_FAILED in header file.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    SDCard_Size = ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024);

    ret = file_sum_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calc md5sum of the test file from SD Card FS.");
        return;
    }
    ret = file_read_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read test file from SD Card FS.");
        return;
    }
    // There is wait_fort_idle function can be usefull on device powering off or SD remove
}


void Flash_Searching(void) {
    if(esp_flash_get_physical_size(NULL, &Flash_Size) == ESP_OK) {
        Flash_Size = Flash_Size / (uint32_t)(1024 * 1024);
        printf("Flash size: %ld MB\n", Flash_Size);
    } else {
        printf("Get flash size failed\n");
    }
}