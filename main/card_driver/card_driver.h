#pragma once

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "mbedtls/md5.h"

#include "sdmmc_cmd.h"
#include "esp_flash.h"

#include "ST7789V3.h"

// SPI default PINs
#define PIN_NUM_MOSI    DISP_GPIO_MOSI
#define PIN_NUM_MISO    GPIO_NUM_5
#define PIN_NUM_SCLK    DISP_GPIO_SCLK
#define PIN_NUM_CS      GPIO_NUM_4

// Will format SD card on failed mount each time
#define FORMAT_IF_MOUNT_FAILED True
#define FORMAT_AT_MOUNT False
#define PARTITION_LABEL "SD_Card"
#define MOUNT_POINT "/sdcard"


// SDIO is probably now available at this board
// #define SDIO_D2_SD_PIN NULL
// #define SDIO_D3_SD_PIN NULL
// #define SDIO_D0_SD_PIN NULL
// #define SDIO_D1_SD_PIN NULL

esp_err_t SD_Card_CS_EN(void);
esp_err_t SD_Card_CS_Dis(void);

esp_err_t s_example_write_file(const char *path, char *data);
void file_sum_test(void);
void file_read_test(void);

extern uint32_t SDCard_Size;
extern uint32_t Flash_Size;

esp_err_t card_init(void);
void Flash_Searching(void);