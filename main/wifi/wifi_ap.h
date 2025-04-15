#pragma once
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_SSID      "CO2.local"
#define WIFI_PASS      "" // Empty str len 0 = No password, this AP is KIOSK-like
#define WIFI_CHANNEL   1
#define MAX_STA_CONN   10

extern int connected_users;
extern bool wifi_ap_mode;

// Start AP
void wifi_setup(void);