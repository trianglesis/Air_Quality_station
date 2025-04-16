#pragma once
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sys.h"


// Connect to local WIFi
#define SSID                        CONFIG_WIFI_SSID
#define PWD                         CONFIG_WIFI_PASSWORD
#define SCAN_METHOD                 CONFIG_SCAN_METHOD
#define SORT_METHOD                 CONFIG_SORT_METHOD
#define FAST_SCAN_THRESHOLD         CONFIG_FAST_SCAN_THRESHOLD
#define RSSI                        CONFIG_FAST_SCAN_MINIMUM_SIGNAL
#define AUTHMODE                    CONFIG_FAST_SCAN_WEAKEST_AUTHMODE
#define RSSI_5G_ADJUSTMENT          CONFIG_FAST_SCAN_RSSI_5G_ADJUSTMENT

// Access point create
#define WIFI_SSID      "CO2.local"
#define WIFI_PASS      "" // Empty str len 0 = No password, this AP is KIOSK-like
#define WIFI_CHANNEL   1
#define MAX_STA_CONN   10

extern int connected_users;
extern bool wifi_ap_mode;

// Start AP
void wifi_setup(void);