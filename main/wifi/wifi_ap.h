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
#define DEFAULT_SSID                "Home Wifi"
#define DEFAULT_PWD                 "Home Passwd"
#define DEFAULT_SCAN_METHOD         WIFI_FAST_SCAN
#define DEFAULT_SORT_METHOD         WIFI_CONNECT_AP_BY_SIGNAL
#define DEFAULT_RSSI                -127
#define DEFAULT_AUTHMODE            WIFI_AUTH_WPA2_PSK
#define DEFAULT_RSSI_5G_ADJUSTMENT  0

// Access point create
#define WIFI_SSID      "CO2.local"
#define WIFI_PASS      "" // Empty str len 0 = No password, this AP is KIOSK-like
#define WIFI_CHANNEL   1
#define MAX_STA_CONN   10

extern int connected_users;
extern bool wifi_ap_mode;

// Start AP
void wifi_setup(void);