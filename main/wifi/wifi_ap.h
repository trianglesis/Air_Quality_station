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
#define RSSI                        CONFIG_FAST_SCAN_MINIMUM_SIGNAL

#if CONFIG_WIFI_ALL_CHANNEL_SCAN
#define SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_WIFI_FAST_SCAN
#define SCAN_METHOD WIFI_FAST_SCAN
#else
#define SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_SCAN_METHOD*/

#if CONFIG_WIFI_CONNECT_AP_BY_SIGNAL
#define SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_WIFI_CONNECT_AP_BY_SECURITY
#define SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#else
#define SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#endif /*CONFIG_SORT_METHOD*/

#if CONFIG_FAST_SCAN_THRESHOLD
#define RSSI CONFIG_FAST_SCAN_MINIMUM_SIGNAL

#if CONFIG_FAST_SCAN_WEAKEST_AUTHMODE_OPEN
#define AUTHMODE WIFI_AUTH_OPEN
#elif CONFIG_FAST_SCAN_WEAKEST_AUTHMODE_WEP
#define AUTHMODE WIFI_AUTH_WEP
#elif CONFIG_FAST_SCAN_WEAKEST_AUTHMODE_WPA
#define AUTHMODE WIFI_AUTH_WPA_PSK
#elif CONFIG_FAST_SCAN_WEAKEST_AUTHMODE_WPA2
#define AUTHMODE WIFI_AUTH_WPA2_PSK
#else
#define AUTHMODE WIFI_AUTH_OPEN
#endif

#if CONFIG_SOC_WIFI_SUPPORT_5G
#define RSSI_5G_ADJUSTMENT CONFIG_FAST_SCAN_RSSI_5G_ADJUSTMENT
#else
#define RSSI_5G_ADJUSTMENT 0
#endif
#else
#define RSSI -127
#define AUTHMODE WIFI_AUTH_OPEN
#define RSSI_5G_ADJUSTMENT 0
#endif /*CONFIG_FAST_SCAN_THRESHOLD*/



// Access point create
#define WIFI_SSID      "CO2.local"
#define WIFI_PASS      "" // Empty str len 0 = No password, this AP is KIOSK-like
#define WIFI_CHANNEL   1
#define MAX_STA_CONN   10

extern int connected_users;
extern bool wifi_ap_mode;

// Start AP
void wifi_setup(void);