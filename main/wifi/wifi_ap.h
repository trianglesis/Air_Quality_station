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

// Connect to local WIFi (STA)
#define SSID                        CONFIG_WIFI_SSID
#define PWD                         CONFIG_WIFI_PASSWORD
#define RSSI                        CONFIG_FAST_SCAN_MINIMUM_SIGNAL
#define ESP_MAXIMUM_RETRY           CONFIG_ESP_MAXIMUM_STA_RETRY

#define DEFAULT_SCAN_LIST_SIZE      20


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
/* AP Configuration */
#define WIFI_SSID                   CONFIG_ESP_WIFI_AP_SSID
#define WIFI_PASS                   CONFIG_ESP_WIFI_AP_PASSWORD // Empty str len 0 = No password, this AP is KIOSK-like
#define WIFI_CHANNEL                CONFIG_ESP_WIFI_AP_CHANNEL
#define MAX_STA_CONN                CONFIG_ESP_MAX_STA_CONN_AP

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/*DHCP server option*/
#define DHCPS_OFFER_DNS             0x02

extern int connected_users;  // Display connected users
extern bool wifi_ap_mode;    // Display AP mode ICON
extern bool found_wifi;      // Hide AP mode icon and connected users, and show local IP
extern char local_ip;      // Hide AP mode icon and connected users, and show local IP

// Start AP
esp_err_t wifi_setup(void);