#include "esp_netif.h"
#include "esp_tls_crypto.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_check.h"

#define QUERY_KEY_MAX_LEN  (64)

// Define auth, if no auth - do not allow file upload
#define BASIC_AUTH 1
#if BASIC_AUTH

#define HTTPD_401 "401 UNAUTHORIZED"  /*!< HTTP Response 401 */
#define USERNAME "admin"
#define PASSWORD "can_upload_1!"

#endif

extern httpd_handle_t server = NULL;

