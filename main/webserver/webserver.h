#include "esp_netif.h"
#include "esp_tls_crypto.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_check.h"

// Include SD Card mount
#include "card_driver.h"
#include "local_flash.h"

#define QUERY_KEY_MAX_LEN  (64)

// Define auth, if no auth - do not allow file upload
#define BASIC_AUTH 0

#if BASIC_AUTH

#define HTTPD_401 "401 UNAUTHORIZED"  /*!< HTTP Response 401 */
#define USERNAME "admin"
#define PASSWORD "can_upload_1!"

typedef struct {
    char    *username;
    char    *password;
} basic_auth_info_t;

#endif

extern httpd_handle_t server = NULL;

// Server index.htnl
#define INDEX_HTML_PATH             "/littlefs/index.html"
#define INDEX_HTML_PATH_SD_CARD     "/sdcard/index.html"
// Server upload files capability
#define UPLOAD_HTML_PATH            "/littlefs/upload_script.html"

// Default Webserver root dir - here upload files: SD Card
#define WEBSERVER_ROOT              MOUNT_POINT
// Upload INIT server root dir - do not upload files here, but use as initial page for upload
#define FILESERVER_INIT_ROOT        LFS_MOUNT_POINT

void index_html_get_handler(httpd_req_t *req);
void favicon_get_handler(httpd_req_t *req);

esp_err_t start_webserver(void);
