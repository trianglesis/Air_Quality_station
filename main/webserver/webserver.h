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

extern httpd_handle_t server;


// Default Webserver root dir - here upload files: SD Card
#define WEBSERVER_ROOT              MOUNT_POINT
// Upload INIT server root dir - do not upload files here, but use as initial page for upload
#define FILESERVER_INIT_ROOT        LFS_MOUNT_POINT

// File server opt
/* Max length a file path can have on storage */
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

/* Max size of an individual file. Make sure this
 * value is same as that set in upload_script.html */
#define MAX_FILE_SIZE   (500*1024) // 500 KB
#define MAX_FILE_SIZE_STR "500KB"

/* Scratch buffer size */
#define SCRATCH_BUFSIZE  8192

// Server upload files capability
#define UPLOAD_INDEX_PATH       "/littlefs/index.html"
#define UPLOAD_HTML_PATH        "/littlefs/upload_script.html"
#define UPLOAD_FAV_PATH         "/littlefs/favicon.ico"
#define QUERY_KEY_MAX_LEN  (64)

esp_err_t index_html_get_handler(httpd_req_t *req);
esp_err_t favicon_get_handler(httpd_req_t *req);
// esp_err_t root_get_handler(httpd_req_t *req);
// esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err);
esp_err_t start_webserver(void);