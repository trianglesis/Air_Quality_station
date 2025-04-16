#include "webserver.h"
#include "file_server.h"

char response_data[4096];
char index_html[4096];

static const char *TAG = "webserver";

#if BASIC_AUTH

static char *http_auth_basic(const char *username, const char *password) {
    size_t out;
    char *user_info = NULL;
    char *digest = NULL;
    size_t n = 0;
    int rc = asprintf(&user_info, "%s:%s", username, password);
    if (rc < 0) {
        ESP_LOGE(TAG, "asprintf() returned: %d", rc);
        return NULL;
    }

    if (!user_info) {
        ESP_LOGE(TAG, "No enough memory for user information");
        return NULL;
    }
    esp_crypto_base64_encode(NULL, 0, &n, (const unsigned char *)user_info, strlen(user_info));

    /* 6: The length of the "Basic " string
     * n: Number of bytes for a base64 encode format
     * 1: Number of bytes for a reserved which be used to fill zero
    */
    digest = calloc(1, 6 + n + 1);
    if (digest) {
        strcpy(digest, "Basic ");
        esp_crypto_base64_encode((unsigned char *)digest + 6, n, &out, (const unsigned char *)user_info, strlen(user_info));
    }
    free(user_info);
    return digest;
}

/* An HTTP GET handler */
static esp_err_t basic_auth_get_handler(httpd_req_t *req)
{
    char *buf = NULL;
    size_t buf_len = 0;
    basic_auth_info_t *basic_auth_info = req->user_ctx;

    buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
    if (buf_len > 1) {
        buf = calloc(1, buf_len);
        if (!buf) {
            ESP_LOGE(TAG, "No enough memory for basic authorization");
            return ESP_ERR_NO_MEM;
        }

        if (httpd_req_get_hdr_value_str(req, "Authorization", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Authorization: %s", buf);
        } else {
            ESP_LOGE(TAG, "No auth value received");
        }

        char *auth_credentials = http_auth_basic(basic_auth_info->username, basic_auth_info->password);
        if (!auth_credentials) {
            ESP_LOGE(TAG, "No enough memory for basic authorization credentials");
            free(buf);
            return ESP_ERR_NO_MEM;
        }

        if (strncmp(auth_credentials, buf, buf_len)) {
            ESP_LOGE(TAG, "Not authenticated");
            httpd_resp_set_status(req, HTTPD_401);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
            httpd_resp_send(req, NULL, 0);
        } else {
            ESP_LOGI(TAG, "Authenticated!");
            char *basic_auth_resp = NULL;
            httpd_resp_set_status(req, HTTPD_200);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            int rc = asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", basic_auth_info->username);
            if (rc < 0) {
                ESP_LOGE(TAG, "asprintf() returned: %d", rc);
                free(auth_credentials);
                return ESP_FAIL;
            }
            if (!basic_auth_resp) {
                ESP_LOGE(TAG, "No enough memory for basic authorization response");
                free(auth_credentials);
                free(buf);
                return ESP_ERR_NO_MEM;
            }
            httpd_resp_send(req, basic_auth_resp, strlen(basic_auth_resp));
            free(basic_auth_resp);
        }
        free(auth_credentials);
        free(buf);
    } else {
        ESP_LOGE(TAG, "No auth header received");
        httpd_resp_set_status(req, HTTPD_401);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
        httpd_resp_send(req, NULL, 0);
    }

    return ESP_OK;
}

static httpd_uri_t basic_auth = {
    .uri       = "/basic_auth",
    .method    = HTTP_GET,
    .handler   = basic_auth_get_handler,
};

static void httpd_register_basic_auth(httpd_handle_t server) {
    basic_auth_info_t *basic_auth_info = calloc(1, sizeof(basic_auth_info_t));
    if (basic_auth_info) {
        basic_auth_info->username = USERNAME;
        basic_auth_info->password = PASSWORD;

        basic_auth.user_ctx = basic_auth_info;
        httpd_register_uri_handler(server, &basic_auth);
    }
}

#endif

/* Handler to redirect incoming GET request for /index.html to /
 * This can be overridden by uploading file with same name */
esp_err_t index_html_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);  // Response body can be empty
    return ESP_OK;
}

/* Handler to respond with an icon file embedded in flash.
 * Browsers expect to GET website icon at URI /favicon.ico.
 * This can be overridden by uploading file with same name */
esp_err_t favicon_get_handler(httpd_req_t *req)
{
    /* Get handle to embedded file upload script */
    char upload_fav[4096];
    const char *file_path;
    file_path = UPLOAD_FAV_PATH;
    struct stat st;
    // Load html file
    memset((void *)upload_fav, 0, sizeof(upload_fav));
    if (stat(file_path, &st)) {
        ESP_LOGE(TAG, "Upload favicon not found at LittleFS!");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, upload_fav, sizeof(upload_fav));
    return ESP_OK;
}


static void load_index_file_buffer(void) {
    // Load html file
    memset((void *)index_html, 0, sizeof(index_html));
    struct stat st;
    if (stat("/littlefs/index.html", &st)) {
        ESP_LOGE(TAG, "index.html not found");
        return;
    }

    FILE *fp = fopen("/littlefs/index.html", "r");
    if (fread(index_html, st.st_size, 1, fp) == 0) {
        ESP_LOGE(TAG, "fread failed");
    }
    fclose(fp);
}

/*
I cannot get this function to work with buffer as arg, so will use it as it is for now.a64l
Everything else is working:a64l

I (18438) webserver: Load HTML from local store path
I (18438) webserver: LittleFS HTML Exist: 0
I (18438) webserver: LittleFS SD Exist: -1
I (18438) webserver: Root index.html is not found at SD Card, use LittleFS!
*/
static void load_index_file_buffer_dyn(char* file_path) {
    ESP_LOGI(TAG, "Load HTML from local store path");
    struct stat st;
    // ESP_LOGI(TAG, "LittleFS HTML Exist: %d", stat("/littlefs/index.html", &st));
    // ESP_LOGI(TAG, "LittleFS SD Exist: %d", stat("/sdcard/index.html", &st));
    
    // Actual path, check if exist
    if (file_path != NULL) {
        ESP_LOGI(TAG, "Load HTML from path: %s", file_path);
        if (stat(file_path, &st) == 0) {
            ESP_LOGI(TAG, "HTML exists at path: %s", file_path);
        } else {
            ESP_LOGI(TAG, "HTML is not exist at path: %s", file_path);
            // Always exist with firmware
            file_path = "/littlefs/index.html";
        }
    } else {
        // Serve index html from LittleFS if there is no index at SD Card yet.   
        if (stat("/sdcard/index.html", &st) == 0) {
            file_path = "/sdcard/index.html";
            ESP_LOGI(TAG, "Root index.html found at SD Card!");
        } else {
            file_path = "/littlefs/index.html";
            ESP_LOGI(TAG, "Root index.html is not found at SD Card, use LittleFS!");
        }
    }
    
    // File size
    // char index_html_buff[4096];
    memset((void *)index_html, 0, sizeof(index_html));

    FILE *f_r = fopen(file_path, "r");
    if (f_r != NULL) {
        int cb = fread(index_html, st.st_size, sizeof(index_html), f_r);
        if (cb == 0) {
            // Check if read and close
            ESP_LOGE(TAG, "fread failed for html at path %s", file_path);
            fclose(f_r);
        } else {
            // Close
            fclose(f_r);
        }
    }
    // Does not work as expected
    // return buf;
}

// Root page if present
esp_err_t root_get_handler(httpd_req_t *req) {
    load_index_file_buffer_dyn(NULL);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};

// HTTP Error (404) Handler - Redirects all requests to the root page
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect...", HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "Redirecting to root");
    return ESP_OK;
}

esp_err_t start_webserver(void) {
    
    // Set global
    static httpd_handle_t server = NULL;
    
    // Use default
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // always check config LWIP_MAX_SOCKETS = 20
    config.max_open_sockets = 13;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 10;
    config.max_resp_headers = 10;

    /* Use the URI wildcard matching function in order to
     * allow the same handler to respond to multiple different
     * target URIs which match the wildcard scheme */
    config.uri_match_fn = httpd_uri_match_wildcard;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }

    httpd_register_uri_handler(server, &root);
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    #if BASIC_AUTH
    httpd_register_basic_auth(server);
    #endif
    
    // Now start file server:
    // TODO STatrt later as soon as index and root is working
    // start_file_server(server);
    // Do not rerutn server, it is now in global var
    return ESP_OK;
}
