#include "wifi_ap.h"
#include "card_driver.h"
#include "local_flash.h"
#include "webserver.h"

char response_data[4096];
char index_html[4096];
char upload_fav[4096];
char upload_html[4096];

static const char *TAG = "webserver";
static const char *TAG_FS = "file_server";


struct file_server_data {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};

/*
    File server part
*/

/* Send HTTP response with a run-time generated html consisting of
 * a list of all files and folders under the requested path.
 * In case of SPIFFS this returns empty list when path is any
 * string other than '/', since SPIFFS doesn't support directories */
 static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath)
 {
     char entrypath[FILE_PATH_MAX];
     char entrysize[16];
     const char *entrytype;
 
     struct dirent *entry;
     struct stat entry_stat;
     struct stat st;

 
     DIR *dir = opendir(dirpath);
     const size_t dirpath_len = strlen(dirpath);
 
     /* Retrieve the base path of file storage to construct the full path */
     strlcpy(entrypath, dirpath, sizeof(entrypath));
 
     if (!dir) {
         ESP_LOGE(TAG_FS, "Failed to stat dir : %s", dirpath);
         /* Respond with 404 Not Found */
         httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
         return ESP_FAIL;
     } else {
        ESP_LOGI(TAG_FS, "Dir exist: %s", dirpath);
     }
 
     /* Send HTML file header */
     httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html><body>");
 
     /* Get handle to embedded file upload script */
    //  extern const unsigned char upload_script_start[] asm("_binary_upload_script_html_start");
    //  extern const unsigned char upload_script_end[]   asm("_binary_upload_script_html_end");
    //  const size_t upload_script_size = (upload_script_end - upload_script_start);

    // File size
    memset((void *)upload_html, 0, sizeof(upload_html));

    if (stat(UPLOAD_HTML_PATH, &st) == 0) {
        ESP_LOGI(TAG_FS, "HTML exists at path: %s", UPLOAD_HTML_PATH);
    }

    FILE *f_r = fopen(UPLOAD_HTML_PATH, "r");
    if (f_r != NULL) {
        int cb = fread(upload_html, st.st_size, sizeof(upload_html), f_r);
        if (cb == 0) {
            // File OK, close after
            ESP_LOGE(TAG_FS, "fread (%d) OK for upload html script at path %s", cb, UPLOAD_HTML_PATH);
            fclose(f_r);
        } else {
            // File not ok, show it
            ESP_LOGE(TAG_FS, "fread (%d) failed for html at path %s", cb, UPLOAD_HTML_PATH);
            fclose(f_r);
        }
    }

     /* Add file upload form and script which on execution sends a POST request to /upload */
     httpd_resp_send_chunk(req, (const char *)upload_html, sizeof(upload_html));
 
     /* Send file-list table definition and column labels */
     httpd_resp_sendstr_chunk(req,
         "<table class=\"fixed\" border=\"1\">"
         "<col width=\"800px\" /><col width=\"300px\" /><col width=\"300px\" /><col width=\"100px\" />"
         "<thead><tr><th>Name</th><th>Type</th><th>Size (Bytes)</th><th>Delete</th></tr></thead>"
         "<tbody>");
 
     /* Iterate over all files / folders and fetch their names and sizes */
     while ((entry = readdir(dir)) != NULL) {
         entrytype = (entry->d_type == DT_DIR ? "directory" : "file");
 
         strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
         if (stat(entrypath, &entry_stat) == -1) {
             ESP_LOGE(TAG_FS, "Failed to stat %s : %s", entrytype, entry->d_name);
             continue;
         }
         sprintf(entrysize, "%ld", entry_stat.st_size);
         ESP_LOGI(TAG_FS, "Found %s : %s (%s bytes)", entrytype, entry->d_name, entrysize);
 
         /* Send chunk of HTML file containing table entries with file name and size */
         httpd_resp_sendstr_chunk(req, "<tr><td><a href=\"");
         httpd_resp_sendstr_chunk(req, req->uri);
         httpd_resp_sendstr_chunk(req, entry->d_name);
         if (entry->d_type == DT_DIR) {
             httpd_resp_sendstr_chunk(req, "/");
         }
         httpd_resp_sendstr_chunk(req, "\">");
         httpd_resp_sendstr_chunk(req, entry->d_name);
         httpd_resp_sendstr_chunk(req, "</a></td><td>");
         httpd_resp_sendstr_chunk(req, entrytype);
         httpd_resp_sendstr_chunk(req, "</td><td>");
         httpd_resp_sendstr_chunk(req, entrysize);
         httpd_resp_sendstr_chunk(req, "</td><td>");
         httpd_resp_sendstr_chunk(req, "<form method=\"post\" action=\"/delete");
         httpd_resp_sendstr_chunk(req, req->uri);
         httpd_resp_sendstr_chunk(req, entry->d_name);
         httpd_resp_sendstr_chunk(req, "\"><button type=\"submit\">Delete</button></form>");
         httpd_resp_sendstr_chunk(req, "</td></tr>\n");
     }
     closedir(dir);
 
     /* Finish the file list table */
     httpd_resp_sendstr_chunk(req, "</tbody></table>");
 
     /* Send remaining chunk of HTML file to complete it */
     httpd_resp_sendstr_chunk(req, "</body></html>");
 
     /* Send empty chunk to signal HTTP response completion */
     httpd_resp_sendstr_chunk(req, NULL);
     return ESP_OK;
 }

#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

/* Set HTTP response content type according to file extension */
esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

/* Handler to download a file kept on the server */
esp_err_t download_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    /* Skip leading "/upload" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri + sizeof("/download") - 1, sizeof(filepath));
    // const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
    //                                          req->uri, sizeof(filepath));
    
    if (!filename) {
        ESP_LOGE(TAG_FS, "Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* If name has trailing '/', respond with directory contents */
    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGI(TAG_FS, "Show conent of dir %s ", filepath);
        return http_resp_dir_html(req, filepath);
    }

    if (stat(filepath, &file_stat) == -1) {
        /* If file not present on SPIFFS check if URI
         * corresponds to one of the hardcoded paths */
        if (strcmp(filename, "/index.html") == 0) {
            return index_html_get_handler(req);
        } else if (strcmp(filename, "/favicon.ico") == 0) {
            return favicon_get_handler(req);
        }
        ESP_LOGE(TAG_FS, "Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE(TAG_FS, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_FS, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

        if (chunksize > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                fclose(fd);
                ESP_LOGE(TAG_FS, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
               return ESP_FAIL;
           }
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);
    ESP_LOGI(TAG_FS, "File sending complete");

    /* Respond with an empty chunk to signal HTTP response completion */
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* Handler to upload a file onto the server */
esp_err_t upload_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;

    /* Skip leading "/upload" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri + sizeof("/upload") - 1, sizeof(filepath));
    if (!filename) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG_FS, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    /*
    Try to get URL query params
        Read URL query string length and allocate memory for length + 1, * extra byte for null termination 
    */
    bool replace = false;
    char *buf_q = NULL;
    size_t buf_len;
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf_q = malloc(buf_len);
        ESP_RETURN_ON_FALSE(buf_q, ESP_ERR_NO_MEM, TAG, "buffer alloc failed");
        if (httpd_req_get_url_query_str(req, buf_q, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf_q);
            char param[QUERY_KEY_MAX_LEN] = {0};
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf_q, "replace", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => replace=%s", param);
                replace = true;
            }
        }
        free(buf_q);
    }

    /* File cannot be larger than a limit */
    if (req->content_len > MAX_FILE_SIZE) {
        ESP_LOGE(TAG_FS, "File too large : %d bytes", req->content_len);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "File size must be less than "
                            MAX_FILE_SIZE_STR "!");
        /* Return failure to close underlying connection else the
         * incoming file content will keep the socket busy */
        return ESP_FAIL;
    }

    /*
    Check and delete if needed to replace with a new file
    */
    if (replace == true) {
        ESP_LOGE(TAG_FS, "File replace url arg, delete old file and upload new: %s", filepath);
        remove(filepath);
    }
    if (stat(filepath, &file_stat) == 0) {
        ESP_LOGE(TAG_FS, "File already exists : %s", filepath);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File already exists");
        return ESP_FAIL;
    }

    FILE *f_d = fopen(filepath, "wb");
    if (f_d == NULL) {
        ESP_LOGE(TAG_FS, "Failed to create file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_FS, "Receiving file : %s...", filename);

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *buf = ((struct file_server_data *)req->user_ctx)->scratch;
    int received;

    /* Content length of the request gives
     * the size of the file being uploaded */
    int remaining = req->content_len;

    while (remaining > 0) {

        ESP_LOGI(TAG_FS, "Remaining size : %d", remaining);
        /* Receive the file part by part into a buffer */
        if ((received = httpd_req_recv(req, buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry if timeout occurred */
                continue;
            }

            /* In case of unrecoverable error,
             * close and delete the unfinished file*/
            fclose(f_d);
            unlink(filepath);

            ESP_LOGE(TAG_FS, "File reception failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            return ESP_FAIL;
        }

        /* Write buffer content to file on storage */
        if (received && (received != fwrite(buf, 1, received, f_d))) {
            /* Couldn't write everything to file!
             * Storage may be full? */
            fclose(f_d);
            unlink(filepath);

            ESP_LOGE(TAG_FS, "File write failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to write file to storage");
            return ESP_FAIL;
        }

        /* Keep track of remaining size of
         * the file left to be uploaded */
        remaining -= received;
    }

    /* Close file upon upload completion */
    fclose(f_d);
    ESP_LOGI(TAG_FS, "File reception complete");

    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, "File uploaded successfully");
    return ESP_OK;
}

/* Handler to delete a file from the server */
esp_err_t delete_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;

    /* Skip leading "/delete" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri  + sizeof("/delete") - 1, sizeof(filepath));
    if (!filename) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG_FS, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG_FS, "File does not exist : %s", filename);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File does not exist");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_FS, "Deleting file : %s", filename);
    /* Delete file */
    unlink(filepath);

    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_sendstr(req, "File deleted successfully");
    return ESP_OK;
}

/*
    File server part FINISHED
*/

/*
    Auth part
*/
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

/*
    Auth part FINISHED
*/

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

/*
Everything else is working
I (126672) webserver: Load HTML from local store path
I (126672) webserver: LFS HTML Exist: 0
I (126672) webserver: SD HTML Exist: -1
W (126672) webserver: Root index.html is not found at SD Card, use LittleFS!
E (126682) webserver: fread (1) failed for html at path /littlefs/index.html
*/
static void load_index_file_buffer_dyn(char* file_path) {
    ESP_LOGI(TAG, "Load HTML from local store path");
    struct stat st;
    
    char lfs_index_html[64];
    snprintf(lfs_index_html, sizeof(lfs_index_html), "%s/index.html", LFS_MOUNT_POINT);
    char sd_index_html[64];
    snprintf(sd_index_html, sizeof(sd_index_html), "%s/index.html", MOUNT_POINT);

    // ESP_LOGI(TAG, "LFS HTML Exist: %d", stat(lfs_index_html, &st));
    // ESP_LOGI(TAG, "SD HTML Exist: %d", stat(sd_index_html, &st));

    // Actual path, check if exist
    if (file_path != NULL) {
        ESP_LOGI(TAG, "Load HTML from path: %s", file_path);
        if (stat(file_path, &st) == 0) {
            ESP_LOGI(TAG, "HTML exists at path: %s", file_path);
        } else {
            ESP_LOGW(TAG, "HTML is not exist at path: %s", file_path);
            // Always exist with firmware
            file_path = lfs_index_html;
        }
    } else {
        // Serve index html from LittleFS if there is no index at SD Card yet.   
        if (stat(sd_index_html, &st) == 0) {
            file_path = sd_index_html;
            ESP_LOGI(TAG, "Root index.html found at SD Card!");
        } else {
            file_path = lfs_index_html;
            ESP_LOGW(TAG, "Root index.html is not found at SD Card, use LittleFS!");
        }
    }
    
    // File size
    memset((void *)index_html, 0, sizeof(index_html));

    FILE *f_r = fopen(file_path, "r");
    if (f_r != NULL) {
        int cb = fread(index_html, st.st_size, sizeof(index_html), f_r);
        if (cb == 0) {
            // File OK, close after
            ESP_LOGI(TAG, "fread (%d) OK for html at path %s", cb, file_path);
            fclose(f_r);
        } else if (cb == -1) {
            // File not ok, show it
            /*
            Strange log but works
            W (6563) httpd_uri: httpd_uri: URI '/favicon.ico' not found
            I (6563) webserver: Redirecting to root
            I (6643) webserver: Load HTML from local store path
            I (6643) webserver: Root index.html found at SD Card!
            E (6643) webserver: fread (1) failed for html at path /sdcard/index.html
            */
            ESP_LOGE(TAG, "fread (%d) failed for html at path %s", cb, file_path);
            fclose(f_r);
        } else {
            // File not ok, show it
            ESP_LOGE(TAG, "fread (%d) failed for html at path %s", cb, file_path);
            fclose(f_r);
        }
    }
    fclose(f_r);
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
    
    static struct file_server_data *server_data = NULL;

    if (server_data) {
        ESP_LOGE(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }

    /* Allocate memory for server data */
    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, MOUNT_POINT,
            sizeof(server_data->base_path));

    // Set global
    static httpd_handle_t server = NULL;
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
    
    // Now start file server only if local Wifi is open:
    if (found_wifi == true) {
        ESP_LOGI(TAG, "Starting file server when local WiFi is connected!");
        
        /* URI handler for getting uploaded files */
        httpd_uri_t file_download = {
            .uri       = "/download/*",  // Match all URIs of type /path/to/file
            .method    = HTTP_GET,
            .handler   = download_get_handler,
            .user_ctx  = server_data    // Pass server data as context
        };
        httpd_register_uri_handler(server, &file_download);

        /* URI handler for uploading files to server */
        httpd_uri_t file_upload = {
            .uri       = "/upload/*",   // Match all URIs of type /upload/path/to/file
            .method    = HTTP_POST,
            .handler   = upload_post_handler,
            .user_ctx  = server_data    // Pass server data as context
        };
        httpd_register_uri_handler(server, &file_upload);

        /* URI handler for deleting files from server */
        httpd_uri_t file_delete = {
            .uri       = "/delete/*",   // Match all URIs of type /delete/path/to/file
            .method    = HTTP_POST,
            .handler   = delete_post_handler,
            .user_ctx  = server_data    // Pass server data as context
        };
        httpd_register_uri_handler(server, &file_delete);

    } else {
        ESP_LOGW(TAG, "Skip file server if there is no local WiFI available!");
        // Do not use now, but can use FS if AUTH is used
        #if BASIC_AUTH
        httpd_register_basic_auth(server);
        /* URI handler for getting uploaded files */
        httpd_uri_t file_download = {
            .uri       = "/*",  // Match all URIs of type /path/to/file
            .method    = HTTP_GET,
            .handler   = download_get_handler,
            .user_ctx  = server_data    // Pass server data as context
        };
        httpd_register_uri_handler(server, &file_download);
        #endif
    }
    // Do not rerutn server, it is now in global var
    return ESP_OK;
}
