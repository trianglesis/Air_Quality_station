#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>

#include "esp_err.h"
#include "esp_log.h"

#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"

/* Max length a file path can have on storage */
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

/* Max size of an individual file. Make sure this
 * value is same as that set in upload_script.html */
#define MAX_FILE_SIZE   (500*1024) // 500 KB
#define MAX_FILE_SIZE_STR "500KB"

/* Scratch buffer size */
#define SCRATCH_BUFSIZE  8192

struct file_server_data {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};

esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath);
esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename);

esp_err_t download_get_handler(httpd_req_t *req);
esp_err_t upload_post_handler(httpd_req_t *req);
esp_err_t delete_post_handler(httpd_req_t *req);

esp_err_t start_file_server(void);
