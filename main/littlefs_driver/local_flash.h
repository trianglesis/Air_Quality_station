// 
#include "esp_log.h"
#include "esp_err.h"
// File system
#include "esp_system.h"
#include "esp_littlefs.h"


#define LFS_MOUNT_POINT "/littlefs"
#define LFS_PARTITION_LABEL "littlefs"

extern float littlefs_total;
extern float littlefs_used;

esp_err_t fs_setup(void);
esp_err_t fs_read(void);
