/* SPIFFS filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <stdbool.h>

#include <sys/time.h>
#include <errno.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <dirent.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "spiffs.h"

static const char *TAG = "spiffs";

static esp_vfs_spiffs_conf_t spiffs_conf = {
   .base_path = "/spiffs",
   .partition_label = CONFIG_SPIFFS_PARTITION_NAME,
   .max_files = 5,
   .format_if_mount_failed = true
};
    
const char* get_spiffs_partition(void)
{
    return spiffs_conf.partition_label ? spiffs_conf.partition_label : "Unknown";
}

esp_err_t init_spiffs(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing SPIFFS");
    
    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    ret = esp_vfs_spiffs_register(&spiffs_conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    } else {
        size_t total = 0, used = 0;
        ret = esp_spiffs_info(spiffs_conf.partition_label, &total, &used);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
        }
    }

    return ret;
}

    
esp_err_t deinit_spiffs(void)
{
    // All done, unmount partition and disable SPIFFS
    int ok = esp_vfs_spiffs_unregister(spiffs_conf.partition_label);

    ESP_LOGI(TAG, "SPIFFS unmounted");

    return ok;
}

esp_err_t format_spiffs(void)
{
    ESP_LOGI(TAG, "Formatting '%s'", spiffs_conf.partition_label);

    deinit_spiffs();

    esp_err_t ret = esp_spiffs_format(spiffs_conf.partition_label);

    if (ret == ESP_OK) {
	ret = init_spiffs();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format partition (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGE(TAG, "Format partition ok");
    }

    return ret;
}

#ifdef NOTUSED
void list_dir(const char* path)
{
    DIR *dir = NULL;
    struct dirent *ent;
    char type;
    char size[20];
    char tpath[255];
    char tbuffer[80];
    struct stat sb;
    struct tm *tm_info;
    int statok;

    printf("\nList of Directory [%s]\n", path);
    printf("-----------------------------------\n");
    // Open directory
    dir = opendir(path);
    if (!dir) {
        printf("Error opening directory\n");
        return;
    }

    // Read directory entries
    uint64_t total = 0;
    int nfiles = 0;
    printf("T  Size      Date/Time         Name\n");
    printf("-----------------------------------\n");
    while ((ent = readdir(dir)) != NULL) {
        sprintf(tpath, path);
        if (path[strlen(path)-1] != '/') {
            strcat(tpath,"/");
        }
        strcat(tpath,ent->d_name);
        tbuffer[0] = '\0';

        // Get file stat
        statok = stat(tpath, &sb);

        if (statok == 0) {
            tm_info = localtime(&sb.st_mtime);
            strftime(tbuffer, 80, "%d/%m/%Y %R", tm_info);
        }
        else sprintf(tbuffer, "                ");

        if (ent->d_type == DT_REG) {
            type = 'f';
            nfiles++;
            if (statok) strcpy(size, "       ?");
            else {
                total += sb.st_size;
                if (sb.st_size < (1024*1024)) {
                    snprintf(size, sizeof(size), "%8d", (int)sb.st_size);
                } else if ((sb.st_size/1024) < (1024*1024)) {
                    snprintf(size, sizeof(size), "%6dKB", (int)(sb.st_size / 1024));
                } else {
                    snprintf(size, sizeof(size), "%6dMB", (int)(sb.st_size / (1024 * 1024)));
                }
            }
        }
        else {
            type = 'd';
            strcpy(size, "       -");
        }

        printf("%c  %s  %s  %s\r\n",
               type,
               size,
               tbuffer,
               ent->d_name
        );
    }

    if (total) {
        printf("-----------------------------------\n");
            if (total < (1024*1024)) printf("   %8d", (int)total);
            else if ((total/1024) < (1024*1024)) printf("   %6dKB", (int)(total / 1024));
            else printf("   %6dMB", (int)(total / (1024 * 1024)));
            printf(" in %d file(s)\n", nfiles);
    }
    printf("-----------------------------------\n");

    closedir(dir);

    uint32_t tot=0, used=0;
    esp_spiffs_info(NULL, &tot, &used);
    printf("SPIFFS: free %d KB of %d KB\n", (tot-used) / 1024, tot / 1024);
    printf("-----------------------------------\n\n");
}
#endif

