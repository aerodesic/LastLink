/*
 * flash_file_system.h
 */
#ifndef __flash_file_system_h_included
#define __flash_file_system_h_included

#include "sdkconfig.h"

const esp_partition_t* add_partition(esp_flash_t* ext_flash, const char* partition_label);
void list_data_partitions(void);
bool mount_fatfs(const char* partition_label);
void get_fatfs_usage(size_t* out_total_bytes, size_t* out_free_bytes);
bool init_flash_file_system(void);

#endif /* __flash_file_system_h_included */

