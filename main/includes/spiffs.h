/* SPIFFS filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef __spiffs_h_included
#define __spiffs_h_included

#include "esp_system.h"

esp_err_t init_spiffs(void);
esp_err_t deinit_spiffs(void);
esp_err_t format_spiffs(void);
const char* get_spiffs_partition(void);

#ifdef NOTUSED
void list_dir(const char* path);
#endif


#endif /* __spiffs_h_included */

