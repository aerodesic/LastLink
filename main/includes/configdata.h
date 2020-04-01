/*
 * configdata.h
 *
 * Storage for configuration information.
 */
#ifndef __configdata_h_included
#define __configdata_h_included

#include <stdbool.h>
#include <stdio.h>

#include "esp_system.h"

esp_err_t init_configuration(const char* filename, const char** default_config);
bool lock_config(void);
bool unlock_config(void);
bool save_config(const char* filename);
bool write_config(FILE* fp);

const char* get_config_str(const char* field, const char* defvalue);
int get_config_int(const char* field, int defvalue);

bool set_config_str(const char* field, const char* value);
bool set_config_int(const char* field, int value);

bool delete_config(const char* field);

#endif /* __configdata_h_included */

