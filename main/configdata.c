/*
 * configdata.c
 *
 * Storage for configuration information.
 */
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>
#include <sdkconfig.h>
#include <errno.h>

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "configdata.h"

static const char *TAG = "configdata";

typedef struct configitem {
    struct configitem *next;
    struct configitem *prev;
    struct configitem **owner;
    const char* name;

    enum {
        CONFIG_SECTION,
        CONFIG_VALUE,
    } type;
    union {
        struct configitem *section;
        const char* value;
    };
} configitem_t;

typedef struct rdinfo {
    char* (*read)(struct rdinfo*, char* buffer, size_t length);
    union {
        FILE* fp;
        const char** bufp;
    };
} rdinfo_t;

static int config_locked;
static int config_changes = 0;
static const char* config_file_name = NULL;
static configitem_t* config_table;

static configitem_t* find_config_entry(const char* name, configitem_t** parent, configitem_t* table);
static char* skip_blanks(char* bufp);
static esp_err_t delete_config_cell(configitem_t* item);
static esp_err_t release_config(configitem_t** table);
static void write_config(FILE* fp, configitem_t* cell, int indent);
static char* read_from_file(rdinfo_t* rdinfo, char* buffer, size_t length);
static char* read_from_table(rdinfo_t* rdinfo, char* buffer, size_t length);
static configitem_t* add_config_cell(configitem_t** owner, const char* info);
static esp_err_t load_config_table(configitem_t** owner, rdinfo_t* rdinfo, char* buffer, size_t bufferlen);
static configitem_t* find_config_entry(const char* name, configitem_t** parent, configitem_t* table);

static char* skip_blanks(char* bufp)
{
    while (isspace(*bufp)) {
        ++bufp;
    }
    return bufp;
}

static esp_err_t delete_config_cell(configitem_t* item)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "delete_config_cell: '%s' type %d", item->name, item->type);

    /* Remove item from table */
    item->next->prev = item->prev;
    item->prev->next = item->next;

    configitem_t* next = item->next;

    if (item->type == CONFIG_SECTION) {
        /* Recursively delete section */
        ret = release_config(&item->section);
    } else if (item->type == CONFIG_VALUE) {
        free((void*) item->value);
    }

    /* If we are removing the cell pointed to by the owner's pointer,
     * we null it out of the list is now empty otherwise we move
     * the pointer on to the next item.
     */
    if (*(item->owner) == item) {
        *(item->owner) = (item->next) == next ? NULL : next;
    }

    free((void*) item->name);

    free((void*) item);

    return ret;
}

static esp_err_t release_config(configitem_t** table)
{
    esp_err_t ret = ESP_OK;

    configitem_t* item = *table;

    while ((ret == ESP_OK) && (item != NULL)) {
        ESP_LOGD(TAG, "'%s' item %p next %p prev %p", item->name, item, item->next, item->prev);
        configitem_t* nextitem = item->next;
        /* If deleting the last item, call it empty after deletion */
        bool empty = nextitem == item;
        ret = delete_config_cell(item);
        ESP_LOGD(TAG, "list is %s", empty ? "empty" : "not empty");
        item = empty ? NULL : nextitem;
    }

    *table = NULL;

    return ret;
}

static void write_config(FILE* fp, configitem_t* cell, int indent)
{
    if (cell != NULL) {

        configitem_t* start = cell;

        do {
            if (cell->type == CONFIG_SECTION) {
                if (fp == NULL) {
                    ESP_LOGI(TAG, "%*.*s[%s] (owned by %p)", indent, indent, "", cell->name, cell->owner);
                } else {
                    fprintf(fp, "%*.*s[%s]\n", indent, indent, "", cell->name);
                }

                write_config(fp, cell->section, indent + 4);

                if (fp == NULL) {
                    ESP_LOGI(TAG, "%*.*s[end] (of '%s' owned by %p)", indent, indent, "", cell->name, cell->owner);
                } else {
                    fprintf(fp, "%*.*s[end]\n", indent, indent, "");
                }
            } else if (cell->type == CONFIG_VALUE) {
                if (fp == NULL) {
                    ESP_LOGI(TAG, "%*.*s%s=%s (owned by %p)", indent, indent, "", cell->name, cell->value, cell->owner);
                } else {
                    fprintf(fp, "%*.*s%s=%s\n", indent, indent, "", cell->name, cell->value);
                }
            }

            cell = cell->next;

        } while (cell != start);
    }
}

static char* read_from_file(rdinfo_t* rdinfo, char* buffer, size_t length)
{
    return fgets(buffer, length, rdinfo->fp);
}

static char* read_from_table(rdinfo_t* rdinfo, char* buffer, size_t length)
{
    char* ret;
    if (*(rdinfo->bufp) == NULL) {
        ret = NULL;
    } else {
        strncpy(buffer, *(rdinfo->bufp), length);
        buffer[length-1] = 0;
        rdinfo->bufp++;
        ret = buffer;
    }
    return ret;
}

static configitem_t* add_config_cell(configitem_t** owner, const char* info)
{
    configitem_t* cell = (configitem_t*) malloc(sizeof(configitem_t));
    if (cell != NULL) {
        if (*owner == NULL) {
            /* First entry on new table */
            cell->next = cell;
            cell->prev = cell;
            *owner = cell;
            ESP_LOGD(TAG, "new cell '%s' at %p owned by list %p", info, cell, owner);
        } else {
            /* Insert it at end of owner */
            ESP_LOGD(TAG, "appending '%s' at %p to owner by %p at %p", info, cell, *owner, (*owner)->prev);
            cell->next = *owner;
            cell->prev = (*owner)->prev;
            (*owner)->prev->next = cell;
            (*owner)->prev = cell;
        }
        cell->owner = owner;
    }
    return cell;
}

static esp_err_t load_config_table(configitem_t** owner, rdinfo_t* rdinfo, char* buffer, size_t bufferlen)
{
    esp_err_t ret = ESP_OK;
    bool done = false;
    *owner = NULL;

    // ESP_LOGI(TAG, "load_config_table owner into %p -> %p", owner, *owner);
    /* Read the file and parse the values */
    while (!done  && (ret == ESP_OK) && (rdinfo->read(rdinfo, buffer, bufferlen) != NULL)) {
        char* p = strchr(buffer, '\n');
        if (p != NULL) {
            *p = '\0';
        }
        p = skip_blanks(buffer);
        /* Ignore blank lines */
        if (*p != 0) {
            // ESP_LOGI(TAG, "load: %s", buffer);
            if (*p == '[') {
                /* Start of a section */
                char* name = skip_blanks(p+1);
                char* endp = strchr(name, ']');
                if (*endp != '\0') {
                    *endp = '\0';
                }
                if (strcmp(name, "end") != 0) {
                    configitem_t* cell = add_config_cell(owner, p);
                    if (cell != NULL) {
                        /* Create a section at this point*/
                        cell->name = strdup(name);
                        cell->type = CONFIG_SECTION;
                        cell->section = NULL;

                        // ESP_LOGI(TAG, "start section in cell %p", cell);
                        ret = load_config_table(&cell->section, rdinfo, buffer, bufferlen);
                    } else {
                        ret = ENOMEM;
                    }
                } else {
                    done = true;
                }
            } else {
                /* Entry */
                char* name = p;
                p = strchr(p, '=');
                if (p != NULL) {
                    *p = '\0';
                    p = skip_blanks(p+1);
                    if (p != NULL) {
                        configitem_t* cell = add_config_cell(owner, name);
                        if (cell != NULL) {
                            char* value = strdup(p);
    
                            cell->name = strdup(name);
                            cell->type = CONFIG_VALUE;
                            cell->value = value;
                        } else {
                            ret = ENOMEM;
                        }
                    }
                }
            }
        }
    }
    // ESP_LOGI(TAG, "load_config_table exit %d", ret);

    return ret;
}

bool save_config(const char* filename)
{
    bool ok = true;

    // ESP_LOGI(TAG, "saving config to '%s'", filename);
    FILE* fp;

    if (filename != NULL) {
        fp = fopen(filename, "w");
        // ESP_LOGI(TAG, "%s opened on %p", filename, fp);
    } else {
        fp = NULL;
    }

    if (filename == NULL || fp != NULL) {
        write_config(fp, config_table, 0);
        if (fp != NULL) {
            fclose(fp);
        }
    } else {
        // ESP_LOGI(TAG, "unable to save config");
        ok = false;
    }

    return ok;
}

bool close_config(void)
{
    esp_err_t ret = ESP_OK;

    if (lock_config()) {

        release_config(&config_table);

        unlock_config();
    }

    return ret;
}

bool lock_config(void)
{
    ++config_locked;

    return true;
}

bool unlock_config(void)
{
    if (--config_locked == 0)
    {
        if (config_changes != 0) {
            save_config(config_file_name);
            config_changes = 0;
        }
    }

    return true;
}

/*
 * init_configuration
 *
 * Opens and reads the configuration data file into internal storage.
 *
 * Entry:
 *       filename       File containing configuration.
 *       default_config Default configuration text for preloading.
 *
 * Returns:
 *       esp_err_t      ESP_OK if all went well, otherwise an error code
 */
esp_err_t init_configuration(const char* filename, const char **default_config)
{
    esp_err_t ret = ESP_OK;

    // ESP_LOGI(TAG, "init_configuration '%s'", filename);
    config_file_name = strdup(filename);

    if (lock_config())
    {
        /* Build head of config table (this is the root 'section') */
        configitem_t* new_config;

        rdinfo_t rdinfo;
        char buffer[50];
        bool reset_default = true;

        rdinfo.read = read_from_table;
        rdinfo.bufp = default_config;

        esp_err_t ret1 = load_config_table(&new_config, &rdinfo, buffer, sizeof(buffer));

        FILE *fp = fopen(filename, "r");

        if (fp != NULL) {
            // ESP_LOGI(TAG, "init_configuration from file");
            rdinfo.read = read_from_file;
            rdinfo.fp = fp;
            esp_err_t ret2 = load_config_table(&config_table, &rdinfo, buffer, sizeof(buffer));
            fclose(fp);

            /* If both reads ok, the  compare versions and don't switch if version matches */
            if (ret1 == ESP_OK && ret2 == ESP_OK) {

                configitem_t* new_version = find_config_entry("configversion", NULL, new_config);
                configitem_t* old_version = find_config_entry("configversion", NULL, config_table);

                if (new_version != NULL && new_version->type == CONFIG_VALUE &&
                    old_version != NULL && old_version->type == CONFIG_VALUE) {

                    ESP_LOGD(TAG, "old_version = %s  new_version = %s", old_version->value, new_version->value);

                    if (strtol(new_version->value, NULL, 0) == strtol(old_version->value, NULL, 0)) {

                        /* Same version, so do not reset */
                        reset_default = false;
                    }
                }
            }
        }

        if (reset_default) {
            ESP_LOGD(TAG, "init_configuration from default");
            release_config(&config_table);
            config_table = new_config;
            save_config(config_file_name);
        } else {
            ESP_LOGD(TAG, "init_configuration from file");
            release_config(&new_config);
        }

        unlock_config();
    }

    /* Write messages to logfile */
#ifdef DEBUG
    save_config(NULL);
#endif

    return ret;
}

/*
 * Search config table recursively looking for name.
 *
 * Entry:
 *     name        Config table entry name, with '.' delimiting sections
 *                 e.g. "wifi.default.password"
 *                 Refers to:
 *                    [wifi]
 *                       [default]
 *                           password=something or other
 *                       [end]
 *                    [end]
 *     parent      If non-null, receives pointer to section that contains this item.
 *     table       Table to search
 */
static configitem_t* find_config_entry(const char* name, configitem_t** parent, configitem_t* table)
{
    const char* delim = ".";
    const char* field;
    configitem_t* found = NULL;

    char* str = strdup(name);
    char* tokens = str;

    while ((table != NULL) && (field = strtok(tokens, delim)) != NULL) {
        str = NULL;
        tokens = NULL;
	found = NULL;

        ESP_LOGD(TAG, "find_config_item looking for '%s'", field);

        configitem_t* item = table;
        do {
            ESP_LOGD(TAG, "looking for '%s' at %p '%s'", field, item, item->name);
            if (strcmp(item->name, field) == 0) {
                /* Found field */
                ESP_LOGD(TAG, "found at %p type %d", item, item->type);
                found = item;
            }
            item = item->next;
        } while (!found && item != table);

        if (found) {
            if (found->type == CONFIG_SECTION) {
                if (parent != NULL) {
                    *parent = table;
                }
                table = found->section;
            }
        } else {
            table = NULL;
        }
    }

    free((void*) str);

    return (found && found->type == CONFIG_VALUE) ? found : NULL;
}

bool delete_config(const char* field)
{
    configitem_t* parent;

    configitem_t* item = find_config_entry(field, &parent, config_table);
    if (item != NULL) {
        /* Remove the item from the list */
        item->next->prev = item->prev;
        item->prev->next = item->next;

        /* parent->type *MUST* be a SECTION, but check anyway */
        if (parent->type == CONFIG_SECTION) {
            if (parent->section == parent) {
                parent->section = NULL;
            }
        } else {
            ESP_LOGI(TAG, "************ parent of %s not a section (%s) *************", item->name, parent->name);
        }

        free((void*) item);
    }

    return item != NULL;
}

const char* get_config_str(const char* field, const char* defvalue)
{
    const char* ret = NULL;

    configitem_t* item = find_config_entry(field, NULL, config_table);
    if (item != NULL && item->type == CONFIG_VALUE) {
        ret = item->value;
    } else {
        ret = defvalue;
    }

    return ret;
}

int get_config_int(const char* field, int defvalue)
{
    const char* str = get_config_str(field, NULL);
    if (str != NULL) {
        defvalue = strtol(str, NULL, 0);
    }
    return defvalue;
}

bool set_config_str(const char* field, const char* value)
{
    configitem_t* item = find_config_entry(field, NULL, config_table);
    if (item != NULL) {
        free((void*) (item->value));
        item->value = strdup(value);
    }
    return item != NULL;
}

bool set_config_int(const char* field, int value)
{
    char* buf;
    bool ok;

    asprintf(&buf, "%d", value);
    ok = set_config_str(field, buf);
    free((void*) buf);

    return ok;
}

