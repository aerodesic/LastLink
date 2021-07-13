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
#include "os_specific.h"

#include "configdata.h"

static const char *TAG = "configdata";

typedef struct configitem {
    struct configitem *next;   /* Next cell in list */
    struct configitem *prev;   /* Previous cell in list */
    struct configitem *owner;  /* Section that owns this cell */
    const char* name;          /* Name of this cell */

    enum {
        CONFIG_SECTION,
        CONFIG_VALUE,
    } type;
    union {
        struct configitem *head;
        const char* value;
    };
} configitem_t;

/* The config table starts out as a naked, un-named section */
static configitem_t config_table = {
    .next = &config_table,
    .prev = &config_table,
    .name = "base_config_section",
    .type = CONFIG_SECTION,
};

static os_mutex_t config_lock;
static int config_locked = 0;
static int config_changes = 0;
static const char* config_file_name = NULL;

static configitem_t* find_config_entry(const char* name, configitem_t* section, bool create);
static char* skip_blanks(char* bufp);
static esp_err_t release_config(configitem_t* table);
static void write_config_value(FILE* fp, configitem_t* cell, int indent);
static configitem_t* add_config_cell(configitem_t* section);
static esp_err_t load_config_table(FILE* fp, configitem_t* section, char* buffer, size_t bufferlen);

static char* skip_blanks(char* bufp)
{
    while (isspace(*bufp)) {
        ++bufp;
    }
    return bufp;
}

/* Iterate through the cells in this group and print their contents */
static void dump_cells(const char* title, configitem_t* section, int indent)
{
    if (section != NULL && section->head != NULL) {
        if (title != NULL) {
            printf("%s owner %p\n", title, section->owner);
        }
        configitem_t *cell = section->head;
        do {
            if (cell->type == CONFIG_SECTION) {
                if (cell->head != NULL) {
                    printf("%*.*s%p: section '%s' owner %p %p\n", indent, indent, "", cell, cell->name, cell->owner, cell->head);
                    dump_cells(NULL, cell, indent+5);
                }
            } else if (cell->type == CONFIG_VALUE) {
                printf("%*.*s%p: value '%s' owner %p \"%s\"\n", indent, indent, "", cell, cell->name, cell->owner, cell->value);
            }
            cell = cell->next;
        } while (cell != section->head);
    }
}

/* Release an item */
static esp_err_t release_config(configitem_t* item)
{
    esp_err_t ret = ESP_OK;

    if (item != NULL) {
        if (item->type == CONFIG_VALUE) {
            /* Just free the value */
            free((void*) item->value);
            item->value = NULL;
        } else if (item->type == CONFIG_SECTION) {
            /* Release the section list */
            while (item->head != NULL) {
                release_config(item->head);
            }
        }

        /* Unlink current item from parent's list */
        item->prev->next = item->next;
        item->next->prev = item->prev;

        /* Item owner MUST be a section.  Check and if section head points to this item, advance the pointer */
        if (item->owner->type != CONFIG_SECTION) {
            /* Owner of an item MUST be a section */
            ESP_LOGE(TAG, "%s: owner of %s (%s) is not a section", __func__, item->name, item->owner->name);
        } else if (item->owner->head == item) {
            /* We are removing the head of the owner's list.  moved to next */
            item->owner->head = item->next;
            /* If still points to this item, clear owner list */
            if (item->owner->head == item) {
                item->owner->head = NULL;
            }
        } 

        /* Free the name */
//ESP_LOGI(TAG, "%s: releasing %s", __func__, item->name);
        free((void*) item->name);

        /* And free the item */
        free((void*) item);

        ++config_changes;
    } else {
        ret = ESP_FAIL;
    }

    return ret;
}

static void write_config_value(FILE* fp, configitem_t* cell, int indent)
{
    if (fp != NULL && cell != NULL) {

        configitem_t* start = cell;

        do {
            if (cell->type == CONFIG_SECTION) {

                fprintf(fp, "%*.*s[%s]\n", indent, indent, "", cell->name);
                if (cell->head != NULL) {
                    write_config_value(fp, cell->head, indent + 4);
                }
                fprintf(fp, "%*.*s[end]\n", indent, indent, "");

            } else if (cell->type == CONFIG_VALUE) {

                if (cell->value != NULL) {
                    fprintf(fp, "%*.*s%s=%s\n", indent, indent, "", cell->name, cell->value);
                }

            }

            cell = cell->next;

        } while (cell != start);
    }
}

static configitem_t* add_config_cell(configitem_t* section)
{
    configitem_t* cell = (configitem_t*) malloc(sizeof(configitem_t));
    if (cell != NULL) {
        if (section->head == NULL) {
            /* First entry on new table */
            cell->next = cell;
            cell->prev = cell;
            section->head = cell;
//ESP_LOGI(TAG, "%s: first cell at %p in section %s", __func__, cell, section->name);
        } else {
            /* Insert it at end of owner */
//ESP_LOGI(TAG, "%s: appending cell at %p to section %s", __func__, cell, section->name);
            cell->next = section->head;
            cell->prev = section->head->prev;
            section->head->prev->next = cell;
            section->head->prev = cell;
        }
        cell->owner = section;
    }
    return cell;
}

static esp_err_t load_config_table(FILE *fp, configitem_t* section, char* buffer, size_t bufferlen)
{
    esp_err_t ret = ESP_OK;
    bool done = false;

//ESP_LOGI(TAG, "load_config_table owner into section %s", section->name);
    /* Read the file and parse the values */
    while (!done  && (ret == ESP_OK) && (fgets(buffer, bufferlen, fp) != NULL)) {
        char* p = strchr(buffer, '\n');
        if (p != NULL) {
            *p = '\0';
        }
        p = skip_blanks(buffer);
        /* Ignore blank lines */
        if (*p != 0) {
//ESP_LOGI(TAG, "load: %s", buffer);
            if (*p == '[') {
                /* Start of a section */
                char* name = skip_blanks(p+1);
                char* endp = strchr(name, ']');
                if (*endp != '\0') {
                    *endp = '\0';
                }
                if (strcmp(name, "end") != 0) {
                    configitem_t* cell = add_config_cell(section);
                    if (cell != NULL) {
                        /* Create a section at this point*/
                        cell->name = strdup(name);
                        cell->type = CONFIG_SECTION;
                        cell->head = NULL;

//ESP_LOGI(TAG, "start section in cell %p", cell);
                        ret = load_config_table(fp, cell, buffer, bufferlen);
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
                        configitem_t* cell = add_config_cell(section);
                        if (cell != NULL) {
                            cell->name = strdup(name);
                            cell->type = CONFIG_VALUE;
                            cell->value = strdup(p);
                        } else {
                            ret = ENOMEM;
                        }
                    }
                }
            }
        }
    }

//dump_cells("after load", &config_table, 0);
    // ESP_LOGI(TAG, "load_config_table exit %d", ret);

    return ret;
}

bool write_config(FILE* fp)
{
    if (fp != NULL) {
        write_config_value(fp, config_table.head, 0);
    }
    return fp != NULL;
}

bool save_config(const char* filename)
{
    bool ok = true;

    FILE* fp;

    if (filename != NULL) {
        fp = fopen(filename, "w");
        // ESP_LOGI(TAG, "%s opened on %p", filename, fp);

        if (fp != NULL) {
            write_config_value(fp, config_table.head, 0);
            fclose(fp);
        }
    } else {
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
    if (os_acquire_recursive_mutex(config_lock)) {
        ++config_locked;
        return true;
    }

    return false;
}

bool unlock_config(void)
{
    if (os_release_recursive_mutex(config_lock)) {
        if (--config_locked == 0) {
            if (config_changes != 0) {
                save_config(config_file_name);
                config_changes = 0;
            }
        }
        return true;
    }

    return false;
}

/*
 * init_configuration
 *
 * Opens and reads the configuration data file into internal storage.
 *
 * Entry:
 *       filename       File containing configuration.
 *
 * Returns:
 *       esp_err_t      ESP_OK if all went well, otherwise an error code
 */
esp_err_t init_configuration(const char* filename)
{
    esp_err_t ret = ESP_OK;

    config_lock = os_create_recursive_mutex();

    free((void*) config_file_name);

    config_file_name = strdup(filename);

    if (lock_config())
    {
        FILE *fp = fopen(filename, "r");

        if (fp != NULL) {
            char buffer[50];
            // ESP_LOGI(TAG, "init_configuration from file");
            ret = load_config_table(fp, &config_table, buffer, sizeof(buffer));
            fclose(fp);
        }

        unlock_config();
    }

    save_config(NULL);

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
 *     section     section to search for item
 *     create      true if to create entry if not found
 */
static configitem_t* find_config_entry(const char* name, configitem_t* section, bool create)
{
    const char* delim = ".";
    const char* field;
    configitem_t* found = NULL;

    char* str = strdup(name);
    char* tokens = str;
    char* rest;

// dump_cells("Table", section, 0);

    bool done = false;
    while (!done && (field = strtok_r(tokens, delim, &rest)) != NULL) {
        tokens = NULL;
        found = NULL;

        // ESP_LOGI(TAG, "find_config_item looking for '%s'", field);

        configitem_t* item = section->head;

        if (item != NULL) {
            do {
                //ESP_LOGI(TAG, "%s: looking for '%s' at %p '%s'", __func__, field, item, item->name);
                if (strcmp(item->name, field) == 0) {
                    /* Found field */
                    //ESP_LOGI(TAG, "%s: found at %p type %d", __func__, item, item->type);
                    found = item;
                }
                item = item->next;
            } while (!found && item != section->head);
        }

        if (found != NULL) {
            if (found->type == CONFIG_SECTION) {
                section = found;
            }
        } else {
            /* Give up */
            done = true;
        }
    }

    if (found == NULL && create) {
        /* Create rest of structure here */
        do {
//ESP_LOGI(TAG, "%s: adding cell for %s", __func__, field);
            found = add_config_cell(section);

            if (found != NULL) {
                found->name = strdup(field);
                found->type = CONFIG_SECTION;   /* Assume section until we run out of fields */
                found->head = NULL;             /* No contents */
                section = found;                /* Next owner cell */

                field =  strtok_r(tokens, delim, &rest);
            } else {
                field = NULL;
            }
        } while (field != NULL);
             
        if (found != NULL) {
            /* Turn last found into a VALUE */
            found->type = CONFIG_VALUE;
            found->value = strdup(""); 
        }
    }

    free((void*) str);

    return (found);
}

bool delete_config(const char* field)
{
    configitem_t* item = find_config_entry(field, &config_table, false);
    if (item != NULL) {
        release_config(item);
//dump_cells("after delete", &config_table, 0);
    }

    return item != NULL;
}

const char* get_config_str(const char* field, const char* defvalue)
{
    const char* ret = NULL;

    configitem_t* item = find_config_entry(field, &config_table, false);
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
    configitem_t* item = find_config_entry(field, &config_table, true);
    
    if (item != NULL) {
        if (strcmp(item->value, value) != 0) {
            free((void*) (item->value));
            item->value = strdup(value);
            config_changes++;
//dump_cells("after set", &config_table, 0);
        }
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

