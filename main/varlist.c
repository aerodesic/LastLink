/*
 * varlist.c
 *
 * Manage a list of vars.
 */
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "varlist.h"

var_item_t *find_var(var_list_t *varlist, const char *name)
{
    var_item_t *found = NULL;

    if (varlist != NULL) {
        for (var_item_t *var = (var_item_t*) FIRST_LIST_ITEM(varlist); found == NULL && var != NULL; var = NEXT_LIST_ITEM(var, varlist)) {
            if (strcmp(var->name, name) == 0) {
                found = var;
            }
        }
    }

    return found;
}

bool delete_var(var_list_t *varlist, const char *name)
{
    var_item_t *var = find_var(varlist, name);
    if (var != NULL) {
        REMOVE_FROM_LIST(varlist, var);
    }
    return var != NULL;
}

var_item_t *create_var_item(const char* name, const char *value)
{
    var_item_t *item = (var_item_t *) malloc(sizeof(var_item_t) + strlen(name) + 1 + strlen(value) + 1);
    if (item != NULL) {
        char *p = (char *) (item + 1);
        strcpy(p, name);
        item->name = p;
        p += strlen(name) + 1;
        strcpy(p, value);
        item->value = p;
    }
    return item;
}

bool set_var(var_list_t *varlist, const char *name, const char *value)
{
    delete_var(varlist, name);

    var_item_t *var = create_var_item(name, value);
    if (var != NULL) {
        ADD_TO_LIST(varlist, var);
    }
    return var != NULL;
}


void free_var_list(void *param)
{
    var_list_t *varlist = (var_list_t*) param;
 
    while (NUM_IN_LIST(varlist) != 0) {
       var_item_t *item = (var_item_t *) FIRST_LIST_ITEM(varlist);
       REMOVE_FROM_LIST(varlist, item);
       free((void*) item);
    }

    free(varlist);
}

