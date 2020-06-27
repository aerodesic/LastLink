/*
 * varlist.h
 *
 * Manage a list of vars.
 */
#ifndef __varlist_h_included
#define __varlist_h_included

#include <stdbool.h>

#include "listops.h"

/* A variable item (fwd ref) */
typedef struct var_item var_item_t;

/* Define the var item */
typedef struct var_item {
    var_item_t     *next;
    var_item_t     *prev;
    const char     *name;
    const char     *value;
} var_item_t;

/* Head of the var item list */
typedef list_head_t  var_list_t;

bool set_var(var_list_t *var_list, const char *name, const char *value);
var_item_t *find_var(var_list_t *varlist, const char *name);
bool delete_var(var_list_t *varlist, const char *name);
var_item_t *create_var_item(const char* name, const char *value);
void free_var_list(void *param);
var_list_t *create_var_list(void);

#endif /* __varlist_h_included */

