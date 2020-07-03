/*
 * httpd_templates.c
 */

#include <esp_log.h>

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/param.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "os_specific.h"
#include "listops.h"
#include "tokenize.h"
#include "varlist.h"
#include "expressions.h"
#include "http_templates.h"

static const char *TAG = __FILE__;

typedef struct include_stack_item include_stack_item_t;
typedef struct include_stack_item {
    include_stack_item_t    *next;
    include_stack_item_t    *prev;
    char                    filename[1];
} include_stack_item_t;

include_stack_t *create_include_stack(void)
{
    include_stack_t *include_stack = (include_stack_t*) malloc(sizeof(include_stack_t));
    if (include_stack != NULL) {
        INIT_LIST(include_stack);
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return include_stack;
}

void free_include_stack(include_stack_t *include_stack)
{
    while (NUM_IN_LIST(include_stack) != 0) {
        include_stack_item_t *item = (include_stack_item_t*) FIRST_LIST_ITEM(include_stack);
        REMOVE_FROM_LIST(include_stack, item);
        free((void*) item);
    }
#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}


typedef struct command_item command_item_t;
typedef struct command_item {
    command_item_t      *next;
    command_item_t      *prev;

    function_argv_t     function_argv;
    function_rawargs_t  function_rawargs;
    char                name[1];
} command_item_t;

typedef list_head_t     command_list_t;

static command_list_t   command_list;

static void init_command_list(void);

static command_item_t *find_command(const char *name)
{
    command_item_t *command = (command_item_t*) FIRST_LIST_ITEM(&command_list);
    while (command != NULL) {
        if (strcmp(command->name, name) == 0) {
            return command;
        } else {
            command = NEXT_LIST_ITEM(command, &command_list);
        }
    }
    return NULL;
}

static command_item_t *create_command_entry(const char *name)
{
    command_item_t *command = NULL;

    if (find_command(name) == NULL) {
        command = (command_item_t*) malloc(sizeof(command_item_t) + strlen(name));
        if (command != NULL) {
            strcpy(command->name, name);
            ADD_TO_LIST(&command_list, command);
        }
    }
#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return command;
}

bool add_template_command_argv(const char *name, function_argv_t function)
{
    command_item_t *command = create_command_entry(name);

    if (command != NULL) {
        command->function_rawargs = NULL;
        command->function_argv = function;
    }

    return command != NULL;
}

bool add_template_command_rawargs(const char *name, function_rawargs_t function)
{
    command_item_t *command = create_command_entry(name);

    if (command != NULL) {
        command->function_argv = NULL;
        command->function_rawargs = function;
    }

    return command != NULL;
}

void free_session_context(void *param)
{
ESP_LOGI(TAG, "%s: called", __func__);
    session_context_t *session = (session_context_t *) param;
    free_var_list(session->varlist);
    free_include_stack(session->include_stack);

    /* Free private context if present */
    if (session->private_context != NULL) {
        if (session->free_private_context != NULL) {
            session->free_private_context(session->private_context);
        } else {
            free((void*) session->private_context);
        }
        session->private_context = NULL;
    }

    free((void*) session);
}

session_context_t *create_session_context(void)
{
    session_context_t *session = (session_context_t*) malloc(sizeof(session_context_t));
    if (session != NULL) {
        session->varlist = create_var_list();
        session->include_stack = create_include_stack();
        session->private_context = NULL;
        session->free_private_context = NULL;
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return session;
}

bool set_session_var(session_context_t *session, const char *name, const char *value)
{
    assert(session != NULL);
    return set_var(session->varlist, name, value);
}

bool delete_session_var(session_context_t *session, const char *name)
{
    assert(session != NULL);
    return delete_var(session->varlist, name);
}

bool get_session_var(session_context_t *session, const char *name, const char **value)
{
    assert(session != NULL);
    var_item_t *var = find_var(session->varlist, name);
    if (var != NULL) {
        *value = var->value;
    }
    return var != NULL;
}

static var_item_t *eval_get_session_var(void *context, const char *name)
{
    assert(context != NULL);
    return find_var(((session_context_t *) context)->varlist, name);
}

/*
 * This needs to be smarter so it can skip over an escape '\' character, but for now, punt.
 */
char *find_string(char *buffer, const char *string)
{
    return strstr(buffer, string);
}

/*
 * Fetch a symbol name from a char**.  Must start with alpha and contain only alphanumerics and _
 *
 * Returns pointer to string if symbol was fetched.
 */
char *fetch_symbol(const char **ptr, char *buffer, size_t bufsize)
{
    int len = 0;

    /* Skip leanding spaces */
    while (isspace(**ptr)) {
       ++(*ptr);
    }

    if (isalpha(**ptr)) {
        while (len < bufsize - 1 && isalnum(**ptr)) {
            buffer[len++] = *(*ptr)++;
        }
        buffer[len] = '\0';
    }

    return len != 0 ? buffer : NULL;
}

/*
 * Create a full pathname from a uri, adding '.html' if necessary
 */
const char* get_pathname_from_uri(const char *uri, const char *prefixdir, char *temp_buffer, size_t temp_buffer_len)
{
    if (strcmp(uri, "/") == 0) {
        uri = "/index";
    }

    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest != NULL) {
        pathlen = MIN(pathlen, quest - uri);
    }

    const char *hash = strchr(uri, '#');
    if (hash != NULL) {
        pathlen = MIN(pathlen, hash - uri);
    }

    size_t baselen = strlen(CONFIG_LASTLINK_HTML_DIRECTORY);
    if (prefixdir != NULL) {
        baselen += strlen(prefixdir);
    }

    pathlen += 6;   /* Room for additional ".html" <nul> */

    const char *pathname = NULL;

    if ((pathlen + baselen) < temp_buffer_len) {
        strcpy(temp_buffer, CONFIG_LASTLINK_HTML_DIRECTORY);
        if (prefixdir != NULL) {
           strcat(temp_buffer, prefixdir);
        }
//        strlcpy(temp_buffer + baselen, uri, pathlen + 1);
        strncat(temp_buffer, uri, pathlen);

        if (strchr(temp_buffer, '.') == NULL) {
            strcat(temp_buffer, ".html");
        }
 
ESP_LOGI(TAG, "%s: '%s' -> '%s'", __func__, uri, temp_buffer);

        int fd = open(temp_buffer, O_RDONLY);
        if (fd >= 0) {
            pathname = temp_buffer;
            close(fd);
        } else {
            ESP_LOGI(TAG, "%s: cannot access %s", __func__, temp_buffer);
        }
    }

    return pathname;
}

const char *get_pathname_from_file(const char *filename, char *temp_buffer, size_t temp_buffer_len)
{
    const char* pathname = NULL;

    if (strlen(CONFIG_LASTLINK_HTML_DIRECTORY) + strlen(filename) + 2 < temp_buffer_len) {
        strcpy(temp_buffer, CONFIG_LASTLINK_HTML_DIRECTORY);
        strcat(temp_buffer, "/");
        strcat(temp_buffer, filename);

//ESP_LOGI(TAG, "%s: '%s' -> '%s'", __func__, filename, temp_buffer);

        int fd = open(temp_buffer, O_RDONLY);
        if (fd >= 0) {
            pathname = temp_buffer;
            close(fd);
        } else {
            ESP_LOGI(TAG, "%s: cannot access %s", __func__, temp_buffer);
        }
    }

    return pathname; 
}

void release_text_buffer(text_buffer_t *text_buffer)
{
    free((void*) text_buffer->base);
    text_buffer->base = NULL;
    text_buffer->current = NULL;
    text_buffer->used = 0;
    text_buffer->len = 0;

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

/*
 * Read a file relative to the HTML base directory
 */
bool read_file(text_buffer_t *text_buffer, const char *pathname)
{
    struct stat  sb;
    bool ok = false;

    if (stat(pathname, &sb) == 0) {
        text_buffer->len = sb.st_size;
        text_buffer->used = sb.st_size;
        /* Always one byte longer for the terminating NUL */
        text_buffer->base = (char *) malloc(sb.st_size + 1);
        text_buffer->current = text_buffer->base;

        if (text_buffer->base!= NULL) {
            FILE *fp = fopen(pathname, "r");
            if (fp != NULL) {
                size_t read_len = fread(text_buffer->base, 1, sb.st_size, fp);
                if (read_len != sb.st_size) {
ESP_LOGI(TAG, "%s: read %d bytes wanted %ld", __func__, read_len, sb.st_size);
                } else {
                    /* NUL terminate the buffer */
                    text_buffer->base[sb.st_size] = '\0';
                    ok = true;
                }

                fclose(fp);
            }

            if (! ok) {
                release_text_buffer(text_buffer);
            }
        }
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return ok;
}

/*
 * Remove text from 'first' through 'last' characters.
 */
void remove_text(text_buffer_t *text_buffer, char *first, char *last)
{
    assert(first <= last);

//ESP_LOGI(TAG, "%s: removing %d chars from %d to %d", __func__, last - first + 1, first - text_buffer->base, last - text_buffer->base);

//char text[last - first + 1 + 1];
//strncpy(text, first, last - first + 1 + 1);
//text[last-first+1] = '\0';
//ESP_LOGI(TAG, "%s: %d characters at %d to %d \"%s\"", __func__, last - first + 1, first - text_buffer->base, last - text_buffer->base, text);

    /* Remove the selected text */
    memcpy(first, last+1, text_buffer->used - (last - text_buffer->base) + 1);

    /* Remove from buffer allocation */
    text_buffer->used -= (last - first + 1);
#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

/*
 * Replace <item_size> bytes at <text_buffer.current> with <replaced>
 *
 * Returns true if ok else false
 */
bool replace_text(text_buffer_t *text_buffer, size_t item_size, const char *replaced)
{
    bool ret = true;

    size_t replace_len = strlen(replaced);
    
//ESP_LOGI(TAG, "%s: replacing %d with %d characters at position %d", __func__, item_size, replace_len, text_buffer->current - text_buffer->base);

    /* We found a var to replace; replace the text here with var value */
    int needed = replace_len - item_size;
    
    /* If needed > 0 then we may need to expand the area */
    if (needed > 0) {
        /* Difference between len and used is the room currently available */
//ESP_LOGI(TAG, "%s: need %d have %d", __func__, needed, text_buffer->len - text_buffer->used);
        if (needed > (text_buffer->len - text_buffer->used)) {
            /* Not enough space, so make some more room */
            size_t resize = needed;
            if (resize < HTTPD_ALLOC_CHUNK_SIZE) {
                resize = HTTPD_ALLOC_CHUNK_SIZE;
            }
            char *new_text = realloc(text_buffer->base, text_buffer->len + resize);
            if (new_text != NULL) {
                size_t current_offset = text_buffer->current - text_buffer->base;
                text_buffer->base = new_text;
                /* Reset current pointer after reallocating */
                text_buffer->current = new_text + current_offset;
                text_buffer->len += resize;
//ESP_LOGI(TAG, "%s: resized text_buffer by %d to %d", __func__, resize, text_buffer->len);
            } else {
ESP_LOGI(TAG, "%s: resize failed", __func__);
            }
        }
    
        /* Need to make room - make sure we succeeded if allocation was requred */
        if (needed <= (text_buffer->len - text_buffer->used)) {
            /* Move text down to make a bigger hole */
            memmove(text_buffer->current + needed, text_buffer->current, text_buffer->used - (text_buffer->current - text_buffer->base) + 1);
            text_buffer->used += needed;
//ESP_LOGI(TAG, "%s: added %d bytes to text", __func__, needed);
        } else {
            /* No room left so give up */
            ret = false;
            ESP_LOGI(TAG, "%s: no room giving up", __func__);
        }
    } else if (needed < 0) {
        /* Need to remove space */
        needed = -needed;
//ESP_LOGI(TAG, "%s: removing %d bytes\n", __func__, needed);
        remove_text(text_buffer, text_buffer->current, text_buffer->current + needed - 1);
//ESP_LOGI(TAG, "%s: removed %d bytes from text", __func__, needed);
    } else {
        /* Just right; nothing to move */
    }
    
    if (ret) {
        memcpy(text_buffer->current, replaced, replace_len);
    }
 
    return ret;
}

static include_stack_item_t *add_include_stack_item(session_context_t *session, const char* filename)
{
    assert(session != NULL);

    include_stack_item_t *item = (include_stack_item_t *) malloc(sizeof(include_stack_item_t) + strlen(filename));

    if (item != NULL) {
        strcpy(item->filename, filename);
        ADD_TO_LIST(session->include_stack, item);
//ESP_LOGI(TAG, "%s: added %s at %d", __func__, filename, NUM_IN_LIST(session->include_stack));
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
    return item;
}

static void remove_include_stack_item(session_context_t *session, include_stack_item_t *item)
{
    assert(session != NULL);

//ESP_LOGI(TAG, "%s: removing %s at %d", __func__, item->filename, NUM_IN_LIST(session->include_stack));

    REMOVE_FROM_LIST(session->include_stack, item);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

static bool is_include_in_use(session_context_t *session, const char *filename)
{
    assert(session != NULL);

    bool found = false;

    for (include_stack_item_t *item = (include_stack_item_t *) FIRST_LIST_ITEM(session->include_stack); !found && item != NULL; item = NEXT_LIST_ITEM(item, session->include_stack)) {
        found = strcmp(item->filename, filename) == 0;
    }

    return found; 
}

static void post_include_error(session_context_t *session, text_buffer_t *text_buffer, size_t item_size, const char* func, const char *msg, const char *filename)
{
    assert(session != NULL);

    const char* current_file;
    if (NUM_IN_LIST(session->include_stack) == 0) {
        current_file = "NONE";
    } else { 
        current_file = ((include_stack_item_t*) LAST_LIST_ITEM(session->include_stack))->filename;
    }

    //ESP_LOGE(TAG, "%s: In %s (text_buffer.base %p text_buffer.len %d item_size %d, %s '%s'", func, current_file, text_buffer->base, text_buffer->len, item_size, msg, filename);

    char *text;
    asprintf(&text, "['%s': %s '%s']", current_file, msg, filename);

    /* If buffer is currently empty (as in we haven't read anything to it yet) just put the message in the entire buffer */
    if (text_buffer->base == NULL) {
        text_buffer->base = text;
        text_buffer->current = text;
        text_buffer->used = strlen(text);
        text_buffer->len = text_buffer->used;
    } else {
        replace_text(text_buffer, item_size, text);
        free((void*) text);
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}


static bool do_commands(text_buffer_t *text_buffer, session_context_t *session)
{
    bool ret = true;

    assert(session != NULL);

    /* Then go through text and substitute all "${var}" with "value" from varlist */
    text_buffer->current = text_buffer->base;

    do {
        /* Look for start of a var name */
        text_buffer->current = find_string(text_buffer->current, "${");
        if (text_buffer->current != NULL) {
//ESP_LOGI(TAG, "%s: found macro header at position %d", __func__, text_buffer->current - text_buffer->base);

            const char *p = text_buffer->current + 2;

            char temp_buffer[HTTPD_MAX_KEYWORD_LEN+1];
            const char *varname = fetch_symbol(&p, temp_buffer, sizeof(temp_buffer));

            /* Skip to end of macro or end of buffer */
            while ((*p != '}') && (*p != '\0')) {
                ++p;
            }

            const char *end_macro = p;

            /* If we see the closing brace of the macro and the first char is an alpha. */
            if (*end_macro == '}' && varname != NULL) {

                /* Compute size of var macro call */
                size_t item_len = end_macro - text_buffer->current + 1;

//ESP_LOGI(TAG, "%s: looking for var '%s'", __func__, varname);
                var_item_t *var = find_var(session->varlist, varname);
                if (var != NULL) {
//ESP_LOGI(TAG, "%s: found '%s' replacing with '%s'", __func__, var->name, var->value);
                    replace_text(text_buffer, item_len, var->value);
                } else {
//ESP_LOGI(TAG, "%s: did not find '%s'; removing %d characters", __func__, varname, item_len);
                    /* Replace with empty */
                    replace_text(text_buffer, item_len, "");
                }
                /* Leave current pointing to begining of replaced text so we can rescan for additional macros (within the macros) */
            } else {
                /* We didn't handle this one - skip over leading '{' so we don't create a forever loop */
//ESP_LOGI(TAG, "%s: skipping over current char '%c'", __func__, *text_buffer->current);
                text_buffer->current++;
            }
        }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
        assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
    } while (text_buffer->current != NULL);

    /* Go through text looking for {% xxx %} and parsing the xxx to call a command */
    text_buffer->current = text_buffer->base;
    do {
        text_buffer->current = find_string(text_buffer->current, "{%");
        if (text_buffer->current != NULL) {
            char *start = text_buffer->current;
            text_buffer->current = find_string(text_buffer->current, "%}");

            if (text_buffer->current != NULL) {
                /* Point to one past the end of command field */
                char *end = text_buffer->current + 2;

                /* Total size of item including surrounding "{% %}" */
                size_t text_item_len = end - start;

                /* Back up current to beginning of item */
                text_buffer->current = start;

                /* Make a copy of this region (without the surrounding {% %} */
                char raw_args[text_item_len - 4 + 1];
                strncpy(raw_args, start + 2, (end - start) - 4);
                raw_args[end - start - 4] = '\0';

                /* Fetch keyword of the string */
                const char *raw_argp = raw_args;
                char temp_arg0[HTTPD_MAX_KEYWORD_LEN + 1];
     
                const char *argv0 = fetch_symbol(&raw_argp, temp_arg0, sizeof(temp_arg0));

                command_item_t *command = find_command(argv0);

                bool ok = false;

                if (command != NULL) {
                    if (command->function_rawargs != NULL) {
                        ok = command->function_rawargs(text_buffer, text_item_len, session, argv0, raw_argp);
                    } else if (command->function_argv != NULL) {
                        const char *args[HTTPD_MAX_COMMAND_ARGS + 1];
                        int argc = tokenize(raw_args, args, HTTPD_MAX_COMMAND_ARGS);

                        /* Parse into argv/argc and pass to function after looking up argv[0] for command */
                        ok = command->function_argv(text_buffer, text_item_len, session, argc, args);
                    } else {
                        char *text;
                        asprintf(&text, "[ Invalid command: %s ]", argv0);
                        replace_text(text_buffer, text_item_len, text);
                        free((void*) text);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
                    }
                }

                if (!ok) {
                    /* Error - just remove the text */
                    replace_text(text_buffer, text_item_len, "");
                }
            }
        }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
        assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
    } while (text_buffer->current != NULL);


    return ret;
}

/*
 * Read in template file and apply commands an resolve macros.
 */
bool read_template(text_buffer_t *text_buffer, size_t item_size, const char *filename, session_context_t *session)
{
    assert(session != NULL);

    if (NUM_IN_LIST(&command_list) == 0) {
        init_command_list();
    }

    if (is_include_in_use(session, filename)) {
        post_include_error(session, text_buffer, item_size, __func__, "Recursive include", filename);

    } else {

        /* Read in include file */
        text_buffer_t  local_text_buffer = {0};

        const char *pathname;

        char temp_buffer[HTTPD_TEMP_URL_BUFFER_LEN];

        if (filename[0] != '/') {
            pathname = get_pathname_from_file(filename, temp_buffer, sizeof(temp_buffer));
        } else {
            pathname = filename;
        }

        /* Read file into local text buffer */
        if (pathname != NULL && read_file(&local_text_buffer, pathname)) {

            include_stack_item_t *item = add_include_stack_item(session, filename);

            /* Remove any trailing newlines */
            while (local_text_buffer.used != 0 && local_text_buffer.base[local_text_buffer.used-1] == '\n') {
                local_text_buffer.base[local_text_buffer.used - 1] = '\0';
                local_text_buffer.used--;
            }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
            assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

            do_commands(&local_text_buffer, session);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
            assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
            if (text_buffer->base == NULL) {
                /* Uninitialized, so just replace it with the contents of local_text_buffer */
                *text_buffer = local_text_buffer;

            } else {
                if (! replace_text(text_buffer, item_size, local_text_buffer.base)) {
                    ESP_LOGI(TAG, "%s: replace failed", __func__);
                }
            
               release_text_buffer(&local_text_buffer);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
               assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
            }

            remove_include_stack_item(session, item);
        } else {
            /* No file */
            post_include_error(session, text_buffer, item_size, __func__, "No file", filename);
        }
    }
 
    return true;
}

/*
 * Include a file into the current text_buffer.
 *
 * We probably need a way to guard against recursive includes...
 */
static bool command_include(text_buffer_t *text_buffer, size_t item_size, session_context_t *session, int argc, const char **argv)
{
    assert(session != NULL);

    return read_template(text_buffer, item_size, argv[1], session);
}

typedef struct if_boundaries {
    char     *else_begin;
    char     *else_end;
    bool     else_if;
    char     *endif_begin;
    char     *endif_end;
} if_boundaries_t;

/*
 * Find boundaries of if .. elseif .. else .. endif statements within the text buffer
 */
static bool find_if_bounds(text_buffer_t *text_buffer, if_boundaries_t *if_boundaries)
{
    char *current = text_buffer->current;
    int level = 0;
    bool ok = false;
    char **end_mark = NULL;

    /* Initialize to false */
    memset(if_boundaries, 0, sizeof(if_boundaries_t));

    while (current != NULL) {
        current = find_string(current, "{%");
        if (current != NULL) {
            char *start = current;

            current += 2;

            char temp_buffer[8];
            char *if_command = fetch_symbol((const char **) &current, temp_buffer, sizeof(temp_buffer));

            if (strcmp(if_command, "elseif") == 0) {
                if (level == 0) {
                    /* Beginning of elseif */
                    if_boundaries->else_if = true;
                    if_boundaries->else_begin = start;
                    end_mark = &if_boundaries->else_end;
                }

            } else if (strcmp(if_command, "else") == 0 || strcmp(if_command, "elseif") == 0) {
                if (level == 0) {
                    if_boundaries->else_begin = start;
                    end_mark = &if_boundaries->else_end;
                }

            } else if (strcmp(if_command, "endif") == 0) {
                if (level == 0) {
                    if_boundaries->endif_begin = start;
                    end_mark = &if_boundaries->endif_end;
                } else {
                    level--;
                }

            } else if (strcmp(if_command, "if") == 0) {
                ++level;
            }

            current = find_string(current, "%}");
            if (current != NULL) {
                current += 2;
            }

            if (level == 0 && current != NULL && end_mark != NULL) {
                /* Mark position of closing brace */
                *end_mark = current - 1;
                end_mark = NULL;

                if (if_boundaries->endif_end != NULL) {
                    /* end if here */
                    current = NULL;
                    ok = true;
                }
            }
        }
    }

//ESP_LOGI(TAG, "%s: ok %s else_begin %d  else_end %d  else_if %s endif_begin %d endif_end %d", __func__,
//         ok ? "Yes" : "No",
//         if_boundaries->else_begin  ? if_boundaries->else_begin - text_buffer->base : -1,
//         if_boundaries->else_end    ? if_boundaries->else_end - text_buffer->base : -1,
//         if_boundaries->else_if     ? "Yes" : "No",
//         if_boundaries->endif_begin ? if_boundaries->endif_begin - text_buffer->base : -1,
//         if_boundaries->endif_end  ? if_boundaries->endif_end - text_buffer->base : -1);

    return ok;
}

/*
 * Set a var value
 *
 * {% setvar varname <value> %}
 *
 * Vars are only strings, so if evaluation to integer, convert to string.
 * NONE values are converted to empty strings.
 * Strings are saved as-is.
 */
static bool command_setvar(text_buffer_t *text_buffer, size_t item_size, session_context_t *session, const char *arg0, const char *args)
{
//ESP_LOGI(TAG, "%s: argv0 '%s' args '%s'", __func__, arg0, args);
    char temp_buffer[HTTPD_MAX_KEYWORD_LEN+1];
    const char *var_name = fetch_symbol(&args, temp_buffer, sizeof(temp_buffer));

    /* Evaluate expression */
    eval_value_t value;
    const char *argp = args;
    eval_error_t error = eval_expression(&value, eval_get_session_var, session, &argp);
    replace_text(text_buffer, item_size, "");

    if (error == EVERR_NONE) {
        /* Remove the command from the text buffer */
        switch (value.type) {
            default:
            case VT_NONE: {
                set_var(session->varlist, var_name, "");
                break;
            }
            case VT_INT: {
                char temp_buf[20];
                snprintf(temp_buf, sizeof(temp_buf), "%d", value.integer);
                set_var(session->varlist, var_name, temp_buf);
                break;
            }
            case VT_STR: {
                set_var(session->varlist, var_name, value.string);
                break;
            }
        }
    } else {
        char temp_buf[80];
        snprintf(temp_buf, sizeof(temp_buf), "[ \"setvar %s %s\" eval error %d ]", var_name, args, error);
        replace_text(text_buffer, 0, temp_buf);
    }

    return true;
}

/*
 * Conditionally process text in the template
 *
 *  {% if <expr> %}
 *    ....
 *  {% endif %}
 */
static bool command_if(text_buffer_t *text_buffer, size_t item_size, session_context_t *session, const char *arg0, const char *args)
{
//ESP_LOGI(TAG, "%s: argv0 '%s' args '%s'", __func__, arg0, args);

    /* Remove the 'if' clause */
    replace_text(text_buffer, item_size, "");

    char *current = text_buffer->current;

    if_boundaries_t if_boundaries;

    /* Returns true if it finds bounding endif.  Returns position if end of endif and end of else */
    if (find_if_bounds(text_buffer, &if_boundaries)) {
        eval_value_t value;
        eval_error_t error = eval_expression(&value, eval_get_session_var, session, &args);

        if (error != EVERR_NONE) {
            ESP_LOGI(TAG, "%s: eval_expression error %d", __func__, error);
        } else {
            char temp_buffer[40];
            const char *expr_value = eval_get_expr_value(&value, temp_buffer, sizeof(temp_buffer));

            ESP_LOGI(TAG, "%s: eval_expression returned %s", __func__,  expr_value);
        }

        if (error != EVERR_NONE) {
            /* do something */
        } else if (value.type == VT_INT && value.integer == 0) {
            /* Clause is false, so remove from beginning to end of else */
            if (if_boundaries.else_begin != NULL) {
                /* Remove the "endif" */
//ESP_LOGI(TAG, "%s: removing the endif", __func__);
                remove_text(text_buffer, if_boundaries.endif_begin, if_boundaries.endif_end);
                /* Remove the 'if' up to 'else' */
//ESP_LOGI(TAG, "%s: remove the 'if' up to 'else'", __func__);
                remove_text(text_buffer, text_buffer->current, if_boundaries.else_begin-1);
                if (if_boundaries.else_if) {
                    /* An elseif - change to 'if' */
                    char *elseif = find_string(if_boundaries.else_begin, "elseif");
                    if (elseif != NULL) {
                        text_buffer->current = elseif;            
                        /* Turn elseif into if */
                        replace_text(text_buffer, 6, "if");
                        text_buffer->current = current;
                    } else {
                        /* Probably an error */
                    }
                }
            } else {
                /* plain "endif" - remove through "endif" */
//ESP_LOGI(TAG, "%s: removing plain endif", __func__);
                remove_text(text_buffer, text_buffer->current, if_boundaries.endif_end);
            }
        } else {
            /* Clause is true.  If an else, remove from else through endif else just remove the endif */
            if (if_boundaries.else_begin != NULL) {
                /* Clause is true - remove "else" through "endif" */
//ESP_LOGI(TAG, "%s: remove 'else' through 'endif'", __func__);
                remove_text(text_buffer, if_boundaries.else_begin, if_boundaries.endif_end);
            } else {
//ESP_LOGI(TAG, "%s: remove 'endif'", __func__);
                remove_text(text_buffer, if_boundaries.endif_begin, if_boundaries.endif_end);
            }
        }
    } else {
        /* Replace with error message - mismatched if/endif */
        replace_text(text_buffer, item_size, "[ Mismatched if/endif ]");
    }

    return true;
}

/*
 * Naked else statements are not allowed.
 */
static bool command_else(text_buffer_t *text_buffer, size_t item_size, session_context_t *session, const char *arg0, const char *args)
{
    replace_text(text_buffer, 0, "[naked else] ");

    return true;
}

/*
 * Naked endif statements are not allowed.
 */
static bool command_elseif(text_buffer_t *text_buffer, size_t item_size, session_context_t *session, const char *arg0, const char *args)
{
    replace_text(text_buffer, 0, "[naked elseif] ");

    return true;
}

static bool command_endif(text_buffer_t *text_buffer, size_t item_size, session_context_t *session, const char *arg0, const char *args)
{
    replace_text(text_buffer, 0, "[naked endif] ");

    return true;
}


static void init_command_list(void)
{
    add_template_command_argv("include",    command_include);
    add_template_command_rawargs("if",      command_if);
    add_template_command_rawargs("else",    command_else);
    add_template_command_rawargs("else",    command_elseif);
    add_template_command_rawargs("else",    command_endif);
    add_template_command_rawargs("setvar",  command_setvar);
}

void release_httpd_templates(void)
{
    while (NUM_IN_LIST(&command_list) != 0) {
        command_item_t *command = (command_item_t *) FIRST_LIST_ITEM(&command_list);
        REMOVE_FROM_LIST(&command_list, command);
        free((void*) command);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
    }
}

