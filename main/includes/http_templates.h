/*
 * httpd_templates.h
 */
#ifndef __httpd_templates_h_included
#define __httpd_templates_h_included

#include "os_specific.h"
#include "listops.h"
#include "tokenize.h"
#include "varlist.h"

#define HTTPD_TEMP_URL_BUFFER_LEN  80
#define HTTPD_ALLOC_CHUNK_SIZE     512
#define HTTPD_MAX_COMMAND_ARGS     5
#define HTTPD_MAX_KEYWORD_LEN      10

typedef list_head_t  include_stack_t;

typedef struct sesssion_context {
    var_list_t         *varlist;
    include_stack_t    *include_stack;
    void               *private_context;
} session_context_t;

/* Defines an editable text buffer */
typedef struct text_buffer {
    char   *base;            /* Base of buffer containing the text */
    char   *current;         /* Current pointer while roving through text */
                             /* Stored here in case a realloc is necessary. */
    size_t len;              /* Always the number of bytes allocated at base */
    size_t used;             /* Number of bytes currently in use at base */
} text_buffer_t;

typedef bool (*function_argv_t)(text_buffer_t *text_buffer, size_t item_size, session_context_t *session, int argc, const char **argv);
typedef bool (*function_rawargs_t)(text_buffer_t *text_buffer, size_t item_size, session_context_t *session, const char* argv0, const char *args);

bool add_template_command_argv(const char *name, function_argv_t function);
bool add_template_command_rawargs(const char *name, function_rawargs_t function);

include_stack_t *create_include_stack(void);
void free_include_stack(include_stack_t *include_stack);

void free_session_context(void *param);
session_context_t *create_session_context(void);

bool set_session_var(session_context_t *session, const char *name, const char *value);
bool delete_session_var(session_context_t *session, const char *name);
bool get_session_var(session_context_t *session, const char *name, const char **value);

char *find_string(char *buffer, const char *string);
char *fetch_symbol(const char **ptr, char *buffer, size_t bufsize);
const char* get_pathname_from_uri(const char *uri, char *temp_buffer, size_t temp_buffer_len);
const char *get_pathname_from_file(const char *filename, char *temp_buffer, size_t temp_buffer_len);

typedef struct text_buffer text_buffer_t; // FWD decl
bool read_file(text_buffer_t *text_buffer, const char *pathname);
void remove_text(text_buffer_t *text_buffer, char *from, char *to);
bool replace_text(text_buffer_t *text_buffer, size_t item_size, const char *replaced);

bool read_template(text_buffer_t *text_buffer, size_t item_size, const char *filename, session_context_t *session);

#endif /* __httpd_templates_h_included */

