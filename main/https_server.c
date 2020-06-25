/*
 * LastLink web server.
 */
#include "sdkconfig.h"
#ifdef CONFIG_ESP_HTTPS_SERVER_ENABLE

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <unistd.h>

#include "esp_netif.h"
#include "esp_eth.h"
#include "network_connect.h"
#include "os_specific.h"
#include "listops.h"
#include "tokenize.h"
#include "varlist.h"

#include <esp_https_server.h>

static const char *TAG = "LastLinkHTTPS";

/* Include stack so to avoid recursive includes */
typedef struct include_stack_item include_stack_item_t;
typedef struct include_stack_item {
    include_stack_item_t    *next;
    include_stack_item_t    *prev;
    char                    filename[1];
} include_stack_item_t;

typedef list_head_t  include_stack_t;

static include_stack_t include_stack;

/* Defines an editable text buffer */
typedef struct text_buffer {
    char   *base;            /* Base of buffer containing the text */
    char   *current;         /* Current pointer while roving through text */
                             /* Stored here in case a realloc is necessary. */
    size_t len;              /* Always the number of bytes allocated at base */
    size_t used;             /* Number of bytes currently in use at base */
} text_buffer_t;

/* A command item  */
typedef struct command_item {
    const char *   command;
    bool (*function)(text_buffer_t *text_buffer, size_t item_size, var_list_t *varlist, int argc, const char **argv);
} command_item_t;

static bool command_include(text_buffer_t *text_buffer, size_t item_size, var_list_t *varlist, int argc, const char **argv);

static command_item_t command_table[] = {
    { .command = "include", .function = command_include },
};


#define TEMP_URL_BUFFER_LEN  80

/*
 * Create a full pathname from a uri, adding '.htm' if necessary
 */
static const char* get_pathname_from_uri(const char *uri, char *temp_buffer, size_t temp_buffer_len)
{
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

    pathlen += 5;   /* Room for additional ".htm" <nul> */

    const char *pathname = NULL;

    if ((pathlen + baselen) < temp_buffer_len) {
        strcpy(temp_buffer, CONFIG_LASTLINK_HTML_DIRECTORY);
        strlcpy(temp_buffer + baselen, uri, pathlen + 1);

        if (strchr(temp_buffer, '.') == NULL) {
            strcat(temp_buffer, ".htm");
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

ESP_LOGI(TAG, "%s: '%s' -> '%s'", __func__, filename, temp_buffer);

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

/*
 * Read a file relative to the HTML base directory
 */
static bool read_file(text_buffer_t *text_buffer, const char *pathname)
{
    struct stat  sb;
    bool ok = false;

    if (stat(pathname, &sb) == 0) {
        text_buffer->len = sb.st_size;
        text_buffer->used = sb.st_size;
        text_buffer->base = malloc(sb.st_size + 1);
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
                free(text_buffer->base);
                text_buffer->base = NULL;
                text_buffer->current = NULL;
                text_buffer->used = 0;
                text_buffer->len = 0;
            }
        }
    }

    return ok;
}

#define ALLOC_CHUNK_SIZE 512

/*
 * Replace <item_size> bytes at <text_buffer.current> with <replaced>
 *
 * Returns true if ok else false
 */
static bool replace_text(text_buffer_t *text_buffer, size_t item_size, const char *replaced)
{
    bool ret = true;

    size_t replace_len = strlen(replaced);
    
ESP_LOGI(TAG, "%s: replacing %d with %d characters at position %d", __func__, item_size, replace_len, text_buffer->current - text_buffer->base);

    /* We found a var to replace; replace the text here with var value */
    int needed = replace_len - item_size;
    
    /* If needed > 0 then we may need to expand the area */
    if (needed > 0) {
        /* Difference between len and used is the room currently available */
        if (needed > (text_buffer->len - text_buffer->used)) {
            /* Not enough space, so make some more room */
            size_t resize = replace_len;
            if (resize < ALLOC_CHUNK_SIZE) {
                resize = ALLOC_CHUNK_SIZE;
            }
            char *new_text = realloc(text_buffer->base, text_buffer->len + resize);
            if (new_text != NULL) {
                size_t current_offset = text_buffer->current - text_buffer->base;
                text_buffer->base = new_text;
                /* Reset current pointer after reallocating */
                text_buffer->current = new_text + current_offset;
                text_buffer->len += resize;
ESP_LOGI(TAG, "%s: resized text_buffer by %d to %d", __func__, resize, text_buffer->len);
            }
        }
    
        /* Need to make room - make sure we succeeded if allocation was requred */
        if (needed < (text_buffer->len - text_buffer->used)) {
            /* Move text down to make a bigger hole */
            memmove(text_buffer->current + needed, text_buffer->current, text_buffer->used - (text_buffer->current - text_buffer->base) + 1);
            text_buffer->used += needed;
ESP_LOGI(TAG, "%s: added %d bytes to text", __func__, needed);
        } else {
            /* No room left so give up */
            ret = false;
        }
    } else if (needed < 0) {
        /* Need to remove space */
        needed = -needed;
        memcpy(text_buffer->current, text_buffer->current + needed, text_buffer->used - (text_buffer->current - text_buffer->base) + 1);
        /* Remove the returned space from used */
        text_buffer->used -= needed;
ESP_LOGI(TAG, "%s: removed %d bytes from text", __func__, needed);
    } else {
        /* Just right; nothing to move */
    }
    
    if (ret) {
        memcpy(text_buffer->current, replaced, replace_len);
    }
 
    return ret;
}

static include_stack_item_t *add_include_stack_item(const char* filename)
{
    include_stack_item_t *item = (include_stack_item_t *) malloc(sizeof(include_stack_item_t) + strlen(filename));
    if (item != NULL) {
        strcpy(item->filename, filename);
        ADD_TO_LIST(&include_stack, item);
ESP_LOGI(TAG, "%s: added %s at %d", __func__, filename, NUM_IN_LIST(&include_stack));
    }
    return item;
}

static void remove_include_stack_item(include_stack_item_t *item)
{
ESP_LOGI(TAG, "%s: removing %s at %d", __func__, item->filename, NUM_IN_LIST(&include_stack));
    REMOVE_FROM_LIST(&include_stack, item);
}

static bool is_include_in_use(const char *filename)
{
    bool found = false;

    for (include_stack_item_t *item = (include_stack_item_t *) FIRST_LIST_ITEM(&include_stack); !found && item != NULL; item = NEXT_LIST_ITEM(item, &include_stack)) {
        found = strcmp(item->filename, filename) == 0;
    }

    return found; 
}

static void post_include_error(text_buffer_t *text_buffer, size_t item_size, const char* func, const char *msg, const char *filename)
{
    const char* current_file;
    if (NUM_IN_LIST(&include_stack) == 0) {
        current_file = "NONE";
    } else { 
        current_file = ((include_stack_item_t*) LAST_LIST_ITEM(&include_stack))->filename;
    }

    ESP_LOGE(TAG, "%s: In %s (text_buffer.base %p text_buffer.len %d item_size %d, %s '%s'", func, current_file, text_buffer->base, text_buffer->len, item_size, msg, filename);

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
}


#define MAX_ARGS   5
static bool do_commands(text_buffer_t *text_buffer, var_list_t *varlist)
{
    bool ret = true;

    /* Go through text looking for {% xxx %} and parsing the xxx to call a command */
    text_buffer->current = text_buffer->base;
    do {
        text_buffer->current = strstr(text_buffer->current, "{%");
        if (text_buffer->current != NULL) {
            char *start = text_buffer->current;
            text_buffer->current = strstr(text_buffer->current, "%}");

            if (text_buffer->current != NULL) {
                char *end = text_buffer->current + 2;

                /* Back up current to beginning of item */
                text_buffer->current = start;

                /* Total size of item including surrounding "{% %}" */
                size_t text_item_len = end - start;
                text_buffer->current[text_item_len - 2] = '\0';

                const char *args[MAX_ARGS + 1];
                int argc = tokenize(start + 2, args, MAX_ARGS);

                command_item_t *found = NULL;
                for (command_item_t *command = &command_table[0]; !found && command < &command_table[ELEMENTS_OF(command_table)]; ++command) {
                    if (strcmp(command->command, args[0]) == 0) {
                        found = command;
                    }
                }

                bool ok = false;

                if (found) {
                    /* Parse into argv/argc and pass to function after looking up argv[0] for command */
                    ok = found->function(text_buffer, text_item_len, varlist, argc, args);
                }

                if (!ok) {
                    /* Error - just remove the text */
                    replace_text(text_buffer, text_item_len, "");
                }
            }
        }
    } while (text_buffer->current != NULL);

    /* Then go through text and substitute all "{var}" with "value" from varlist */
    text_buffer->current = text_buffer->base;

    do {
        /* Look for start of a var name */
        text_buffer->current = strstr(text_buffer->current, "${");
        if (text_buffer->current != NULL) {
            /* Fetch up to the closing '}' */
            char *var_start = text_buffer->current + 2;
            while (isspace(*var_start)) {
                ++var_start;
            }

            char *var_end = var_start;

            /* Look for the end of the var name */
            while (isalnum(*var_end)) {
                ++var_end;
            }

            while ((*var_end != '}') && (*var_end != '\0')) {
                ++var_end;
            }

            if (*var_end == '}') {
                /* Process the var */
                *var_end++ = '\0';
                size_t item_len = var_end - text_buffer->current;

ESP_LOGI(TAG, "%s: looking for var '%s'", __func__, var_start);
                var_item_t *var = find_var(varlist, var_start);
                if (var != NULL) {
ESP_LOGI(TAG, "%s: found '%s' replacing with '%s'", __func__, var->name, var->value);
                    replace_text(text_buffer, item_len, var->value);
                } else {
ESP_LOGI(TAG, "%s: did not find '%s'; replacing with ''", __func__, var_start);
                    /* Replace with empty */
                    replace_text(text_buffer, item_len, "");
                }
                /* Leave current pointing to begininng of replaced text so we can rescan for additional macros */
            } else {
                /* We didn't handle this one - skip over leading '{' so we don't create a forever loop */
ESP_LOGI(TAG, "%s: skipping over current char '%c'", __func__, *text_buffer->current);
                text_buffer->current++;
            }
        }

    } while (text_buffer->current != NULL);

    return ret;
}

/*
 * Include file <filename> at text_buffer.current, replacing <item_size> bytes with result.
 */
static bool do_include(text_buffer_t *text_buffer, size_t item_size, const char *filename, var_list_t *varlist)
{
    if (is_include_in_use(filename)) {
        post_include_error(text_buffer, item_size, __func__, "Recursive include", filename);

    } else {

        /* Read in include file */
        text_buffer_t  local_text_buffer;

        const char *pathname;

        char temp_buffer[TEMP_URL_BUFFER_LEN];

        if (filename[0] != '/') {
            pathname = get_pathname_from_file(filename, temp_buffer, sizeof(temp_buffer));
        } else {
            pathname = filename;
        }

        /* Read file into local text buffer */
        if (pathname != NULL && read_file(&local_text_buffer, pathname)) {

            include_stack_item_t *item = add_include_stack_item(filename);

            /* Remove any trailing newlines */
            while (local_text_buffer.used != 0 && local_text_buffer.base[local_text_buffer.used-1] == '\n') {
                local_text_buffer.base[local_text_buffer.used - 1] = '\0';
                local_text_buffer.used--;
            }

            do_commands(&local_text_buffer, varlist);

            if (text_buffer->base == NULL) {
                /* Uninitialized, so just replace it with the contents of local_text_buffer */
                *text_buffer = local_text_buffer;

            } else {
                if (! replace_text(text_buffer, item_size, local_text_buffer.base)) {
                    ESP_LOGI(TAG, "%s: replace failed", __func__);
                }
            
                free(local_text_buffer.base);
            }

            remove_include_stack_item(item);
        } else {
            /* No file */
            post_include_error(text_buffer, item_size, __func__, "No file", filename);
        }
    }
 
    return true;
}

/*
 * Include a file into the current text_buffer.
 *
 * We probably need a way to guard against recursive includes...
 */
static bool command_include(text_buffer_t *text_buffer, size_t item_size, var_list_t *varlist, int argc, const char **argv)
{
    ESP_LOGI(TAG, "%s: item_size %d argc %d", __func__, item_size, argc);

    for (int arg = 0; arg < argc; ++arg) {
        ESP_LOGI(TAG, "%s: arg %d = '%s'", __func__, arg, argv[arg]);
    }

    return do_include(text_buffer, item_size, argv[1], varlist);
}

/*
 * Process a file for commands a vars and send it to client.
 *
 * If pathname is absolute (prepended /) it does not get altered; otherwise prepends html folder name.
 */
static esp_err_t httpd_resp_send_file(httpd_req_t *req, const char *pathname)
{
    esp_err_t ret;

    if (pathname == NULL) {
        ret = httpd_resp_send_404(req);

    } else {
        text_buffer_t  text_buffer = {0};

        do_include(&text_buffer, 0, pathname, (var_list_t *) req->sess_ctx);

ESP_LOGI(TAG, "%s: after parsing, base %p, current %p, len %d, used %d", __func__, text_buffer.base, text_buffer.current, text_buffer.len, text_buffer.used);

        if (text_buffer.base == NULL) {
            text_buffer.len = text_buffer.used = asprintf(&text_buffer.base, "<html><body><h1>%s not found</h1><body><html>", pathname);
        }

        if (text_buffer.used > 0) {
            ret = httpd_resp_send(req, text_buffer.base, text_buffer.used);
        } else {
            ret = httpd_resp_send(req, "<h1>Cannot edit file</h1>", -1);
        }

        free((void*) text_buffer.base);
    }

    return ret;
}

/*
 * authenticate username/password.
 *
 * Performs authentication and return token (to be delivered to user as cookie.
 *
 * Returns true if successful authentication.
 *
 * This is currently a stub.
 */
static bool authenticate(const char *username, const char *password, char *token, size_t token_len)
{
    ESP_LOGI(TAG, "%s: username '%s' password '%s'", __func__, username, password);
    snprintf(token, token_len, "%s-%s", username, password);
    return true;
}

static void redirect(httpd_req_t *req, const char* uri)
{
    ESP_LOGI(TAG, "%s: to '%s'", __func__, uri);

    /* Redirect to the page */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", uri);
    httpd_resp_send(req, NULL, 0);
}

static bool get_lastlink_auth_token(httpd_req_t *req, char *auth_token, size_t auth_token_len)
{
    size_t cookie_len = httpd_req_get_hdr_value_len(req, "Cookie");
    ESP_LOGI(TAG, "%s: cookie_len %d", __func__, cookie_len);

    if (cookie_len != 0) {
        char *cookie = (char *) malloc(cookie_len);

        esp_err_t ret =  httpd_req_get_hdr_value_str(req, "Cookie", cookie, cookie_len);
        if (ret == ESP_OK) {
           ESP_LOGI(TAG, "%s: cookie is '%s'", __func__, cookie);

           char *token = strstr(cookie, "lastlink_auth_token=");

            if (token != NULL) {
                token = strchr(token, '=') + 1;
                char *p = strchr(token, '&');
                if (p != NULL) {
                   *p = '\0';
                }
                strncpy(auth_token, token, auth_token_len);
                auth_token[auth_token_len - 1] = '\0';
                return true;
            }
        } else {
            ESP_LOGI(TAG, "%s: ret is %d", __func__, ret);
        }

        free((void*) cookie);
    }
    return false;
}

static bool check_if_logged_in(httpd_req_t *req)
{
    char auth_token[50];

    /* Simple test for now - just check for existance */
    if (get_lastlink_auth_token(req, auth_token, sizeof(auth_token))) {
        ESP_LOGI(TAG, "%s: lastlink_auth_token \"%s\"", __func__, auth_token);
        return true;
    }

    return false;
}

/* An HTTP GET handler that requires validation of user */
static esp_err_t validate_user_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

ESP_LOGI(TAG, "%s: uri '%s'", __func__, req->uri);

    if (check_if_logged_in(req)) {
        /* Present the page from a file */
        char temp_buffer[TEMP_URL_BUFFER_LEN];

        const char *uri = req->uri;
        if (strcmp(uri, "/") == 0) {
            uri = "/index";
        }

        const char *pathname = get_pathname_from_uri(uri, temp_buffer, sizeof(temp_buffer));

        httpd_resp_send_file(req, pathname);

    } else {
        redirect(req, "/login");
    }

    return ESP_OK;
}

static esp_err_t unprotected_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

ESP_LOGI(TAG, "%s: uri '%s'", __func__, req->uri);

    char temp_buffer[TEMP_URL_BUFFER_LEN];

    const char *pathname = get_pathname_from_uri(req->uri, temp_buffer, sizeof(temp_buffer));

    httpd_resp_send_file(req, pathname);

    return ESP_OK;
}

static const httpd_uri_t validate_user_uri = {
    .uri       = "/*",
    .method    = HTTP_GET,
    .handler   = validate_user_get_handler
};

static const httpd_uri_t js_uri = {
    .uri       = "/*.js",
    .method    = HTTP_GET,
    .handler   = unprotected_get_handler
};

static const httpd_uri_t css_uri = {
    .uri       = "/*.css",
    .method    = HTTP_GET,
    .handler   = unprotected_get_handler
};

static esp_err_t login_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    char temp_buffer[TEMP_URL_BUFFER_LEN];

    const char *pathname = get_pathname_from_uri(req->uri, temp_buffer, sizeof(temp_buffer));

    return httpd_resp_send_file(req, pathname);
}

static const httpd_uri_t login = {
    .uri       = "/login",
    .method    = HTTP_GET,
    .handler   = login_get_handler
};

static esp_err_t login_post_handler(httpd_req_t *req)
{
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */
    char content[100];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = MIN(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }

    content[recv_size] = '\0';

    ESP_LOGI(TAG, "%s: content '%s'", __func__, content);

    char *username = strstr(content, "uname=");
    char *password = strstr(content, "psw=");
    char *remember = strstr(content, "rem=");

    if (username != NULL && password != NULL && remember != NULL) {
        username += 6;
        password += 4;
        remember += 4;
        char *p = strchr(username, '&');
        if (p != NULL) {
            *p = '\0';
        }
        p = strchr(password, '&');
        if (p != NULL) {
            *p = '\0';
        }
        p = strchr(remember, '&');
        if (p != NULL) {
            *p = '\0';
        }

        ESP_LOGI(TAG, "%s: username '%s' password '%s' remember '%s'", __func__, username, password, remember);

        char token[50];

        if (authenticate(username, password, token, sizeof(token))) {
            char *cookie;
            asprintf(&cookie, "lastlink_auth_token=\"%s\"; Max-Age=3600; Secure", token);
            httpd_resp_set_hdr(req, "Set-Cookie", cookie);
            httpd_resp_send(req, NULL, 0);  // Response body can be empty
            free((void*) cookie);

            if (req->sess_ctx == NULL) {
                var_list_t *var_list = (var_list_t*) malloc(sizeof(var_list_t));
                INIT_LIST(var_list);
                req->sess_ctx = (void*) var_list;
                req->free_ctx = free_var_list;
            }

            set_var((var_list_t*) req->sess_ctx, "username", username);  
            redirect(req, "/index");
        } else {
            redirect(req, "/login");
        }
    } else {
        redirect(req, "/login");
    }

    return ESP_OK;
}

static const httpd_uri_t login_answer = {
    .uri       = "/login",
    .method    = HTTP_POST,
    .handler   = login_post_handler
};

static httpd_handle_t start_https_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting https server");

    httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();
    config.httpd.uri_match_fn = httpd_uri_match_wildcard;
    // config.httpd.max_resp_headers = 50;

    extern const unsigned char cacert_pem_start[] asm("_binary_cacert_pem_start");
    extern const unsigned char cacert_pem_end[]   asm("_binary_cacert_pem_end");
    config.cacert_pem = cacert_pem_start;
    config.cacert_len = cacert_pem_end - cacert_pem_start;

    extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
    extern const unsigned char prvtkey_pem_end[]   asm("_binary_prvtkey_pem_end");
    config.prvtkey_pem = prvtkey_pem_start;
    config.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

    esp_err_t ret = httpd_ssl_start(&server, &config);
    if (ESP_OK != ret) {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &login);                        /* The 'login' page */
    httpd_register_uri_handler(server, &login_answer);                 /* Handler for login page */
    httpd_register_uri_handler(server, &js_uri);                       /* All javascript refs */
    httpd_register_uri_handler(server, &css_uri);                      /* All css refs */
    httpd_register_uri_handler(server, &validate_user_uri);            /* All pages that need validation */
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, NULL);     /* Anything else if unavailable */

    return server;
}

static void stop_https_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_ssl_stop(server);
}

static void disconnect_https_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        stop_https_webserver(*server);
        *server = NULL;
    }
}

static void connect_https_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        *server = start_https_webserver();
    }
}

/* For http, only do a redirect to the https server */
static esp_err_t redirect_page(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_status(req, "301 Moved Permanently");
    httpd_resp_set_hdr(req, "Location", "https://lastlink.local/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t redirect_handler = {
    .uri       = "/*",
    .method    = HTTP_GET,
    .handler   = redirect_page
};

static httpd_handle_t start_http_webserver(void)
{
    httpd_handle_t server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting http server");

    esp_err_t ret = httpd_start(&server, &config);
    if (ESP_OK != ret) {
        ESP_LOGI(TAG, "Error starting http server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &redirect_handler);

    return server;
}

static void stop_http_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_http_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        stop_http_webserver(*server);
        *server = NULL;
    }
}

static void connect_http_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        *server = start_http_webserver();
    }
}

static httpd_handle_t https_server_handler = NULL;
static httpd_handle_t http_server_handler = NULL;

void https_server(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Register event handlers to start servers when Wi-Fi is connected
     * and stop server when disconnection happens.
     */

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_https_handler, &https_server_handler));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &disconnect_https_handler, &https_server_handler));

    /* Also start an http server (no ssl) to redirect to https */
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_http_handler, &http_server_handler));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &disconnect_http_handler, &http_server_handler));

    ESP_ERROR_CHECK(network_connect());
}

#endif /* CONFIG_ESP_HTTPS_SERVER_ENABLE */
