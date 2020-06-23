/* Simple HTTP + SSL Server Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "sdkconfig.h"
#ifdef CONFIG_ESP_HTTPS_SERVER_ENABLE

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "network_connect.h"
#include "os_specific.h"
#include "listops.h"

#include <esp_https_server.h>

typedef struct var_item var_item_t;

typedef struct var_item {
    var_item_t     *next;
    var_item_t     *prev;
    const char     *name;
    const char     *value;
} var_item_t;

typedef list_head_t  var_list_t;

/*
 * A simple example that demonstrates how to create GET and POST
 * handlers and start an HTTPS server.
 */

static const char *TAG = "LastLinkHTTPS";

static var_item_t *create_var_item(const char* name, const char *value)
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

static void release_var_list(var_list_t *var_list)
{
    while (NUM_IN_LIST(var_list) != 0) {
       var_item_t *item = (var_item_t *) FIRST_LIST_ITEM(var_list);
       REMOVE_FROM_LIST(var_list, item);
       free((void*) item);
    }
}

/*
 * Open a file and send it, after editing it with the values in the var_map
 */
static esp_err_t httpd_resp_send_file(httpd_req_t *req, const char *filename, var_list_t *varlist)
{
    esp_err_t ret;
    char *text = NULL;
    size_t len = 0;
    struct stat  sb;

    if (stat(filename, &sb) == 0) {
        len = sb.st_size;
ESP_LOGI(TAG, "%s: '%s' is %d bytes long", __func__, filename, len);

        size_t max_len = len * 2; 
        text = (char *) malloc(max_len + 1);
        if (text != NULL) {
            FILE *fp = fopen(filename, "r");
            if (fp != NULL) {
                size_t read_len = fread(text, 1, len, fp);
                if (read_len == len) {
ESP_LOGI(TAG, "%s: read %d bytes", __func__, len);
                    /* Go through text and substitute all "{var}" with "value" from varlist */
                    if (varlist != NULL) {
                        for (var_item_t *var = (var_item_t *) FIRST_LIST_ITEM(varlist); var != NULL; var = NEXT_LIST_ITEM(var, varlist)) {

ESP_LOGI(TAG, "%s: looking for '%s' ('%s')", __func__, var->name, var->value);

                            size_t wanted_len = strlen(var->name) + 2;
                            size_t value_len = strlen(var->value);
                            char varwanted[wanted_len + 1];
                            sprintf(varwanted, "{%s}", var->name);
                            char *p = text;

                            do {
                                p = strstr(p, varwanted);
                                if (p != NULL) {
                                    /* Replace the text here */
                                    int needed = value_len - wanted_len;

                                    if (needed > 0) {
                                        /* Need to make room - make sure there is enough */
                                        if (len + needed < max_len) {
                                            /* Move to make room */
                                            memmove(p + needed, p, len - (p - text));
                                            len += needed;
ESP_LOGI(TAG, "%s: addec %d bytes to text", __func__, needed);
                                        } else {
                                            len = 0;
                                        }
                                    } else if (needed < 0) {
                                        /* Need to remove space */
                                        memcpy(p, p - needed, len - (p - text));
                                        /* Remove from the current length (needed is currently negative) */
                                        len += needed;
ESP_LOGI(TAG, "%s: removed %d bytes from text", __func__, -needed);
                                    } else {
                                        /* Just right */
                                    }

                                    memcpy(p, var->value, value_len);
                                }
                            } while (len != 0 && p != NULL); 
                        }
                    } 
                } else {
ESP_LOGI(TAG, "%s: read %d bytes wanted %d", __func__, read_len, len);
                    len = 0;
                } 

                fclose(fp);
            }
        }
    }
    
    if (text == NULL) {
        len = asprintf(&text, "<html><body><h1>%s not found</h1><body><html>", filename);
    }

    if (len > 0) {
        ret = httpd_resp_send(req, text, len);
    } else {
        ret = httpd_resp_send(req, "<h1>Cannot edit file</h1>", -1);
    }

    free((void*) text);

    return ret;
}


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
    httpd_resp_set_status(req, "301 Moved Permanently");
    httpd_resp_set_hdr(req, "Location", uri);
    httpd_resp_send(req, NULL, 0);
}

static bool get_lastlink_auth_token(httpd_req_t *req, char *auth_token, size_t auth_token_len)
{
    size_t cookie_len = httpd_req_get_hdr_value_len(req, "Cookie");
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
        ESP_LOGI(TAG, "%s: lastlink_auth_token %s", __func__, auth_token);
        return true;

    } else {
        /* Redirect to the login page */
        redirect(req, "/login");
    }

    return false;
}

/* An HTTP GET handler */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    if (check_if_logged_in(req)) {
        /* Create a var map to things */
        var_list_t var_list;
        INIT_LIST(&var_list);
        ADD_TO_LIST(&var_list, create_var_item("test", "this is a test"));
        ADD_TO_LIST(&var_list, create_var_item("zot", "this is a zot"));

        /* Present the home page */
        httpd_resp_send_file(req, CONFIG_LASTLINK_HTML_DIRECTORY "index.htm", &var_list);

        release_var_list(&var_list);
    }

    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler
};

static esp_err_t login_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    return httpd_resp_send_file(req, CONFIG_LASTLINK_HTML_DIRECTORY "login.htm", NULL);
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
            asprintf(&cookie, "lastlink_auth_token=%s; Max-Age=3600; Secure", token);
            //asprintf(&cookie, "lastlink_auth_token=%s; Max-Age=3600", token);
            httpd_resp_set_hdr(req, "Set-Cookie", cookie);
            httpd_resp_send(req, NULL, 0);  // Response body can be empty
            free((void*) cookie);
            redirect(req, "/");
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

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server");

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();

    extern const unsigned char cacert_pem_start[] asm("_binary_cacert_pem_start");
    extern const unsigned char cacert_pem_end[]   asm("_binary_cacert_pem_end");
    conf.cacert_pem = cacert_pem_start;
    conf.cacert_len = cacert_pem_end - cacert_pem_start;

    extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
    extern const unsigned char prvtkey_pem_end[]   asm("_binary_prvtkey_pem_end");
    conf.prvtkey_pem = prvtkey_pem_start;
    conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret) {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &login);
    httpd_register_uri_handler(server, &login_answer);
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, NULL);

    return server;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_ssl_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        *server = start_webserver();
    }
}

static httpd_handle_t server = NULL;

void https_server(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Register event handlers to start server when Wi-Fi is connected
     * and stop server when disconnection happens.
     */

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &disconnect_handler, &server));

    ESP_ERROR_CHECK(network_connect());
}

#endif /* CONFIG_ESP_HTTPS_SERVER_ENABLE */
