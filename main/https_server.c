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
#include "http_templates.h"

#include <esp_https_server.h>

#define TEMP_URL_BUFFER_LEN     200

static const char *TAG = "LastLinkHTTPS";

/*
 * Process a file for commands a vars and send it to client.
 *
 * If pathname is absolute (prepended /) it does not get altered; otherwise prepends html folder name.
 */
static esp_err_t httpd_resp_send_file(httpd_req_t *req, const char *pathname)
{
    esp_err_t ret;

    session_context_t *session = (session_context_t *) req->sess_ctx;
    assert(session != NULL);

    if (pathname == NULL) {
        ret = httpd_resp_send_404(req);

    } else {
        text_buffer_t  text_buffer = {0};

        read_template(&text_buffer, 0, pathname, session);

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
    return strcmp(password, "password") == 0;
}

static esp_err_t  redirect(httpd_req_t *req, const char* uri)
{
    ESP_LOGI(TAG, "%s: to '%s'", __func__, uri);

    /* Redirect to the page */
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_status(req, "303 Moved Permanently");
    httpd_resp_set_hdr(req, "Location", uri);

    return httpd_resp_send(req, NULL, 0);
}

static bool get_lastlink_auth_token(httpd_req_t *req, char *auth_token, size_t auth_token_len)
{
    size_t cookie_len = httpd_req_get_hdr_value_len(req, "Cookie");
    ESP_LOGI(TAG, "%s: cookie_len %d", __func__, cookie_len);

    if (cookie_len != 0) {
        char *cookie = (char *) malloc(++cookie_len);

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
ESP_LOGI(TAG, "****************************************************");
ESP_LOGI(TAG, "%s: uri %s", __func__, req->uri);
ESP_LOGI(TAG, "****************************************************");

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

    if (req->sess_ctx == NULL) {
        req->sess_ctx = create_session_context();
        req->free_ctx = free_session_context;
    }

    if (check_if_logged_in(req)) {

        char temp_buffer[TEMP_URL_BUFFER_LEN];
        const char *pathname = get_pathname_from_uri(req->uri, temp_buffer, sizeof(temp_buffer));
        httpd_resp_send_file(req, pathname);

    } else {
        /* Not logged in; force a trip to the login screen */
        redirect(req, "/login"); 
    }

    return ESP_OK;
}

static esp_err_t unprotected_get_handler(httpd_req_t *req)
{
    if (req->sess_ctx == NULL) {
        req->sess_ctx = create_session_context();
        req->free_ctx = free_session_context;
    }

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
    if (req->sess_ctx == NULL) {
        req->sess_ctx = create_session_context();
        req->free_ctx = free_session_context;
    }

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
    if (req->sess_ctx == NULL) {
        req->sess_ctx = create_session_context();
        req->free_ctx = free_session_context;
    }

    session_context_t *session = (session_context_t *) req->sess_ctx;
    assert(session != NULL);

    char user_token[50];

    user_token[0] = '\0';

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

        if (authenticate(username, password, user_token, sizeof(user_token))) {

            set_session_var(session, "username", username);  

            delete_session_var(session, "error");

            set_session_var(session, "zip", "zippy");

        } else {
            user_token[0] = '\0';

            set_session_var(session, "error", "Invalid login");
        }
    }

    const char *next_uri;
    char *cookie;

    if (user_token[0] != '\0') {
        asprintf(&cookie, "lastlink_auth_token=\"%s\"; Max-Age=3600; Secure", user_token);
        httpd_resp_set_hdr(req, "Set-Cookie", cookie);

        next_uri = "/"; 
    } else {
        cookie = NULL;
        next_uri = "/login";
    }

    char temp_buffer[TEMP_URL_BUFFER_LEN];
    const char *pathname = get_pathname_from_uri(next_uri, temp_buffer, sizeof(temp_buffer));

    esp_err_t err = httpd_resp_send_file(req, pathname);

    free((void*) cookie);
 
    return err;
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
    return redirect(req, "https://lastlink.local");
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
