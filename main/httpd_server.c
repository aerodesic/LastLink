/*
 * LastLink web server.
 */
#include "sdkconfig.h"
#ifdef CONFIG_LASTLINK_WEB_SERVER_ENABLED

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <unistd.h>

#include <esp_http_server.h>

#ifdef CONFIG_LASTLINK_HTTPS_SERVER_ENABLED
#include <esp_https_server.h>
#endif

#include "esp_netif.h"
#include "esp_eth.h"
#include "network_connect.h"
#include "os_specific.h"
#include "http_templates.h"
#include "simpletimer.h"
#include "uuid.h"
#include "httpd_server.h"


#define TEMP_URL_BUFFER_LEN     200

static const char *TAG = "LastLinkWeb";

typedef uuid_text_trimmed_t    authtoken_value_t;

typedef struct authtoken       authtoken_t;
typedef struct authtoken {
    authtoken_t          *next;
    authtoken_t          *prev;
    authtoken_value_t    token;
    simpletimer_t        expiretime;
    char                 username[1];
} authtoken_t;

typedef list_head_t  authtoken_list_t;

authtoken_list_t     authtoken_list;
os_mutex_t           authtoken_lock;
    
static authtoken_t *check_authtoken_expired(authtoken_t *token)
{
    if (token != NULL && simpletimer_is_expired(&token->expiretime)) {
ESP_LOGI(TAG, "%s: token '%s' expired for user '%s' -- removing", __func__, token->token, token->username);
        REMOVE_FROM_LIST(&authtoken_list, token);
        free((void*) token);
        token = NULL;
    }

    return token;
}

static authtoken_t *find_authtoken_by_username(const char *username)
{
    authtoken_t *token;

    for (token = (authtoken_t*) FIRST_LIST_ITEM(&authtoken_list); token != NULL; token = NEXT_LIST_ITEM(token, &authtoken_list)) {
        if (strcmp(token->username, username) == 0) {
            break;
        }
    }

    return check_authtoken_expired(token);
}

static authtoken_t *find_authtoken_by_token(char *tokenname)
{
    authtoken_t *token;

//ESP_LOGI(TAG, "%s: looking for '%s'", __func__, tokenname);
    for (token = (authtoken_t*) FIRST_LIST_ITEM(&authtoken_list); token != NULL; token = NEXT_LIST_ITEM(token, &authtoken_list)) {
//ESP_LOGI(TAG, "%s: looking at '%s'", __func__, token->token);
        if (strcmp(token->token, tokenname) == 0) {
            break;
        }
    }

    token = check_authtoken_expired(token);

    ESP_LOGI(TAG, "%s: authtoken is %s", __func__, token ? token->token : "NULL");

    return token;
}

static void release_authtoken_list(void)
{
    os_acquire_recursive_mutex(authtoken_lock);

    while (NUM_IN_LIST(&authtoken_list) != 0) {
        authtoken_t *token = (authtoken_t*) FIRST_LIST_ITEM(&authtoken_list);
        REMOVE_FROM_LIST(&authtoken_list, token);
        free((void*) token);
    } 

    os_release_recursive_mutex(authtoken_lock);
}

static void allocate_session(httpd_req_t *req)
{
    if (req->sess_ctx == NULL) {
        req->sess_ctx = create_session_context();
        assert(req->sess_ctx);
        req->free_ctx = free_session_context;
    }
}

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

//ESP_LOGI(TAG, "%s: after parsing, base %p, current %p, len %d, used %d", __func__, text_buffer.base, text_buffer.current, text_buffer.len, text_buffer.used);

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

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return ret;
}

/*
 * Redirect to anoter location.
 */
static esp_err_t  redirect(httpd_req_t *req, const char* uri)
{
    //ESP_LOGI(TAG, "%s: to '%s'", __func__, uri);

    /* Redirect to the page */
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_status(req, "303 Moved Permanently");
    httpd_resp_set_hdr(req, "Location", uri);

    return httpd_resp_send(req, NULL, 0);
}

/*
 * authenticate username/password.
 *
 * Performs authentication and return token (to be delivered to user as cookie.)
 *
 * Returns true if successful authentication.
 *
 * This is currently a stub.
 */
static bool authenticate(const char *username, const char *password, authtoken_value_t token)
{
    os_delay(5000);

    if (strcmp(password, "password") == 0) {

        //ESP_LOGI(TAG, "%s: username '%s' password '%s'", __func__, username, password);

        /* Create local temporary token for this authentication */
        uuid_gen_trimmed(token);

        authtoken_t *authtoken = (authtoken_t*) malloc(sizeof(authtoken_t) + strlen(username));
        strcpy(authtoken->token, token);
        strcpy(authtoken->username, username);
        simpletimer_start(&authtoken->expiretime, CONFIG_LASTLINK_AUTHTOKEN_EXPIRE_TIME*1000);

        os_acquire_recursive_mutex(authtoken_lock);
        /* Find and remove any old authtoken for this user */
        authtoken_t *old_token = find_authtoken_by_username(username);
        if (old_token != NULL) {
            REMOVE_FROM_LIST(&authtoken_list, old_token);
        }
        /* Add new authtoken to list */
        ADD_TO_LIST(&authtoken_list, authtoken);
        os_release_recursive_mutex(authtoken_lock);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

        return true;
    }

    return false;
}

static authtoken_t *get_lastlink_authtoken(httpd_req_t *req)
{
    authtoken_t *authtoken = NULL;

    size_t cookie_len = httpd_req_get_hdr_value_len(req, "Cookie");
    //ESP_LOGI(TAG, "%s: cookie_len %d", __func__, cookie_len);

    if (cookie_len != 0) {
        char *cookie = (char *) malloc(++cookie_len);

        esp_err_t ret =  httpd_req_get_hdr_value_str(req, "Cookie", cookie, cookie_len);
        if (ret == ESP_OK) {
           //ESP_LOGI(TAG, "%s: cookie is '%s'", __func__, cookie);

#ifdef CONFIG_LASTLINK_HTTPS_SERVER_ENABLED
           char *token = strstr(cookie, "__Secure-lastlink_auth_token=");
#else
           char *token = strstr(cookie, "lastlink_auth_token=");
#endif

            if (token != NULL) {
                /* Get value between "" */
                token = strchr(token, '=') + 1;

                authtoken = find_authtoken_by_token(token);
            }
        } else {
            //ESP_LOGI(TAG, "%s: ret is %d", __func__, ret);
        }

        free((void*) cookie);
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return authtoken;
}

static bool check_if_logged_in(httpd_req_t *req)
{
//ESP_LOGI(TAG, "****************************************************");
//ESP_LOGI(TAG, "%s: uri %s", __func__, req->uri);
//ESP_LOGI(TAG, "****************************************************");

    /* See if an authtoken exists from this user */
    authtoken_t * authtoken = get_lastlink_authtoken(req);

    if (authtoken != NULL) {
        /* Refresh expire timer */
        simpletimer_start(&authtoken->expiretime, CONFIG_LASTLINK_AUTHTOKEN_EXPIRE_TIME*1000);

        //ESP_LOGI(TAG, "%s: lastlink_auth_token \"%s\" for user \"%s\"", __func__, authtoken->token, authtoken->username);
        /* Set context to authtoken */
        session_context_t *session = (session_context_t*) req->sess_ctx;
        assert(session != NULL);

        if (session != NULL) {

            /* Make copy of authtoken for this session (freed when session is released) */
ESP_LOGI(TAG, "%s: authtoken '%s' new '%s' for username '%s'", __func__, session->private_context ? (const char*) session->private_context : "NONE", (char*) authtoken->token, authtoken->username);

            /* If new or different, release (if different) and assign */
            if (session->private_context == NULL || strcmp(session->private_context, authtoken->token) != 0) {
                if (session->private_context != NULL) {
                    free((void*) session->private_context);
                }
           
                /* This might not be necessary ... */
                session->private_context = strdup(authtoken->token);

                set_session_var(session, "authtoken", authtoken->token);
            }

            set_session_var(session, "username", authtoken->username);  
        }
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return authtoken != NULL;
}

static const char *get_file_type(const char *uri)
{
    size_t len;

    char *p = strchr(uri, '?');
    if (p != NULL) {
        len = p - uri;
    } else {
        len = strlen(uri);
    }

    if (strncmp(uri + len - 3, ".js", 3) == 0) {
        return "application/javascript";
    } else if (strncmp(uri + len - 4, ".css", 4) == 0) {
        return "text/css";
    } else if (strncmp(uri + len - 4, ".ico", 4) == 0) {
        return "image/png";
    } else {
        return "text/html";
    }
}

static const char *get_file_prefix(const char *uri)
{
    size_t len;

    char *p = strchr(uri, '?');
    if (p != NULL) {
        len = p - uri;
    } else {
        len = strlen(uri);
    }

    if (strncmp(uri + len - 3, ".js", 3) == 0) {
        return "/static";
    } else if (strncmp(uri + len - 4, ".css", 4) == 0) {
        return "/static";
    } else if (strncmp(uri + len - 4, ".ico", 4) == 0) {
        return "/static";
    } else {
        return NULL;
    }
}

static bool protected_file_wanted(const char *uri)
{
    size_t len;

    char *p = strchr(uri, '?');
    if (p != NULL) {
        len = p - uri;
    } else {
        len = strlen(uri);
    }

    if (strncmp(uri + len - 3, ".js", 3) == 0) {
        return false;
    } else if (strncmp(uri + len - 4, ".css", 4) == 0) {
        return false;
    } else if (strncmp(uri + len - 4, ".ico", 3) == 0) {
        return false;
    } else if (strncmp(uri + len - 6, "/login", 6) == 0) {
        return false;
    } else {
        return true;
    }
}

/* An HTTP GET handler that requires validation of user */
static esp_err_t get_file_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;

    allocate_session(req);

    if (protected_file_wanted(req->uri) && !check_if_logged_in(req)) {
        /* For html files we need to be logged in so all roads lead to "login" */
        ret = redirect(req, "/login");
    } else {

        httpd_resp_set_type(req, get_file_type(req->uri));

        char temp_buffer[TEMP_URL_BUFFER_LEN];
        const char *pathname = get_pathname_from_uri(req->uri, get_file_prefix(req->uri), temp_buffer, sizeof(temp_buffer));
        ret = httpd_resp_send_file(req, pathname);
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return ret;
}

static const httpd_uri_t user_get_uri = {
    .uri       = "/*",
    .method    = HTTP_GET,
    .handler   = get_file_handler
};


static esp_err_t login_post_handler(httpd_req_t *req)
{
    allocate_session(req);

ESP_LOGI(TAG, "%s: uri '%s'", __func__, req->uri);

    session_context_t *session = (session_context_t *) req->sess_ctx;
    assert(session != NULL);

    uuid_text_trimmed_t user_token;

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

    //ESP_LOGI(TAG, "%s: content '%s'", __func__, content);

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

        if (authenticate(username, password, user_token)) {

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
ESP_LOGD(TAG, "%s: user_token %s", __func__, user_token);
  #ifdef CONFIG_LASTLINK_HTTPS_SERVER_ENABLED
        asprintf(&cookie, "__Secure-lastlink_auth_token=%s; Max-Age=3600; Secure", user_token);
  #else
        asprintf(&cookie, "lastlink_auth_token=%s; Max-Age=3600;", user_token);
  #endif
        httpd_resp_set_hdr(req, "Set-Cookie", cookie);

        next_uri = "/"; 
    } else {
        cookie = NULL;
        next_uri = "/login";
    }


    /* Redirect to the page */
    esp_err_t err = redirect(req, next_uri);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    free((void*) cookie);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return err;
}

static const httpd_uri_t login_post_uri = {
    .uri       = "/login",
    .method    = HTTP_POST,
    .handler   = login_post_handler
};

#ifdef CONFIG_LASTLINK_HTTPS_SERVER_ENABLED
static httpd_handle_t start_https_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting https server");

    httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();
    config.httpd.uri_match_fn = httpd_uri_match_wildcard;
    // config.httpd.stack_size   = 30000;  /* Increased from default of 10240 */

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
    httpd_register_uri_handler(server, &login_post_uri);               /* The 'login' page POST */
    httpd_register_uri_handler(server, &user_get_uri);                 /* All gets pass through here */
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, NULL);     /* Anything else if unavailable */

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    return server;
}

static void stop_https_webserver(httpd_handle_t server)
{
#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    // Stop the httpd server
    httpd_ssl_stop(server);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

static void disconnect_https_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        stop_https_webserver(*server);
        *server = NULL;
    }
#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

static void connect_https_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        *server = start_https_webserver();
    }
#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

#ifdef CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED
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
#endif /* CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED */

static httpd_handle_t https_server_handler = NULL;
#endif /* CONFIG_LASTLINK_HTTPS_SERVER_ENABLED */

#if defined(CONFIG_LASTLINK_HTTP_SERVER_ENABLED) || defined(CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED)
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
#ifdef CONFIG_LASTLINK_HTTPS_SERVER_ENABLED
 #ifdef CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED
    httpd_register_uri_handler(server, &redirect_handler);
 #endif /* CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED */
#else /* HTTPS is not enabled so enable for HTTP */
    /* Running only HTTP */
    httpd_register_uri_handler(server, &login_post_uri);               /* The 'login' page POST */
    httpd_register_uri_handler(server, &user_get_uri);                 /* All gets pass through here */
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, NULL);     /* Anything else if unavailable */
#endif /* ! defined(CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED) */

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

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

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

static void connect_http_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        *server = start_http_webserver();
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

static httpd_handle_t http_server_handler = NULL;

#endif /* defined(CONFIG_LASTLINK_HTTP_SERVER_ENABLED) || defined(CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED) */



void httpd_server_start(void)
{
    authtoken_lock = os_create_recursive_mutex();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Register event handlers to start servers when Wi-Fi is connected
     * and stop server when disconnection happens.
     */

#ifdef CONFIG_LASTLINK_HTTPS_SERVER_ENABLED
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_https_handler, &https_server_handler));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &disconnect_https_handler, &https_server_handler));
#endif

    /* Also start an http server (no ssl) to redirect to https or to server pages if no HTTPS */
#if defined(CONFIG_LASTLINK_HTTP_SERVER_ENABLED) || defined(CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED)
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_http_handler, &http_server_handler));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &disconnect_http_handler, &http_server_handler));
#endif /* defined(CONFIG_LASTLINK_HTTP_SERVER_ENABLED) || defined(CONFIG_LASTLINK_HTTP_REDIRECT_ENABLED)*/

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    ESP_ERROR_CHECK(network_connect());

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}

void http_server_stop(void)
{
    ESP_ERROR_CHECK(network_disconnect());
    ESP_ERROR_CHECK(esp_event_loop_delete_default());

    release_authtoken_list();

    os_delete_mutex(authtoken_lock);
    authtoken_lock = NULL;
}

#endif /* CONFIG_LASTLINK_WEB_SERVER_ENABLED */
