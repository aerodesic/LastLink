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

#include <esp_https_server.h>


/* A simple example that demonstrates how to create GET and POST
 * handlers and start an HTTPS server.
*/

static const char *TAG = "LastLinkHTTPS";

static bool authenticate(const char *username, const char *password, char *token, size_t token_len)
{
    ESP_LOGI(TAG, "%s: username '%s' password '%s'", __func__, username, password);
    snprintf(token, token_len, "%s:%s", username, password);
    return true;
}

static void redirect(httpd_req_t *req, const char* uri)
{
    ESP_LOGI(TAG, "%s: to '%s'", __func__, uri);

    /* Redirect to the login page */
    httpd_resp_set_status(req, "301 Moved Permanently");
    httpd_resp_set_hdr(req, "Location", uri);
    httpd_resp_send(req, NULL, 0);
}

static bool get_auth_token(httpd_req_t *req, char *auth_token, size_t auth_token_len)
{
    size_t cookie_len = httpd_req_get_hdr_value_len(req, "Cookie");
    if (cookie_len != 0) {
        char *cookie = (char *) malloc(cookie_len);

        esp_err_t ret =  httpd_req_get_hdr_value_str(req, "Cookie", cookie, cookie_len);
        if (ret == ESP_OK) {
           ESP_LOGI(TAG, "%s: cookie is '%s'", __func__, cookie);

           char *token = strstr(cookie, "auth_token=");

           if (token != NULL) {
               token += 11;
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
    if (get_auth_token(req, auth_token, sizeof(auth_token))) {
        ESP_LOGI(TAG, "%s: auth_token %s", __func__, auth_token);
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
        /* Present the home page */
        httpd_resp_send(req, "<h1>Hello Secure World!</h1>", -1); // -1 = use strlen()
    }

    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler
};

static const char login_page_text[] = 
       "<html>\n"
         "<body>\n"
           "<form action=\"login\" method=\"post\">\n"
           // "<div class=\"imgcontainer\">\n"
           //   "<img src=\"img_avatar2.png\" alt=\"Avatar\" class=\"avatar\">\n"
           //"</div>\n"

           "<div class=\"container\">\n"
             "<label for=\"uname\"><b>Username</b></label>\n"
             "<input type=\"text\" placeholder=\"Enter Username\" name=\"uname\" required>\n"
             "<p>\n"
             "<label for=\"psw\"><b>Password</b></label>\n"
             "<input type=\"password\" placeholder=\"Enter Password\" name=\"psw\" required>\n"
  
             "<button type=\"submit\">Login</button>\n"
             "<label>\n"
               "<input type=\"checkbox\" checked=\"checked\" name=\"rem\"> Remember me\n"
             "</label>\n"
           "</div>\n"

           "<div class=\"container\" style=\"background-color:#f1f1f1\">\n"
             "<button type=\"button\" class=\"cancelbtn\">Cancel</button>\n"
             "<span class=\"psw\">Forgot <a href=\"#\">password?</a></span>\n"
           "</div>\n"
         "</form> \n"
       "</body>\n"
     "</html>\n";

static esp_err_t login_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    httpd_resp_send(req, login_page_text, sizeof(login_page_text));
    
    return ESP_OK;
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
            char cookie[65];
            snprintf(cookie, sizeof(cookie), "auth_token=%s", token);
            httpd_resp_set_hdr(req, "Cookie", cookie);
            httpd_resp_send(req, NULL, 0);  // Response body can be empty
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
