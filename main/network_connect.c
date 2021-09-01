/* Common functions for protocol examples, to establish Wi-Fi or Ethernet connection.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#include "sdkconfig.h"
#ifdef CONFIG_LASTLINK_WEB_SERVER_ENABLED

#include <string.h>
#include "network_connect.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#ifdef CONFIG_LASTLINK_CONNECT_IPV6
#define MAX_IP6_ADDRS_PER_NETIF (5)
#define NR_OF_IP_ADDRESSES_TO_WAIT_FOR (s_active_interfaces*2)

#if defined(CONFIG_LASTLINK_CONNECT_IPV6_PREF_LOCAL_LINK)
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE ESP_IP6_ADDR_IS_LINK_LOCAL
#elif defined(CONFIG_LASTLINK_CONNECT_IPV6_PREF_GLOBAL)
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE ESP_IP6_ADDR_IS_GLOBAL
#elif defined(CONFIG_LASTLINK_CONNECT_IPV6_PREF_SITE_LOCAL)
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE ESP_IP6_ADDR_IS_SITE_LOCAL
#elif defined(CONFIG_LASTLINK_CONNECT_IPV6_PREF_UNIQUE_LOCAL)
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE ESP_IP6_ADDR_IS_UNIQUE_LOCAL
#endif // if-elif CONFIG_LASTLINK_CONNECT_IPV6_PREF_...

#else
#define NR_OF_IP_ADDRESSES_TO_WAIT_FOR (s_active_interfaces)
#endif

/* Local min function */
#define min(a,b) ({ __typeof__ (a) _a = (a);  __typeof__ (b) _b = (b);  _a < _b ? _a : _b; })


static int s_active_interfaces = 0;
static esp_netif_t *s_lastlink_esp_netif = NULL;

#ifdef CONFIG_LASTLINK_CONNECT_IPV6
static esp_ip6_addr_t s_ipv6_addr;

/* types of ipv6 addresses to be displayed on ipv6 events */
static const char *s_ipv6_addr_types[] = {
    "ESP_IP6_ADDR_IS_UNKNOWN",
    "ESP_IP6_ADDR_IS_GLOBAL",
    "ESP_IP6_ADDR_IS_LINK_LOCAL",
    "ESP_IP6_ADDR_IS_SITE_LOCAL",
    "ESP_IP6_ADDR_IS_UNIQUE_LOCAL",
    "ESP_IP6_ADDR_IS_IPV4_MAPPED_IPV6"
    };
#endif

static const char *TAG = "network_connect";

#ifdef CONFIG_LASTLINK_CONNECT_WIFI
static esp_netif_t* wifi_start(const char* ssid, const char* password);
static void wifi_stop(void);
#endif

/**
 * @brief Checks the netif description if it contains specified prefix.
 * All netifs created withing common connect component are prefixed with the module TAG,
 * so it returns true if the specified netif is owned by this module
 */
static bool is_our_netif(const char *prefix, esp_netif_t *netif)
{
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix)-1) == 0;
}

/* set up connection, Wi-Fi and/or Ethernet */
static void start(const char* ssid, const char*  password)
{

#ifdef CONFIG_LASTLINK_CONNECT_WIFI
    s_lastlink_esp_netif = wifi_start(ssid, password);
    s_active_interfaces++;
#endif
}

/* tear down connection, release resources */
static void stop(void)
{
#ifdef CONFIG_LASTLINK_CONNECT_WIFI
    wifi_stop();
    s_active_interfaces--;
#endif
}

esp_err_t network_connect(const char* ssid, const char* password)
{
    start(ssid, password);
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&stop));
    // iterate over active interfaces, and print out IPs of "our" netifs
    esp_netif_t *netif = NULL;
    esp_netif_ip_info_t ip;
    for (int i=0; i<esp_netif_get_nr_of_ifs(); ++i) {
        netif = esp_netif_next(netif);
        if (is_our_netif(TAG, netif)) {
            ESP_LOGI(TAG, "Connected to %s", esp_netif_get_desc(netif));
            ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip));

            ESP_LOGI(TAG, "- IPv4 address: " IPSTR, IP2STR(&ip.ip));
#ifdef CONFIG_LASTLINK_CONNECT_IPV6
            esp_ip6_addr_t ip6[MAX_IP6_ADDRS_PER_NETIF];
            int ip6_addrs = esp_netif_get_all_ip6(netif, ip6);
            for (int j = 0; j < ip6_addrs; ++j) {
                esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&(ip6[j]));
                ESP_LOGI(TAG, "- IPv6 address: " IPV6STR ", type: %s", IPV62STR(ip6[j]), s_ipv6_addr_types[ipv6_type]);
            }
#endif

        }
    }
    return ESP_OK;
}

esp_err_t network_disconnect(void)
{
    stop();
    return ESP_OK;
}

#ifdef CONFIG_LASTLINK_CONNECT_WIFI

static esp_netif_t* wifi_start(const char* ssid, const char* password)
{
    char *desc;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    //esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();
    // Prefix the interface description with the module TAG
    // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
    asprintf(&desc, "%s: %s", TAG, esp_netif_config.if_desc);
    esp_netif_config.if_desc = desc;
    esp_netif_config.route_prio = 128;
    //esp_netif_t *netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    esp_netif_t *netif = esp_netif_create_wifi(WIFI_IF_AP, &esp_netif_config);
    free(desc);

    //esp_wifi_set_default_wifi_sta_handlers();
    esp_wifi_set_default_wifi_ap_handlers();

    wifi_config_t wifi_config = {
        .ap = {
            .max_connection = CONFIG_LASTLINK_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .beacon_interval = 5000,   /* Slow down for power save */
        },
    };

    memcpy((char*) wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = min(strlen(ssid), sizeof(wifi_config.ap.ssid));

    if (strlen(password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        wifi_config.ap.password[0] = '\0';
    } else {
        memcpy((char*) wifi_config.ap.password, password, sizeof(wifi_config.ap.password));
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    //ESP_ERROR_CHECK(esp_wifi_connect());
    ESP_ERROR_CHECK(esp_wifi_start());

    return netif;
}

static void wifi_stop(void)
{
    //esp_netif_t *wifi_netif = get_lastlink_netif_from_desc("sta");
    esp_netif_t *wifi_netif = get_lastlink_netif_from_desc("ap");
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(wifi_netif));
    esp_netif_destroy(wifi_netif);
    s_lastlink_esp_netif = NULL;
}
#endif // CONFIG_LASTLINK_CONNECT_WIFI

esp_netif_t *get_lastlink_netif(void)
{
    return s_lastlink_esp_netif;
}

esp_netif_t *get_lastlink_netif_from_desc(const char *desc)
{
    esp_netif_t *netif = NULL;
    char *expected_desc;
    asprintf(&expected_desc, "%s: %s", TAG, desc);
    while ((netif = esp_netif_next(netif)) != NULL) {
        if (strcmp(esp_netif_get_desc(netif), expected_desc) == 0) {
            free(expected_desc);
            return netif;
        }
    }
    free(expected_desc);
    return netif;
}
#endif /* CONFIG_LASTLINK_WEB_SERVER_ENABLED */

