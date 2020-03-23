/*
 * LastLink main
 */
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portable.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_init.h"
#include "spiffs.h"
#include "nvs_support.h"

#include "linklayer.h"
#include "lsocket.h"
#include "configdata.h"
#include "default_config.h"
#include "os_freertos.h"
#include "packets.h"

/* TEST */
extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

const uint8_t *xyz = server_root_cert_pem_end;
/* END TEST */

const char* TAG = "lastlink";

static const char* default_config[] = DEFAULT_CONFIG;

#if CONFIG_LASTLINK_PING_ADDRESS
static int button_interrupts;

static void test_button_handler(void* param)
{
    /* Create a ping packet to broadcast */
    ++button_interrupts;
}
#endif

void app_main(void)
{
#if 1
    //Initialize NVS
    if (init_nvs() == ESP_OK) {
        /* pass */
    }
#endif

#if 1
    if (init_spiffs() == ESP_OK) {
	/* Try to open .config and if not found, format the spiffs */
	FILE *fp = fopen(CONFIG_LASTLINK_CONFIG_FILE, "r");
	if (fp == NULL) {
            /* Format the device */
	    format_spiffs();
        } else {
            fclose(fp);
        }
    }


    ESP_LOGD(TAG, "About to load configuration");

    /* load config file */
    init_configuration(CONFIG_LASTLINK_CONFIG_FILE, default_config);

    ESP_LOGD(TAG, "zap = '%s'", get_config_str("zip", "not found"));
    ESP_LOGD(TAG, "section1.this = '%s'", get_config_str("section1.this", "not found"));
    ESP_LOGD(TAG, "section1.section2.blot = '%s'", get_config_str("section1.section2.blot", "not found"));
    ESP_LOGD(TAG, "zorch = '%s'", get_config_str("zorch", "not found"));
    ESP_LOGD(TAG, "section1.section2.section3.only = '%s'", get_config_str("section1.section2.section3.only", "not found"));
    ESP_LOGD(TAG, "notfound = '%s'", get_config_str("notfound", "not found"));
    ESP_LOGD(TAG, "address %d flags 0x%02x announce %d", get_config_int("lastlink.address", 99), get_config_int("lastlink.flags", 99), get_config_int("lastlink.announce", -1));
#endif

#if 1
    /* initialize the lastlink network */
    #if CONFIG_LASTLINK_ADDRESS_OVERRIDE
        linklayer_init(CONFIG_LASTLINK_ADDRESS_OVERRIDE, get_config_int("lastlink.flags", 0), get_config_int("lastlink.announce", 0));
    #else
        linklayer_init(get_config_int("lastlink.address", 1), get_config_int("lastlink.flags", 0), get_config_int("lastlink.announce", 0));
    #endif
    linklayer_set_debug(true);
#endif

#if CONFIG_LASTLINK_PING_ADDRESS
    if (os_attach_gpio_interrupt(0, GPIO_PIN_INTR_NEGEDGE, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, test_button_handler, (void*) 0)) {
        ESP_LOGI(TAG, "Button interrupt attached");
    } else {
        ESP_LOGI(TAG, "Button interrupt attach failed");
    }
#endif

#if 0
    /* This becomes the main thread */
    wifi_init_softap();
#else
    int last_packet_count = -99;

  #if CONFIG_LASTLINK_PING_ADDRESS
    int last_button_interrupts = button_interrupts;
  #endif

    while(true) {
        for (int count = 0; count < 20; ++count) {
            esp_task_wdt_reset();

            os_delay(100);

  #if CONFIG_LASTLINK_PING_ADDRESS
            if (button_interrupts != last_button_interrupts) {
                int paths[100];
                ESP_LOGI(TAG, "Sending ping");
                int path_len = ping(CONFIG_LASTLINK_PING_ADDRESS, paths, 100, 10000);
                if (path_len < 0) {
                    ESP_LOGI(TAG, "Ping error %d", path_len);
                } else {
                    print("Path:");
                    for (int path = 0; path < path_len; ++path) {
                        printf(" %d", paths[path]);
                    }
                    printf("\n");
                }
                last_button_interrupts = button_interrupts;
                last_packet_count = -1; /* Force display of available packets */
            }
  #endif

            int packet_count = available_packets();
            if (packet_count != last_packet_count) {
                ESP_LOGI(TAG, "%d free packets", available_packets());
                last_packet_count = packet_count;
            }
        }
    }
#endif
}

