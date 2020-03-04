/*
 * LastLink main
 */
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_init.h"
#include "spiffs.h"
#include "nvs_support.h"

//#include "lastlink.h"
#include "configdata.h"
#include "default_config.h"
#include "packet.h"

const char* TAG = "lastlink";

static const char* default_config[] = DEFAULT_CONFIG;

void app_main(void)
{
    //Initialize NVS
    if (init_nvs() == ESP_OK) {
        /* pass */
    }

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

#ifdef DEBUG
    /* Initialize packet system */
    int  num_packets = init_packets(CONFIG_LASTLINK_NUM_PACKETS);
    ESP_LOGD(TAG, "init_packets(%d) returned %d", CONFIG_LASTLINK_NUM_PACKETS, num_packets);

    /* Get a free packet */
    packet_t* packet = allocate_packet();
    ESP_LOGD(TAG, "allocate_packet returned %p (use %d, length %d)", packet, packet->use, packet->length);

    /* Show packets available */
    ESP_LOGD(TAG, "number of packets available %d", packets_available());

    bool ok = release_packet(packet);
    ESP_LOGD(TAG, "release_packet returned %s", ok ? "OK" : "FAIL");

    /* Show packets available */
    ESP_LOGD(TAG, "number of packets available %d", packets_available());
#endif

    /* initialize the lastlink network */
    //lastlink_init();

    /* This becomes the main thread */
    wifi_init_softap();
}
