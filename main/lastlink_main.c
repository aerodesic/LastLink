/*
 * LastLink main
 */
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/uart.h"

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

#include "commands.h"
#include "ssd1306_i2c.h"


/* TEST */
extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

const uint8_t *xyz = server_root_cert_pem_end;
/* END TEST */

const char* TAG = "lastlink";

static const char* default_config[] = DEFAULT_CONFIG;

#define NUMLINES 6
char lines[NUMLINES][16];
int nextline = 0;

static display_t *display;

static void add_line_to_buffer(const char* buffer)
{
    /* Scroll the text */
    if (nextline == NUMLINES) {
        memcpy(lines, lines + 1, sizeof(lines) - sizeof(lines[0]));
        nextline = NUMLINES - 1;
    }
    strcpy(lines[nextline], buffer);

    nextline++;

    display->hold(display);
    display->draw_rectangle(display, 0, 12, 127, 52, draw_flag_clear);
    for (int line = 0; line < NUMLINES && lines[line] != NULL; ++line) {
        display->draw_text(display, 0, 12 + line*8, lines[line]);
    }
    display->show(display);
}

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
    /* initialize the lastlink network */
    #if CONFIG_LASTLINK_ADDRESS_OVERRIDE
        linklayer_init(CONFIG_LASTLINK_ADDRESS_OVERRIDE, get_config_int("lastlink.flags", 0), get_config_int("lastlink.announce", 0));
    #else
        linklayer_init(get_config_int("lastlink.address", 1), get_config_int("lastlink.flags", 0), get_config_int("lastlink.announce", 0));
    #endif

    linklayer_set_debug(true);

#if 1
    // start_commands(0, 1);
    ESP_LOGE(TAG, "stdin.fd %d stdout.fd %d", fileno(stdin), fileno(stdout));
    start_commands(fileno(stdin), fileno(stdout));
#endif

    display = ssd1306_i2c_create(DISPLAY_FLAGS_DEFAULT);
    display->draw_text(display, 0, 0, "LastLink v1.0");
    
    os_queue_t queue = linklayer_set_promiscuous_mode(true);

    int tick_count = 0;

    packet_t *packet; 
    while (true) {
        char buffer[16];
        if (os_get_queue_with_timeout(queue, (os_queue_item_t*) &packet, 1000)) {
            snprintf(buffer, sizeof(buffer), "%04x %04x %02x",
                     get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN),
                     get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                     get_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN));
            add_line_to_buffer(buffer);
        } else {
            snprintf(buffer, sizeof(buffer), "T %d", ++tick_count);
            add_line_to_buffer(buffer);
        }
    }
}

