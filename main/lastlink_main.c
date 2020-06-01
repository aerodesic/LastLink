/*
 * LastLink main
 */
#include "sdkconfig.h"

#define VERSION "1.0"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portable.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "bootloader_random.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_init.h"
#include "spiffs.h"
#include "nvs_support.h"

#include "linklayer.h"
#include "lsocket.h"
#include "configdata.h"
#include "default_config.h"
#include "os_specific.h"
#include "packets.h"
#include "service_names.h"

#include "commands.h"
#include "ssd1306_i2c.h"
#include "packet_window.h"


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

display_t *display;

static void add_line_to_buffer(const char* fmt, ...)
{
    /* Scroll the text */
    if (nextline == NUMLINES) {
        memcpy(lines, lines + 1, sizeof(lines) - sizeof(lines[0]));
        nextline = NUMLINES - 1;
    }

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(lines[nextline++], sizeof(lines[nextline]), fmt, ap);
    //nextline++;
    va_end(ap);

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


    init_commands();

    display = ssd1306_i2c_create(DISPLAY_FLAGS_DEFAULT);
    display->contrast(display, get_config_int("display.contrast", 128));

    if (init_spiffs() == ESP_OK) {
	/* Try to open .config and if not found, format the spiffs */
        FILE *fp = fopen(CONFIG_LASTLINK_CONFIG_FILE, "r");
	    if (fp == NULL) {
            /* Format the device */
            display->draw_text(display, 0, 32, "Formatting...");
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

    // linklayer_set_debug(true);
    bool listen_only = get_config_int("lastlink.listen_only", 0) != 0;

    linklayer_set_listen_only(get_config_int("lastlink.listen_only", 0));

#if CONFIG_LASTLINK_RECEIVE_ONLY_FROM_TABLE
    linklayer_set_receive_only_from(get_config_str("lastlink.receive_only_from", ""));
#endif

#if CONFIG_LASTLINK_SERVICE_NAMES_ENABLE
    init_service_names();
#endif

    start_commands(stdin, stdout);

    bootloader_random_enable();
    ESP_LOGI(TAG, "random number 1 %d", esp_random());
    ESP_LOGI(TAG, "random number 2 %d", esp_random());
    ESP_LOGI(TAG, "random number 3 %d", esp_random());

    display->clear(display);

    char *buffer;
    asprintf(&buffer, "Lastlink #%d", get_config_int("lastlink.address", 0));
    display->draw_text(display, 0, 0, buffer);
    free((void*) buffer);
    
#if 0
    wifi_init_softap();
#endif

    init_packet_window();

    if (listen_only) {
        linklayer_set_listen_only(true);

        os_queue_t queue = linklayer_set_promiscuous_mode(true);

        packet_t *packet; 
        while (true) {
            if (os_get_queue(queue, (os_queue_item_t*) &packet)) {

                if (packet == NULL) {
                    ESP_LOGE(TAG, "%s: null packet", __func__);
                } else {
                    /* Format for log message */
                    const char *p_message = linklayer_format_packet(packet);

                    release_packet(packet);

                    ESP_LOGI(TAG, "%s", p_message);

                    free((void*) p_message);
                }
            }
        }
    } else {
        while (true) {
            os_delay(1000);
        }
    }
}

