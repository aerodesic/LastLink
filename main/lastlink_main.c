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

#ifdef NOTUSED
static int readline(char *buffer, size_t len)
{
    int index = 0;

    bool reading = true;

    while (reading) {
        int ch = getchar();

        if (ch < 0) {
            os_delay(5);
        } else {
            if (ch == '\n') {
                buffer[index] = '\0';
                reading = false;
                putchar('\n');
            } else if (ch == '\b') {
                if (index > 0) {
                    putchar('\b');
                    putchar(' ');
                    putchar('\b');
                    --index;
                }
            } else if (isprint(ch)) {
                if (index < len-1) {
                    putchar(ch);
                    buffer[index++] = ch;
                }
            }
        }
    }

printf("readline returning '%s'\n", buffer);
    return index;
}
#endif


void app_main(void)
{
#if 1
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
#endif


    ESP_LOGD(TAG, "About to load configuration");

    /* load config file */
    init_configuration(CONFIG_LASTLINK_CONFIG_FILE, default_config);

#if 0
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

   #if 1
    // start_commands(0, 1);
    ESP_LOGE(TAG, "stdin.fd %d stdout.fd %d", fileno(stdin), fileno(stdout));
    start_commands(fileno(stdin), fileno(stdout));
   #endif

    printf("FIRST_LASTLINK_FD %d LAST_LASTLINK_FD %d LWIP_SOCKET_OFFSET %d\n", FIRST_LASTLINK_FD, LAST_LASTLINK_FD, LWIP_SOCKET_OFFSET);
#endif

#if 1
#define X1  33
#define Y1  17
#define W   64
#define H   32

    display_t *display = ssd1306_i2c_create(DISPLAY_FLAGS_DEFAULT);
    display->write_text(display, "Hello");

    for (int progress = 0; progress <= 100; ++progress) {
        display->draw_progress_bar(display, X1, Y1, W, H, 100, progress, "Hello");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int progress = 100; progress >= 0; --progress) {
        display->draw_progress_bar(display, X1, Y1, W, H, 100, progress, "Goodbye");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    //display->set_xy(display, 40, 30);
    //display->draw_line(display, 0, 0, 127, 63);
    //display->draw_line(display, 127, 0, 0, 63);
    //display->show(display);
#endif

#if 0
    /* This becomes the main thread */
    wifi_init_softap();
#endif
}

