/*
 * LastLink main
 */
#include "sdkconfig.h"

#define ENABLE_LASTLINK_NETWORK

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

#include "spiffs.h"
#include "nvs_support.h"

#include "linklayer.h"
#include "lsocket.h"
#include "configdata.h"
//#include "default_config.h"
#include "os_specific.h"
#include "packets.h"
#include "service_names.h"

#include "commands.h"
#ifdef CONFIG_SSD1306_I2C_ENABLED
#include "ssd1306_i2c.h"
#endif /* CONFIG_SSD1306_I2C_ENABLED */

#ifdef CONFIG_LASTLINK_WEB_SERVER_ENABLED
#include "httpd_server.h"
#include "mdns_config.h"
#endif /* CONFIG_LASTLINK_WEB_SERVER_ENABLED */

#include "uuid.h"

const char* TAG = "lastlink";

#ifdef CONFIG_SSD1306_I2C_ENABLED
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
#endif /* CONFIG_SSD1306_I2C_ENABLED */

void app_main(void)
{
    //Initialize NVS
    if (init_nvs() == ESP_OK) {
        /* pass */
    }


#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    init_commands();

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

#ifdef CONFIG_SSD1306_I2C_ENABLED
    display = ssd1306_i2c_create(DISPLAY_FLAGS_DEFAULT);
    display->contrast(display, get_config_int("display.contrast", 128));
#endif /* CONFIG_SSD1306_I2C_ENABLED */

    if (init_spiffs() == ESP_OK) {
	/* Try to open .config and if not found, format the spiffs */
        FILE *fp = fopen(CONFIG_LASTLINK_CONFIG_FILE, "r");
	    if (fp == NULL) {
#ifdef CONFIG_SSD1306_I2C_ENABLED
            /* Format the device */
            display->draw_text(display, 0, 32, "Formatting...");
#endif /* CONFIG_SSD1306_I2C_ENABLED */
	        format_spiffs();
        } else {
            fclose(fp);
        }
    }

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

#ifdef ENABLE_LASTLINK_NETWORK
    ESP_LOGD(TAG, "About to load configuration");

    /* load config file */
    init_configuration(CONFIG_LASTLINK_CONFIG_FILE);
    /* initialize the lastlink network */
    #if CONFIG_LASTLINK_ADDRESS_OVERRIDE
        linklayer_init(CONFIG_LASTLINK_ADDRESS_OVERRIDE, get_config_int("lastlink.flags", 0), get_config_int("lastlink.announce", 0));
    #else
        linklayer_init(get_config_int("lastlink.address", 1), get_config_int("lastlink.flags", 0), get_config_int("lastlink.announce", 0));
    #endif

    // linklayer_set_debug(true);
    bool listen_only = get_config_int("lastlink.listen_only", 0) != 0;

    linklayer_set_listen_only(listen_only);

    //linklayer_set_channel_and_datarate(-1, get_config_int("lastlink.channel", 0), get_config_int("lastlink.datarate", 0));

#if CONFIG_LASTLINK_RECEIVE_ONLY_FROM_TABLE
    linklayer_set_receive_only_from(get_config_str("lastlink.receive_only_from", ""));
#endif

#if CONFIG_LASTLINK_SERVICE_NAMES_ENABLE
    init_service_names();

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

#endif
#endif /* DISABLED_WHILE_LOOKING_FOR_CRASH */

    start_commands(stdin, stdout);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

    bootloader_random_enable();

#ifdef NOTUSED
    ESP_LOGI(TAG, "random number 1 %d", esp_random());
    ESP_LOGI(TAG, "random number 2 %d", esp_random());
    ESP_LOGI(TAG, "random number 3 %d", esp_random());
#endif

    uuid_text_t          uuid_buf;
    uuid_text_trimmed_t  uuid_buf_trimmed;
    ESP_LOGI(TAG, "%s: uuid normal %s", __func__, uuid_gen(uuid_buf));
    ESP_LOGI(TAG, "%s: uuid trimmed %s", __func__, uuid_gen_trimmed(uuid_buf_trimmed));

#ifdef CONFIG_SSD1306_I2C_ENABLED
    display->clear(display);

    char *buffer;
    asprintf(&buffer, "LastLink #%d", get_config_int("lastlink.address", 0));
    display->draw_text(display, 0, 0, buffer);
    free((void*) buffer);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

#endif  /* CONFIG_SSD1306_I2C_ENABLED */

#ifdef CONFIG_LASTLINK_WEB_SERVER_ENABLED
    httpd_server_start();
 #ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
 #endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

 #ifdef CONFIG_LASTLINK_MDNS_ENABLED
    start_mdns_service();
 #endif /* CONFIG_LASTLINK_MDNS_ENABLED */
 #ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
 #endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
#endif /* CONFIG_LASTLINK_WEB_SERVER_ENABLED */

    printf("Node address %d\n", linklayer_node_address);

#ifdef NOTUSED
    if (listen_only) {
        linklayer_set_listen_only(true);

        os_queue_t queue = linklayer_set_promiscuous_mode(true);

        packet_t *packet;
        while (true) {
            if (os_get_queue(queue, (os_queue_item_t) &packet)) {

                if (packet == NULL) {
                    ESP_LOGE(TAG, "%s: null packet", __func__);
                } else {
                    /* Format for log message */
                    const char *p_message = linklayer_format_packet(packet);

                    release_packet(packet);

                    printf("%s\n", p_message);

                    free((void*) p_message);
                }
            }
        }
    } else {
        while (true) {
            os_delay(1000);
        }
    }
#else
    while (true) {
        os_delay(1000);
    }
#endif
}

