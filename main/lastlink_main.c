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

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_init.h"
#include "spiffs.h"
#include "nvs_support.h"

#include "linklayer.h"
#include "configdata.h"
#include "default_config.h"
#include "os_freertos.h"

/* TEST */
extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

const uint8_t *xyz = server_root_cert_pem_end;
/* END TEST */

const char* TAG = "lastlink";

#if 1
static char cTaskListBuf[4000];
#endif

static const char* default_config[] = DEFAULT_CONFIG;

static int button_interrupts;

static void test_button_handler(void* param)
{
    ++button_interrupts;
}

void app_main(void)
{
#if 1
    //Initialize NVS
    if (init_nvs() == ESP_OK) {
        /* pass */
    }
#endif

#if 0
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
    linklayer_init(get_config_int("lastlink.address", 1), get_config_int("lastlink.flags", 0), get_config_int("lastlink.announce", 0));
    linklayer_set_debug(true);
#endif


#if 1
    /* This becomes the main thread */
    wifi_init_softap();
#else
    if (os_attach_gpio_interrupt(0, GPIO_PIN_INTR_NEGEDGE, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, test_button_handler, (void*) 0)) {
        ESP_LOGI(TAG, "Button interrupt attached");
    } else {
        ESP_LOGI(TAG, "Button interrupt attach failed");
    }

    while(true) {
       os_delay(1000);
  #if 1
       vTaskList(cTaskListBuf);
       puts(cTaskListBuf);
    #if 1
       printf("button interrupts: %d\n", button_interrupts);
    #endif
  #else
       printf("xPortCanYield %d Tasks %u\n", xPortCanYield(), uxTaskGetNumberOfTasks());
  #endif
    // printf("Tick\n");
    }
#endif
}

