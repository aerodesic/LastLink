/*
 * wifi_init.h
 */
#ifndef __wifi_init_h_defined
#define __wifi_init_h_defined

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

void wifi_init_softap(void);

#endif /* __wifi_init_h_defined */
