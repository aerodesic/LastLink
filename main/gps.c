/*
 * gps.c
 *
 */

#include "sdkconfig.h"

#ifdef CONFIG_LASTLINK_SENSORS_GPS_ENABLE

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>

#include "os_specific.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nmea_parser.h"

#ifdef CONFIG_NMEA_POWER_CONTROL_LDO3
#include "power_manager.h"
#endif /* CONFIG_NMEA_POWER_CONTROL_LDO3 */

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
#include "commands.h"
#endif

#include "sensors.h"

#define TAG  "gps"

#define TIME_ZONE (0)    //UTC
#define YEAR_BASE (2000) //date in GPS starts from 2000


static os_mutex_t gps_lock;
static gps_t      gps_data;

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id) {
        case GPS_UPDATE: {
            os_acquire_recursive_mutex(gps_lock);
            /* Copy to global data structure */
            memcpy(&gps_data, (gps_t*) event_data, sizeof(gps_t));
            os_release_recursive_mutex(gps_lock);
            break;
        }

        case GPS_UNKNOWN: {
            /* print unknown statements */
            ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
            break;
        }

        default: {
            break;
        }
    }
}

static bool process_gps_location(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool ok = false;
    if (transaction == SENSOR_READ) {
        os_acquire_recursive_mutex(gps_lock);
        float latitude = gps_data.latitude;
        float longitude = gps_data.longitude; 
        os_release_recursive_mutex(gps_lock);

        // snprintf(reply_buffer, reply_len, "%.05f°N %.05f°E", latitude, longitude);
        snprintf(reply_buffer, reply_len, "%.05f, %.05f", latitude, longitude);
        ok = true;
    }
    return ok;
}

static bool process_gps_sats(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool ok = false;
    if (transaction == SENSOR_READ) {
        os_acquire_recursive_mutex(gps_lock);
        uint8_t sats_in_use = gps_data.sats_in_use;
        uint8_t sats_id_in_use[GPS_MAX_SATELLITES_IN_USE];
        memcpy(sats_id_in_use, gps_data.sats_id_in_use, sats_in_use);
        os_release_recursive_mutex(gps_lock);

        char* p = reply_buffer;
        size_t r = reply_len;
        for (int sat = 0; sat < sats_in_use; ++sat) {
            int used = snprintf(p, r, "%d ", sats_id_in_use[sat]);
            p += used;
            r -= used;
        } 
        /* Remove last blank appended */
        if (p != reply_buffer) {
            --p;
        }
        /* NUL terminate buffer */
        p[0] = '\0';
        ok = true;
    }
    return ok;
}

static bool process_gps_datetime(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool ok = false;
    if (transaction == SENSOR_READ) {
        os_acquire_recursive_mutex(gps_lock);
        gps_time_t tim = gps_data.tim;
        gps_date_t date = gps_data.date;
        os_release_recursive_mutex(gps_lock);

        /* Answer in ISO-8601 format */
        snprintf(reply_buffer, reply_len, "%04d-%02d-%02dT%02d:%02d:%02d +0000",
                 date.year + YEAR_BASE, date.month, date.day,
                 tim.hour, tim.minute, tim.second);

        ok = true;
    }
    return ok;
}

static bool process_gps_altitude(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool ok = false;
    if (transaction == SENSOR_READ) {
        os_acquire_recursive_mutex(gps_lock);
        float altitude = gps_data.altitude;
        os_release_recursive_mutex(gps_lock);
        snprintf(reply_buffer, reply_len, "%.02f", altitude);
        ok = true;
    }
    return ok;
}

static bool process_gps_speed(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool ok = false;
    if (transaction == SENSOR_READ) {
        os_acquire_recursive_mutex(gps_lock);
        float speed = gps_data.speed;
        os_release_recursive_mutex(gps_lock);
        snprintf(reply_buffer, reply_len, "%f", speed);
        ok = true;
    }
    return ok;
}

static bool process_gps_fix(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool ok = false;
    if (transaction == SENSOR_READ) {
        os_acquire_recursive_mutex(gps_lock);
        gps_fix_t      fixtype = gps_data.fix;
        gps_fix_mode_t fixmode = gps_data.fix_mode;
        bool           fixvalid = gps_data.valid;
        os_release_recursive_mutex(gps_lock);

        const char *type;
        const char* mode;

        switch (fixtype) {
            case GPS_FIX_GPS:  type = "GPS"; break;
            case GPS_FIX_DGPS: type = "DGPS"; break;
            default:           type = "Invalid"; break;
        }

        switch (fixmode) {
            case GPS_MODE_2D:  mode = "/2D";     break;
            case GPS_MODE_3D:  mode = "/3D";     break;
            default:           mode = "";       break;
        }

        snprintf(reply_buffer, reply_len, "%s%s%s", type, mode, fixvalid ? "" : " invalid");

        ok = true;
    }
    return ok;
}

static bool process_gps_cog(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool ok = false;
    if (transaction == SENSOR_READ) {
        os_acquire_recursive_mutex(gps_lock);
        float cog = gps_data.cog;
        os_release_recursive_mutex(gps_lock);

        snprintf(reply_buffer, reply_len, "%.2f", cog);

        ok = true;
    }
    return ok;
}

#ifdef CONFIG_AXP192_ENABLE
#ifdef CONFIG_NMEA_POWER_CONTROL_LDO3
static bool process_gps_power(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool ok = false;
    if (transaction == SENSOR_READ) {
        os_acquire_recursive_mutex(gps_lock);
        int value = axp192_test_ldo3();
        os_release_recursive_mutex(gps_lock);

        snprintf(reply_buffer, reply_len, "%s", value >= 0 ? (value != 0 ? "1" : "0") : "error");

        ok = true;

    } else if (transaction == SENSOR_WRITE) {
        va_list ap;
        va_start(ap, reply_len);
        const char* value = va_arg(ap, const char*);
        bool state = strtol(value, NULL, 0);
        ESP_LOGI(TAG, "Setting gps power to %s", state ? "On" : "Off");
        ok = axp192_enable_ldo3(state);
        va_end(ap);
    }

    return ok;
}
#endif /* CONFIG_NMEA_POWER_CONTROL_LDO3 */
#endif /* CONFIG_AXP192_ENABLE */

static nmea_parser_handle_t nmea_handle;

bool start_gps(void)
{
    if (gps_lock == NULL) {
        gps_lock = os_create_recursive_mutex();
    }

    nmea_parser_config_t config = {
        .uart = {
            .uart_port = CONFIG_LASTLINK_SENSORS_GPS_PORT_NUMBER,
            .rx_pin = CONFIG_LASTLINK_SENSORS_GPS_RX_PIN,
//           .tx_pin = CONFIG_LASTLINK_SENSORS_GPS_TX_PIN,
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .event_queue_size = 16,
         }
    };

    os_acquire_recursive_mutex(gps_lock);

    nmea_handle = nmea_parser_init(&config);

    if (nmea_handle != NULL) {
        if (nmea_parser_add_handler(nmea_handle, gps_event_handler, NULL) != ESP_OK) {
            nmea_parser_deinit(nmea_handle);
            ESP_LOGE(TAG, "%s: parser created but not added to handler", __func__);
            nmea_handle = NULL;
        } else {
            /* Add sensors */
#ifdef CONFIG_AXP192_ENABLE
#ifdef CONFIG_NMEA_POWER_CONTROL_LDO3
            if (! register_sensor("gpspower", SENSOR_TYPE_OUTPUT, "Power",   process_gps_power,   NULL)) {
            }
#endif /* CONFIG_NMEA_POWER_CONTROL_LDO3 */
#endif /* CONFIG_AXP192_ENABLE */
            if (! register_sensor("location", SENSOR_TYPE_INPUT, "gps",      process_gps_location, NULL)) {
            }
            if (! register_sensor("gpstime",  SENSOR_TYPE_INPUT, "utc",      process_gps_datetime, NULL)) {
            }
            if (! register_sensor("altitude", SENSOR_TYPE_INPUT, "M",        process_gps_altitude, NULL)) {
            }
            if (! register_sensor("speed",    SENSOR_TYPE_INPUT, "M/S",      process_gps_speed,    NULL)) {
            }
            if (! register_sensor("fix",      SENSOR_TYPE_INPUT, "",         process_gps_fix,      NULL)) {
            }
            if (! register_sensor("sats",     SENSOR_TYPE_INPUT, "in view",  process_gps_sats,     NULL)) {
            }
            if (! register_sensor("cog",      SENSOR_TYPE_INPUT, "degrees",  process_gps_cog,      NULL)) {
            }
        }
    }

    os_release_recursive_mutex(gps_lock);

    return nmea_handle != NULL;
}

bool stop_gps(void)
{
    os_acquire_recursive_mutex(gps_lock);

    nmea_parser_remove_handler(nmea_handle, gps_event_handler);

    bool ok  = nmea_parser_deinit(nmea_handle) == ESP_OK;

    os_release_recursive_mutex(gps_lock);

    return ok;
}

#endif /* CONFIG_LASTLINK_SENSORS_GPS_ENABLE */
