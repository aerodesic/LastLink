//
// DHT 'driver'.
//

#include "sdkconfig.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#define TAG "dht"

#include "esp_system.h"
#include "esp_log.h"
#include "esp32/clk.h"

#include "os_specific.h"

#include "xtensa/core-macros.h"

#include "dht.h"

#define DHT_TIMEOUT_VALUE  2000

#define BIT_WIDTH_1        120  // Nominal uS in a '1' bit
#define BIT_WIDTH_0        70   // Nominal uS in a '0' bit
#define BIT_WIDTH_ACK      160  // Nominal uS in ack bit

#define MIN_READ_INTERVAL  2000 /* Wait at least 2 seconds between actual readings */

static int                 dht_int_count;
static os_queue_t          dht_queue;
// static esp_timer_handle_t  dht_timer;
static bool                dht_running;
static bool                dht_initialized;
static uint32_t            last_timer_val;

//
// As interrupts occur, low going edges (end of the event) put the time (in XTHAL ticks) into
// a queue to be read at user level.
//
// When a message starts, the first bit should be ignored (because it's the first low-going edge of the data.
// Second bit should be for about 160 uS (two 80uS edges) of the 'Acknowledge' pulse
// Subsequence 40 bits of data will be either (approximately) 77 uS (for 0) and 120 uS for 1.
//
static void dht_interrupt(void* ignored)
{
    uint32_t value;

    ++dht_int_count;

    // Get current timer elapsed
//    int64_t timer_val = esp_timer_get_time();
    uint32_t timer_val = XTHAL_GET_CCOUNT();

    if (! dht_running) {
        value = 0;
        dht_running = true;
    } else {
        value = (uint32_t) (timer_val - last_timer_val);
    }
        
    last_timer_val = timer_val;
         
    bool awakened;
    os_put_queue_from_isr(dht_queue, (os_queue_item_t) &value, &awakened);

    if (awakened) {
        portYIELD_FROM_ISR();
    }
}


dht_ret_t dht_read(dht_value_t* rh, dht_value_t* temperature)
{
    dht_ret_t rc = DHT_OK;

    static dht_value_t old_rh;
    static dht_value_t old_temp;
    static dht_ret_t   old_rc = DHT_ERROR;
    static uint64_t    old_read_time;

    dht_int_count = 0;

    if (! dht_initialized) {
        dht_queue = os_create_queue(50, sizeof(uint32_t));

        if (! os_attach_gpio_interrupt(CONFIG_DHT_PIN, GPIO_INTR_DISABLE, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, dht_interrupt, (void*) NULL)) {
            rc = DHT_ERROR;
        }
        // ESP_LOGD(TAG, "dht_initialize rc %d\n", rc);

        dht_initialized = true;
    }

    uint64_t now = get_milliseconds();

    /* Don't do an actual read unless it's been long enough */ 
    if (now - old_read_time >= MIN_READ_INTERVAL) {
        old_read_time = now;

        unsigned char message[5] = { 0 };

        if (rc == DHT_OK) {

            // Configure pin as output to generate pulse
            gpio_set_direction(CONFIG_DHT_PIN, GPIO_MODE_OUTPUT);

            // Pull low
            gpio_set_level(CONFIG_DHT_PIN, 0);

            // Wait 20 millseconds
            os_delay(20);

            // Reset queue
            os_reset_queue(dht_queue);

            // Return to input and interruptable
            gpio_set_intr_type(CONFIG_DHT_PIN, GPIO_INTR_NEGEDGE);
            gpio_set_direction(CONFIG_DHT_PIN, GPIO_MODE_INPUT);

            uint32_t count = 0;

            // Skip first to bits.  #1 is due to leading edge of first 'ack' bit.
            // #2 is end of ack bit
            // 3..42 will be the 40 data bits.
            if (! os_get_queue_with_timeout(dht_queue, (os_queue_item_t) &count, DHT_TIMEOUT_VALUE) ||
                ! os_get_queue_with_timeout(dht_queue, (os_queue_item_t) &count, DHT_TIMEOUT_VALUE)) {
                ESP_LOGD(TAG, "Timeout1");
                rc = DHT_TIMEOUT;
            } else {
                // ESP_LOGD(TAG, "count %u", count);

                // Compute slicing point ratiometrically.  
                // uint32_t slice = (uint32_t) (0.000100 / (1.0 / (double) esp_clk_cpu_freq()));
                uint32_t slice = count * ((BIT_WIDTH_1 + BIT_WIDTH_0) / 2) / BIT_WIDTH_ACK;

                //ESP_LOGD(TAG, "slice %u", slice);
    
//rc = DHT_OK;  // Force scan

                for (int bit = 0; rc == DHT_OK && bit < 40; ++bit) {
                    if (os_get_queue_with_timeout(dht_queue, (os_queue_item_t) &count, DHT_TIMEOUT_VALUE)) {
                        // ESP_LOGI(TAG, "Bit: %d: %d (%u)", bit, (count > slice) ? 1 : 0, count);

                        // Add to message if a 1
                        if (count > slice) {
                            message [ bit / 8 ] |= 0x80 >> (bit % 8);
                        }
                    } else {
                        rc = DHT_TIMEOUT;
                    }
                }

                // Disable interrupt
                gpio_set_intr_type(CONFIG_DHT_PIN, GPIO_INTR_DISABLE);
            }
        }

        // ESP_LOGD(TAG, "DHT: msg: %02x %02x %02x %02x %02x (%d)", message[0], message[1], message[2], message[3], message[4], rc);

        if (rc == DHT_OK) {
            /* Check checksum in final byte */
            if (((message[0] + message[1] + message[2] + message[3]) & 0xFF) != message[4]) {
                rc = DHT_CHECKSUM;
                ESP_LOGD(TAG, "DHT: msg: %02x %02x %02x %02x %02x (%d)", message[0], message[1], message[2], message[3], message[4], rc);
            }
        }

        if (rc == DHT_OK) {
            /* Compute and return rh */
            old_rh = (dht_ret_t) (message[0]) + (dht_ret_t) (message[1]) / 10.0;

            /* Compute and retur temp */
            old_temp = (dht_ret_t) (message[2]) + (dht_ret_t) (message[3]) / 10.0;

            //ESP_LOGD(TAG, "DHT: msg: %02x %02x %02x %02x %02x", message[0], message[1], message[2], message[3], message[4]);
        }

        old_rc = rc;
    }

    if (rh != NULL) {
        *rh = old_rh;
    }
    if (temperature != NULL) {
        *temperature = old_temp;
    }

    rc = old_rc;
    // ESP_LOGD(TAG, "dht_int_count %d", dht_int_count);

    return rc;
}

