/*
 * packets.c
 *
 * Defines LastLink packet structure.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "packets.h"

/* Where freed packets go - where we look first */
static QueueHandle_t     free_packets_queue;
static QueueHandle_t     packet_freeing_queue;
static TaskHandle_t      packet_freeing_thread;
static SemaphoreHandle_t packet_mutex;
static int               active_packets;
static int               dropped_allocations;

static bool validate_field(packet_t *p, int from, int length);

static packet_t *allocate_packet_help(bool fromisr);
static packet_t *packet_create_help(char *buf, int length, bool fromisr);

#define TAG     "packets"

static void packet_freeing_process(void* param)
{
    packet_t* packet;

    /* Gobble packets from the freeing queue and release them */
    while (xQueueReceive(packet_freeing_queue, &packet, 0) == pdPASS) {
        packet_release(packet);
    }
}

/*
 * init_packets
 *
 * Initialize the packet store.
 *
 * Entry:
 *     num_packets     Number of available packets to allocate
 *
 * Returns true if successful.  If false, any partially constructed stuff will have been removed.
 */
bool init_packets(int num_packets)
{
    bool ok = false;

    ESP_LOGD(TAG, "init_packets: %d", num_packets);

    packet_mutex = xSemaphoreCreateRecursiveMutex();

    if (packet_mutex != NULL) {
        ESP_LOGD(TAG, "packet_mutex created");

        packet_lock();
        /* Create scavanger queue for ISR to dispose of packets. */
        packet_freeing_queue = xQueueCreate((UBaseType_t) num_packets, (UBaseType_t) sizeof(packet_t*));

        if (packet_freeing_queue != NULL) {

            /* Create thread for freeing packets */
            if (xTaskCreate(packet_freeing_process, "packet_freeing",  configMINIMAL_STACK_SIZE, NULL, 0, &packet_freeing_thread) == pdTRUE) {

                free_packets_queue = xQueueCreate((UBaseType_t) num_packets, (UBaseType_t) sizeof(packet_t*));

                if (free_packets_queue != NULL) {

                    ESP_LOGD(TAG, "free_packets_queue created");

                    ok = true;

                    int count = 0;

                    while(ok && count < num_packets) {
                        packet_t *p = (packet_t *) malloc(sizeof(packet_t));
                        if (p != NULL) {
                            memset(p, 0, sizeof(packet_t));
                            p->use = 1;
                            packet_release(p);
                        } else {
                            ok = false;
                        }
                        ++count;
                    }
                }
            }
        }
        packet_unlock();
    }

    if (!ok) {
        /* Undo failed initialization */
        packet_deinit();
    }

    return ok;
}

/*
 * Deinitialize packet handler.
 *
 * If packets are still outstanding, returns the number other all packets are freed
 * and the semaphore is released.
 *
 * If 0 is returned, all items created by init_packets will have been released.
 */
int deinit_packets(void)
{
    int packets_left = 0;

    if (packet_mutex != NULL) {
        packet_lock();

        if (free_packets_queue != NULL) {

            if (active_packets != 0) {
                packets_left = active_packets;
            } else {
                /* Release free packets */
                packet_t* packet;
                while (xQueueReceive(free_packets_queue, &packet, 0) == pdPASS) {
                    free((void*) packet);
                }
            }

            vQueueDelete(free_packets_queue);
            free_packets_queue = NULL;
        }

        packet_unlock();

        if (packets_left == 0) {
            if (packet_freeing_thread != NULL) {
                vTaskDelete(packet_freeing_thread);
                packet_freeing_thread = NULL;
                vQueueDelete(packet_freeing_queue);
                packet_freeing_queue = NULL;
            }

            vSemaphoreDelete(packet_mutex);
            packet_mutex = NULL;
        }
    }

    return packets_left;
}

/*
 * Validate the from and length fields of packet.
 */
static bool validate_field(packet_t *p, int from, int length)
{
    return p != NULL && from < p->length && from + length <= p->length;
}

/*
 * Allocate a packet of specified length.  Packet is filled with zeroes.
 */
static packet_t *allocate_packet_help(bool fromisr)
{
    ESP_LOGD(TAG, "allocate_packet_help fromisr %s", fromisr ? "YES" : "NO");

    packet_t* packet = NULL;

    ESP_LOGD(TAG, "allocate_packet_help packet_mutex acquired");

    /* Make sure free packet queue is present */
    if (free_packets_queue != NULL) {

        void* p;

        if ((fromisr && (xQueueReceiveFromISR(free_packets_queue, &p, NULL) == pdPASS)) ||
            (!fromisr && (xQueueReceive(free_packets_queue, &p, 0) == pdPASS))) {

            ESP_LOGD(TAG, "allocate_packet_help packet from queue is %p", p);

            packet = (packet_t*) p;

            /* Check for the impossible... */
            if (packet != NULL) {
                memset(packet, 0, sizeof(*packet));
                packet->length = 0;
                packet->use = 1;
                ++active_packets;
            }
        }
    }

    if (packet == NULL) {
        /* Record the fact we dropped this one */
        ++dropped_allocations;
    }

    if (fromisr) {
        packet_unlock_from_isr();
    } else {
        packet_unlock();
    }

    ESP_LOGD(TAG, "allocate_packet_help packet_mutex released");

    return packet;
}

packet_t* allocate_packet(void)
{
    return allocate_packet_help(false);
}

packet_t* allocate_packet_from_isr(void)
{
    return allocate_packet_help(true);
}

/*
 * Create a packet from a user supplied buffer of specified length.
 */
static packet_t *packet_create_help(char *buf, int length, bool fromisr)
{
    packet_t *p = allocate_packet_help(fromisr);
    memcpy(p->buffer, buf, length);
    p->length = length;
    return p;
}

packet_t *packet_create(char* buf, int length)
{
    return packet_create_help(buf, length, false);
}

packet_t *packet_create_from_isr(char* buf, int length)
{
    return packet_create_help(buf, length, true);
}

/*
 * Decrement packet use count and if 0, free it.
 */
bool packet_release(packet_t *p)
{
    ESP_LOGD(TAG, "packet_release: %p use %d", p, p->use);

    bool ok = false;
    
    /* We don't need to lock on mutex - just make sure it's there.  The queue
     * function is atomic.
     */
    if (free_packets_queue != NULL) {
        ok = true;

        if (p != NULL && --(p->use) == 0) {
            ok = xQueueSend(free_packets_queue, (void*) &p, 0) == pdPASS;
        }
    }

    return ok;
}

/*
 * Releasing a packet from an ISR requires tossing it on a queue
 * so the program level can do the release.
 *
 * No mutex required.
 */
bool packet_release_from_isr(packet_t* p)
{
    /* Put on freeing queue */
    return xQueueSendFromISR(packet_freeing_queue, (void*) &p, NULL) == pdTRUE;
}


static int packets_available_help(bool fromisr)
{
    int available = 0;

    if (free_packets_queue != NULL) {
        if (fromisr) {
            available = uxQueueMessagesWaitingFromISR(free_packets_queue);
        } else {
            available = uxQueueMessagesWaiting(free_packets_queue);
        }
    }

    return available;
}

int packets_available(void)
{
    return packets_available_help(false);
}

int packets_available_from_isr(void)
{
    return packets_available_help(true);
}

/*
 * Get an unsigned integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_uint_field(packet_t *p, int from, int length)
{
    unsigned int value = 0;
    if (validate_field(p, from, length)) {
        for (int index = 0; index < length; ++index) {
            value = (value << 8) | (p->buffer[from + index] & 0xFF);
        }
    } else {
        /* Bad index */
        value = -1;
    }
    return value;
}

/*
 * Get an integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_int_field(packet_t *p, int from, int length)
{
    int value = get_uint_field(p, from, length);

    if (value >= 0 && ((p->buffer[from] & 0x80) != 0)) {
        /* Valid value, so field is good and sign bit set - extend it */
        value -= (1 << (length * 8));
    }

    return value;
}

/*
 * Set an integer field from a integer value.
 */
bool set_int_field(packet_t *p, int from, int length, int value)
{
    bool ok = true;
    if (validate_field(p, from, length)) {
        for (int index = length - 1; index >= 0; --index) {
            p->buffer[from + index] = value & 0xFF;
            value >>= 8;
        }
    } else {
        ok = false;
    }
    return ok;
}

/*
 * Get bytes from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_bytes_field(packet_t *p, int from, int length)
{
    char* value = NULL;

    if (length < 0) {
        length = p->length - from;
    }
    if (validate_field(p, from, length)) {
        value = (char*) malloc(length + 1);
        memcpy(value, p->buffer + from, length);
        value[length] = '\0';
    }

    return value;
}

/*
 * Get NUL terminated string from a field.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_str_field(packet_t *p, int from, int length)
{
    char* value = NULL;

    if (validate_field(p, from, length)) {
        /* Determine length of field to copy */
        length = strnlen(p->buffer + from, length); 
        value = (char*) malloc(length + 1);
        strncpy(value, p->buffer + from, length);
        value[length] = '\0';
    }

    return value;
}

/*
 * Set a string field from a char* array.
 *
 * Returns number of bytes copied.  -1 if error.
 */
int set_bytes_field(packet_t *p, int from, int length, const uint8_t* value)
{
    int moved = 0;

    if (validate_field(p, from, length) && value != NULL) {
        memcpy(p->buffer + from, value, length);
        moved = length;
    } else {
        moved = -1;
    }

    return moved;
}

/*
 * Set a string field from a NUL terminated string.
 *
 * Returns number of bytes copied.  -1 if error.
 */
int set_str_field(packet_t *p, int from, int length, const char* value)
{
    int moved = 0;

    if (validate_field(p, from, length) && value != NULL) {
        memset(p->buffer + from, 0, length);
        strncpy(p->buffer + from, value, length);
        moved = strlen(value);
        if (length < moved) {
            moved = length;
        }
    } else {
        moved = -1;
    }

    return moved;
}

packet_t* packet_ref(packet_t* p)
{
    p->use++;
    return p;
}

bool packet_lock(void)
{
    return xSemaphoreTakeRecursive(packet_mutex, 0) == pdTRUE;
}

bool packet_lock_from_isr(void)
{
    return xSemaphoreTakeFromISR(packet_mutex, NULL) == pdTRUE;
}

void packet_unlock(void)
{
    xSemaphoreGiveRecursive(packet_mutex);
}

void packet_unlock_from_isr(void)
{
    xSemaphoreGiveRecursive(packet_mutex);
}

