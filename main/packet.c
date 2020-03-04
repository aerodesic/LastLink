/*
 * packet.c
 *
 * Defines LastLink packet structure.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "packet.h"

/* Where freed packets go - where we look first */
static QueueHandle_t     free_packets;
static SemaphoreHandle_t packet_mutex;
static int               active_packets;
static int               dropped_allocations;

static bool validate_field(packet_t *p, int from, int length);

static packet_t *allocate_packet_help(bool fromisr);
static packet_t *create_packet_help(char *buf, int length, bool fromisr);
static bool release_packet_help(packet_t *p, bool fromisr);

#define TAG     "packets"

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

        xSemaphoreTakeRecursive(packet_mutex, 0);

        free_packets = xQueueCreate((UBaseType_t) num_packets, (UBaseType_t) sizeof(packet_t*));
	if (free_packets != NULL) {

	    ESP_LOGD(TAG, "free_packets created");

            ok = true;

            int count = 0;

            while(ok && count < num_packets) {
                packet_t *p = (packet_t *) malloc(sizeof(packet_t));
                if (p != NULL) {
                    p->use = 1;
                    release_packet(p);
                } else {
                    ok = false;
                }
                ++count;
            }
	}
    }

    if (!ok) {
        /* Undo failed initialization */
	deinit_packets();
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
        xSemaphoreTakeRecursive(packet_mutex, 0);

	if (free_packets != NULL) {

            if (active_packets != 0) {
                packets_left = active_packets;
            } else {
                /* Release free packets */
                packet_t* packet;
	        while (xQueueReceive(free_packets, &packet, 0) == pdPASS) {
                    free((void*) packet);
                }
            }

	    vQueueDelete(free_packets);
	    free_packets = NULL;
	}

        xSemaphoreGiveRecursive(packet_mutex);

        if (packets_left == 0) {
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

    if ((fromisr && (xSemaphoreTakeFromISR(packet_mutex, NULL) == pdPASS)) ||
	(!fromisr && (xSemaphoreTakeRecursive(packet_mutex, 0) == pdPASS))) {

        ESP_LOGD(TAG, "allocate_packet_help packet_mutex acquired");

        /* Make sure free packet queue is present */
        if (free_packets != NULL) {

	    void* p;

	    if ((fromisr && (xQueueReceiveFromISR(free_packets, &p, NULL) == pdPASS)) ||
	        (!fromisr && (xQueueReceive(free_packets, &p, 0) == pdPASS))) {

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
    }

    if (packet == NULL) {
	/* Record the fact we dropped this one */
	++dropped_allocations;
    }

    if (fromisr) {
        xSemaphoreGiveFromISR(packet_mutex, NULL);
    } else {
        xSemaphoreGiveRecursive(packet_mutex);
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
static packet_t *create_packet_help(char *buf, int length, bool fromisr)
{
    packet_t *p = allocate_packet_help(fromisr);
    memcpy(p->buffer, buf, length);
    p->length = length;
    return p;
}

packet_t *create_packet(char* buf, int length)
{
    return create_packet_help(buf, length, false);
}

packet_t *create_packet_from_isr(char* buf, int length)
{
    return create_packet_help(buf, length, true);
}

/*
 * Decrement packet use count and if 0, free it.
 */
static bool release_packet_help(packet_t *p, bool fromisr)
{
    ESP_LOGD(TAG, "release_packet: %p use %d fromisr %s", p, p->use, fromisr ? "YES" : "NO");

    bool ok = false;
    
    /* We don't need to lock on mutex - just make sure it's there.  The queue
     * function is atomic.
     */
    if (packet_mutex != NULL) {
        ok = true;

        if (p != NULL && --(p->use) == 0) {
            ok = ((fromisr && (xQueueSendFromISR(free_packets, (void*) p, NULL) == pdPASS)) ||
                  (!fromisr && (xQueueSend(free_packets, (void*) &p, 0) == pdPASS)));
        }
    }

    return ok;
}

bool release_packet_from_isr(packet_t* p)
{
    return release_packet_help(p, true);
}


bool release_packet(packet_t* p)
{
    return release_packet_help(p, false);
}

static int packets_available_help(bool fromisr)
{
    int available = 0;

    if (free_packets != NULL) {
        if (fromisr) {
            available = uxQueueMessagesWaitingFromISR(free_packets);
	} else {
            available = uxQueueMessagesWaiting(free_packets);
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
 * Get an integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_int_field(packet_t *p, int from, int length)
{
    int value = 0;
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
 * Set a string field from a char* array.
 * If len is < 0, value is copied until end of string or buffer, whichever is first.
 */
bool set_bytes_field(packet_t *p, int from, int length, const char *value)
{
    bool ok = true;

    if (length < 0) {
        length = p->length - from;
    }
    if (validate_field(p, from, length) && value != NULL) {
        memcpy(p->buffer + from, value, length);
    } else {
        ok = false;
    }

    return ok;
}

bool set_str_field(packet_t *p, int from, int length, const char* value)
{
    bool ok = true;

    if (length < 0) {
        length = p->length - from;
    }
    if (validate_field(p, from, length) && value != NULL) {
        strncpy(p->buffer + from, value, length);
    } else {
        ok = false;
    }
    return ok;
}

