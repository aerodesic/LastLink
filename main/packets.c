/*
 * packets.c
 *
 * Defines LastLink packet structure.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "os_freertos.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "linklayer.h" /*DEBUG*/
#include "packets.h"

#if CONFIG_LASTLINK_TABLE_COMMANDS
#include "commands.h"
#endif

/* Round up packet len so each is on a 32 byte boundary and multiples of 32 bytes */
#define ALLOCATED_PACKET_LEN   ((MAX_PHYSICAL_PACKET_LEN + 31) & 0xFFE0)

/* Where freed packets go - where we look first */
static packet_t          *packet_table;
static uint8_t           *dma_buffers;
static int               packet_table_len;
static os_queue_t        free_packets_queue;
static os_mutex_t        packet_mutex;
static int               packets_in_use;
static int               dropped_allocations;

static bool validate_field(const packet_t *p, size_t from, size_t length);

#define TAG     "packets"

/*
 * Validate the from and length fields of packet.
 */
static bool validate_field(const packet_t *p, size_t from, size_t length)
{
    bool rc = p != NULL && from < p->length && from + length <= p->length;

// ESP_LOGI(TAG, "%s: length %u from %u for %u bytes: %s", __func__, p->length, from, length, rc ? "OK": "FAIL");

    return rc;
}

/*
 * Allocate a packet of specified length.  Packet is filled with zeroes.
 */
packet_t *allocate_packet_plain(void)
{
    // ESP_LOGD(TAG, "allocate_packet");

    packet_t *packet = NULL;

    packet_lock();

    /* Make sure free packet queue is present */
    if (free_packets_queue != NULL) {

        void* p;

        if  (os_get_queue_with_timeout(free_packets_queue, &p, 0)) {

            // ESP_LOGD(TAG, "allocate_packet packet from queue is %p", p);

            packet = (packet_t*) p;

            /* Check for the impossible... */
            if (packet != NULL) {

                if (packet->ref != 0) { 
                    ESP_LOGE(TAG, "%s: ref not zero: %d", __func__, packet->ref);
                }

                memset(packet->buffer, 0, MAX_PHYSICAL_PACKET_LEN);
                packet->length = 0;
                packet->transmitted = false;
                packet->delay = 0;
                packet->ref = 1;
                packet->radio_num = UNKNOWN_RADIO;
                packet->rssi = 0;
                packet->crc_ok = false;

                ++packets_in_use;
            }
        }
    }

    if (packet == NULL) {
        /* Record the fact we dropped this one */
        ++dropped_allocations;
    }

    packet_unlock();

    return packet;
}

/*
 * Create a packet from a user supplied buffer of specified length.
 */
packet_t *create_packet_plain(uint8_t* buf, size_t length)
{
    if (length > MAX_PACKET_LEN) {
       length = MAX_PACKET_LEN;
    }

    packet_t *p = allocate_packet();

    if (p != NULL) {
        memcpy(p->buffer, buf, length);
        p->length = length;
       p->radio_num = UNKNOWN_RADIO;
    }

    return p;
}

packet_t *duplicate_packet_plain(packet_t *packet)
{
    packet_t *new_packet = create_packet(packet->buffer, packet->length);

    if (new_packet != NULL) {
        new_packet->transmitted = packet->transmitted;
        new_packet->radio_num   = packet->radio_num;
        new_packet->rssi        = packet->rssi;
        new_packet->snr         = packet->snr;
        new_packet->crc_ok      = packet->crc_ok;
    }

    return new_packet;
}

bool release_packet_plain(packet_t *packet)
{
    bool ok = false;

    if (packet != NULL) {

        if (free_packets_queue != NULL) {
            packet_lock();

            if (--(packet->ref) == 0) {
                packet->routed_callback = NULL;
                packet->routed_callback_data = NULL;
                ok = os_put_queue_with_timeout(free_packets_queue, packet, 0);
                packets_in_use--;
            } else {
                ok = true;
            }

            packet_unlock();
        }
    }

    return ok;
}

packet_t *ref_packet_plain(packet_t *packet)
{
    if (packet != NULL) {
        packet_lock();
        packet->ref++;
        packet_unlock();
    }
    return packet;
}
#if CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION
packet_t *touch_packet_debug(const char *filename, int lineno, packet_t *packet)
{
    if (packet != NULL) {
        packet->last_referenced_filename = filename;
        packet->last_referenced_lineno = lineno;
    }

    return packet;
}

packet_t *allocate_packet_debug(const char *filename, int lineno)
{
    return touch_packet_debug(filename, lineno, allocate_packet_plain());
}

packet_t *create_packet_debug(const char* filename, int lineno, uint8_t* buf, size_t length)
{
    return touch_packet_debug(filename, lineno, create_packet_plain(buf, length));
}

packet_t *duplicate_packet_debug(const char* filename, int lineno, packet_t *packet)
{
    return touch_packet_debug(filename, lineno, duplicate_packet_plain(packet));
}

packet_t *ref_packet_debug(const char* filename, int lineno, packet_t *packet)
{
    return touch_packet_debug(filename, lineno, ref_packet_plain(packet));
}

/*
 * Decrement packet ref count and if 0, free it.
 */
bool release_packet_debug(const char *filename, int lineno, packet_t *packet)
{
    bool ok = false;

    if (packet != NULL) {
        if (packet->ref != 0) {
            // For now, don't 'touch' a packet when released.  We want to find the last actual use (or ref)
            ok = release_packet_plain(packet);
            // ok = release_packet_plain(touch_packet_debug(filename, lineno, packet));
        } else {
            ESP_LOGE(TAG, "********************************************************");
            ESP_LOGE(TAG, "%s: packet_release called by %s:%d", __func__, filename, lineno);
            ESP_LOGE(TAG, "%s: last accessed by %s:%d", __func__, packet->last_referenced_filename, packet->last_referenced_lineno);
            linklayer_print_packet("Already Released", packet);
        }
    } else {
        ESP_LOGE(TAG, "********************************************************");
        ESP_LOGE(TAG, "%s: release_packet called with NULL by %s:%d", __func__, filename, lineno);
    }

    return ok;
}
#endif /* CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION */

int available_packets(void)
{
    int available = 0;

    if (free_packets_queue != NULL) {
        available = os_items_in_queue(free_packets_queue);
    }

    return available;
}

/*
 * Get an unsigned integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_uint_field(const packet_t *p, size_t from, size_t length)
{
    unsigned int value = 0;
    if (validate_field(p, from, length)) {
        for (int index = 0; index < length; ++index) {
            value = (value << 8) | *(p->buffer + from + index);
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
int get_int_field(const packet_t *p, size_t from, size_t length)
{
    int value = get_uint_field(p, from, length);

    if (value >= 0 && ((*(p->buffer + from) & 0x80) != 0)) {
        /* Valid value, so field is good and sign bit set - extend it */
        value -= (1 << (length * 8));
    }

    return value;
}

/*
 * Set an integer field from a integer value.
 */
bool set_int_field(packet_t *p, size_t from, size_t length, int value)
{
    bool ok = true;
    if (validate_field(p, from, length)) {
        for (int index = length - 1; index >= 0; --index) {
//ESP_LOGI(TAG, "%s: storing %02x at %p + %d + %d", __func__, value & 0xFF, p->buffer, from, index);
            *(p->buffer + from + index) = value & 0xFF;
            value >>= 8;
        }
    } else {
        ok = false;
    }
    return ok;
}

/*
 * Get bytes from a field.  If length == 0, field is to end of buffer.
 * Returns a freshly allocated uint8_t* array which must be freed by the caller.
 */
const uint8_t* get_bytes_field(const packet_t *p, size_t from, size_t length)
{
    uint8_t* value = NULL;

    if (length == 0) {
        length = p->length - from;
    }
    if (validate_field(p, from, length)) {
        value = (uint8_t*) malloc(length + 1);
        memcpy(value, p->buffer + from, length);
        value[length] = '\0';
    }

    return (const uint8_t*) value;
}

/*
 * Get NUL terminated string from a field.
 * Returns a freshly allocated uint8_t* array which must be freed by the caller.
 */
const char* get_str_field(const packet_t *p, size_t from, size_t length)
{
    char* value = NULL;

    if (validate_field(p, from, length)) {
        value = strndup((char*) p->buffer + from, length);
    }

    return (const char*) value;
}

/*
 * Set a string field from a char* array.
 *
 * Returns number of bytes copied.  -1 if error.
 */
int set_bytes_field(packet_t *p, size_t from, size_t length, const uint8_t* value)
{
    size_t moved = 0;

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
int set_str_field(packet_t *p, size_t from, size_t length, const char* value)
{
    int moved = 0;

    if (validate_field(p, from, length) && value != NULL) {
        memset(p->buffer + from, 0, length);
        strncpy((char*) p->buffer + from, value, length);
        moved = strlen(value);
        if (length < moved) {
            moved = length;
        }
    } else {
        moved = -1;
    }

    return moved;
}

bool packet_lock(void)
{
    return os_acquire_recursive_mutex(packet_mutex);
    // This was failing without being noticed.  Lots of bad stuff could happen...
    //return xSemaphoreTakeRecursive(packet_mutex, 0) == pdTRUE;
}

void packet_unlock(void)
{
    os_release_mutex(packet_mutex);
}

/*
 * Called to report success or failure on routing to this packets destination.
 *
 * Returns true or false.
 */
bool packet_tell_routed_callback(packet_t *packet, bool success)
{
    bool ok = true;

    if (packet->routed_callback != NULL) {
        /* Tell supplier a route exists or if failed */
        ok = packet->routed_callback(success ? ref_packet(packet) : NULL, packet->routed_callback_data);
        packet->routed_callback = NULL;
    }

    return ok;
}

void packet_set_routed_callback(packet_t *packet, bool (*callback)(packet_t *packet, void* data), void* data)
{
    if (packet->routed_callback != NULL) {
        ESP_LOGE(TAG, "%s: Already has a routed callback; overriding", __func__);
    }
    packet->routed_callback = callback;
    packet->routed_callback_data = data;
}

#if CONFIG_LASTLINK_TABLE_COMMANDS
typedef struct {
    void*         address;
    void*         buffer;
    int           ref;
    int           radio_num;
    int           length;
    bool          routed_callback;
    void*         routed_callback_data;
    int           routeto;
    int           origin;
    int           dest;
    int           sender;
    int           protocol;
#if CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION
    const char    *last_referenced_filename;
    unsigned int  last_referenced_lineno;
#endif
} packet_info_table_t;

int print_packet_table(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Show packet table");
    } else {
        int num_packets = 0;

        packet_info_table_t table[CONFIG_LASTLINK_NUM_PACKETS];

        if (packet_lock()) {
            /* Make a static copy of the table */
            while (num_packets < ELEMENTS_OF(table)) {
                table[num_packets].address                  = &packet_table[num_packets];
                table[num_packets].buffer                   = packet_table[num_packets].buffer;
                table[num_packets].ref                      = packet_table[num_packets].ref;
                table[num_packets].radio_num                = packet_table[num_packets].radio_num;
                table[num_packets].length                   = packet_table[num_packets].length;
                table[num_packets].routed_callback          = packet_table[num_packets].routed_callback != NULL;
                table[num_packets].routed_callback_data     = packet_table[num_packets].routed_callback_data;
                table[num_packets].routeto                  = get_uint_field(&packet_table[num_packets], HEADER_ROUTETO_ADDRESS, ADDRESS_LEN);
                table[num_packets].origin                   = get_uint_field(&packet_table[num_packets], HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
                table[num_packets].dest                     = get_uint_field(&packet_table[num_packets], HEADER_DEST_ADDRESS, ADDRESS_LEN);
                table[num_packets].sender                   = get_uint_field(&packet_table[num_packets], HEADER_SENDER_ADDRESS, ADDRESS_LEN);
                table[num_packets].protocol                 = get_uint_field(&packet_table[num_packets], HEADER_PROTOCOL, PROTOCOL_LEN);

                #if CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION
                table[num_packets].last_referenced_filename = packet_table[num_packets].last_referenced_filename;
                table[num_packets].last_referenced_lineno   = packet_table[num_packets].last_referenced_lineno;
                #endif

                ++num_packets;
            }

            packet_unlock();
        }

        bool all = argc > 1 && strcmp(argv[1], "-a") == 0;

        if (num_packets != 0) {
            bool header_printed = false;

            for (int index = 0; index < num_packets; ++index) {
                if (all || table[index].ref != 0) {
                    if (! header_printed) {
                        #if CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION
                        printf("Packet      Buffer      Ref  Radio  Length  RouteTo  Origin  Dest  Sender  Proto  Callback  CB Data     Last Accessed\n");
                        #else
                        printf("Packet      Buffer      Ref  Radio  Length  RouteTo  Origin  Dest  Sender  Proto  Callback  CB Data\n");
                        #endif
                        header_printed = true;
                    }
                    #if CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION
                    printf("%-10p  %-10p  %-3d  %-5d  %-6d  %-7x  %-6x  %-4x  %-6x  %-5d  %-8s  %-10p  %s:%d\n",
                            table[index].address,
                            table[index].buffer,
                            table[index].ref,
                            table[index].radio_num,
                            table[index].length,
                            table[index].routeto & 0xFFFF,
                            table[index].origin & 0xFFFF,
                            table[index].dest & 0xFFFF,
                            table[index].sender & 0xFFFF,
                            table[index].protocol,
                            table[index].routed_callback ? "YES" : "NO",
                            table[index].routed_callback_data,
                            table[index].last_referenced_filename,
                            table[index].last_referenced_lineno);
                    #else
                    printf("%-10p  %-10p  %-3d  %-5d  %-6d  %-7x  %-6x  %-4x  %-6x  %-5d  %-8s  %p\n",
                            table[index].address,
                            table[index].buffer,
                            table[index].ref,
                            table[index].radio_num,
                            table[index].length,
                            table[index].routeto & 0xFFFF,
                            table[index].origin & 0xFFFF,
                            table[index].dest & 0xFFFF,
                            table[index].sender & 0xFFFF,
                            table[index].protocol,
                            table[index].routed_callback ? "YES" : "NO",
                            table[index].routed_callback_data);
                    #endif
                }
            }
        }

        printf("\nPacket lock:    %s\n", os_get_mutex_count(packet_mutex) ? "UNLOCKED" : "LOCKED");
        printf("\nFree packets:   %d\n", available_packets());
        printf("\nPackets in use: %d\n", packets_in_use);
    }

    return 0;
}
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */

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

    ESP_LOGI(TAG, "init_packets: %d", num_packets);

    packet_mutex = os_create_recursive_mutex();

    if (packet_mutex != NULL) {
        // ESP_LOGI(TAG, "packet_mutex created");

        packet_lock();

        packets_in_use = num_packets;  // Until initially freed, they are 'in use'.

        /* Create the bulk table */
        packet_table = (packet_t*) malloc(sizeof(packet_t) * num_packets);
        if (packet_table != NULL) {
            /* Create the bulk DMA buffer */
            dma_buffers = (uint8_t*) os_alloc_dma_memory(ALLOCATED_PACKET_LEN * num_packets);

            if (dma_buffers != NULL) {
                /* Zero the table */
                memset(packet_table, 0, num_packets * sizeof(packet_t));

                packet_table_len = num_packets;
                free_packets_queue = os_create_queue(num_packets, sizeof(packet_t*));

                if (free_packets_queue != NULL) {
                    // ESP_LOGD(TAG, "free_packets_queue created");
    
                    ok = true;

                    int count = 0;

                    while(ok && count < num_packets) {
                        packet_t *p = &packet_table[count];
                        memset(p, 0, sizeof(packet_t));
                        p->ref = 1;
                        p->buffer = dma_buffers;
                        release_packet(p);

                        dma_buffers += ALLOCATED_PACKET_LEN;
                        ++count;
                    }      
                } else {
                    ESP_LOGE(TAG, "%s: Unable to allocate packet queue", __func__);
                }
            } else {
                ESP_LOGE(TAG, "%s: Unable to allocate packet dma pool", __func__);
            }
        } else {
            ESP_LOGE(TAG, "%s: Unable to allocate packet table", __func__);
        }

        packet_unlock();
    }

    if (!ok) {
        /* Undo failed initialization */
        deinit_packets();
    } else {
#if CONFIG_LASTLINK_TABLE_COMMANDS
        add_command("p", print_packet_table);
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */
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
    int packets_left = -1;


    if (packet_mutex != NULL) {

        packet_lock();

        if (free_packets_queue != NULL) {

            if (packets_in_use != 0) {
                packets_left = packets_in_use;
            } else {
                /* Release free packets */
                packet_t *packet;
                while (os_get_queue_with_timeout(free_packets_queue, (os_queue_item_t*) &packet, 0)) {
                    packet->buffer = NULL;
                }

                os_delete_queue(free_packets_queue);
                free_packets_queue = NULL;
            }
        }

        packet_unlock();

        if (packets_left == 0) {
            os_delete_mutex(packet_mutex);
            packet_mutex = NULL;

            /* Free the bulk store */
            free((void*) packet_table);
            free((void*) dma_buffers);
            packet_table = NULL;
            dma_buffers = NULL;
            packet_table_len = 0;

#if CONFIG_LASTLINK_TABLE_COMMANDS
            remove_command("p");
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */
        }
    }

    return packets_left;
}

