/*
 * packets.h
 *
 * Defines LastLink packet structure.
 */
#ifndef __packets_h_included
#define __packets_h_included
#include <stdbool.h>
#include <unistd.h>

#if CONFIG_LASTLINK_CRC16_PACKETS
  /* If CRC enabled, two bytes are taken to support crc transfer */
  #define MAX_PACKET_LEN       (CONFIG_LASTLINK_MAX_PACKET_LENGTH - 2)
#else
  #define MAX_PACKET_LEN       CONFIG_LASTLINK_MAX_PACKET_LENGTH
#endif
#define MAX_PHYSICAL_PACKET_LEN  CONFIG_LASTLINK_MAX_PACKET_LENGTH

#define UNKNOWN_RADIO        -1

typedef struct packet packet_t;  // Forward red

typedef struct packet {
#if CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION
    const char       *last_referenced_filename;
    int              last_referenced_lineno;
#endif

    /* Callback when packet has been routed.  Can then resend.  If entry is NULL, resend is automatic */
    bool             (*routed_callback)(bool success, packet_t* packet, void* data);
    void             *routed_callback_data;

    /* Callback when packet has been transmitted.  Nulled automatically when complete */
    void             (*transmitted_callback)(packet_t *packet, void *data);
    void             *transmitted_callback_data;

    uint8_t          queued;                     /* Counts up one for each time transmit queue */
    bool             transmitted;                /* true if packet was transmitted (promiscuous mode status) */
    int              delay;                      /* A delay factor before transmitting packet */
    uint8_t          radio_num;                  /* Radio source of packet (and placeholder for destination when sending) */
    int8_t           rssi;                       /* Received signal strength */
    int8_t           snr;                        /* In .1 db */
    bool             crc_ok;                     /* True if good crc check */

    int              ref;                        /* Ref counter - when released is decremented; when 0 item is freed */
    int              length;                     /* Number of bytes used buffer */
    uint8_t          *buffer;                    /* Buffer pointer */
} packet_t;

/*
 * Initialize packet system.
 */
bool init_packets(int num_packets);
int deinit_packets(void);

packet_t *allocate_packet_plain(void);
packet_t *create_packet_plain(uint8_t *buf, size_t length);
packet_t *duplicate_packet_plain(packet_t *packet);
bool release_packet_plain(packet_t *packet);
packet_t* ref_packet_plain(packet_t* packet);
inline packet_t *touch_packet_plain(packet_t *packet)
{
    return packet;
}

/*
 * Return number of available packets.
 */
int available_packets(void);

#if CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION
packet_t *allocate_packet_debug(const char* filename, int lineno);
#define allocate_packet()  allocate_packet_debug(__FILE__, __LINE__)

packet_t *create_packet_debug(const char* filename, int lineno, uint8_t *buf, size_t length);
#define create_packet(buf, length)  create_packet_debug(__FILE__, __LINE__, buf, length)

packet_t *duplicate_packet_debug(const char* filename, int lineno, packet_t *packet);
#define duplicate_packet(packet) duplicate_packet_debug(__FILE__, __LINE__, packet)

bool release_packet_debug(const char* filename, int lineno, packet_t* packet);
#define release_packet(packet) release_packet_debug(__FILE__, __LINE__, packet)

packet_t* ref_packet_debug(const char* filename, int lineno, packet_t *packet);
#define ref_packet(packet)  ref_packet_debug(__FILE__, __LINE__, packet)

packet_t *touch_packet_debug(const char *filename, int lineno, packet_t *packet);
#define touch_packet(packet) touch_packet_debug(__FILE__, __LINE__, packet)

#else

#define allocate_packet()           allocate_packet_plain()
#define create_packet(buf, length)  create_packet_plain(buf, length)
#define duplicate_packet(packet)    duplicate_packet_plain(packet)
#define release_packet(packet)      release_packet_plain(packet)
#define ref_packet(packet)          ref_packet_plain(packet)
#define touch_packet(packet)        (packet)

#endif

/*
 * Get an integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_int_field(const packet_t *p, size_t from, size_t length);
int get_uint_field(const packet_t *p, size_t from, size_t length);

/*
 * Set an integer field from a integer value.
 */
bool set_int_field(packet_t *p, size_t from, size_t length, int value);
#define set_uint_field(p, from, length, value) set_int_field(p, from, length, value)

/* Or in bits */
bool set_bits_field(packet_t *p, size_t from, size_t length, unsigned int value);

/* Remove bits */
bool clear_bits_field(packet_t *p, size_t from, size_t length, unsigned int value);

/*
 * Get a string from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const uint8_t* get_bytes_field(const packet_t *p, size_t from, size_t length);

/*
 * Set a string field from a char* array.
 * If length is < 0, value is copied until end of string or buffer, whichever is first.
 */
int set_bytes_field(packet_t *p, size_t from, size_t length, const uint8_t *value);

/*
 * Get a string from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_str_field(const packet_t *p, size_t from, size_t length);

/*
 * Set a string field from a char* array.
 * If length is < 0, value is copied until end of string or buffer, whichever is first.
 */
int set_str_field(packet_t *p, size_t from, size_t length, const char *value);

#ifdef NOTUSED
/*
 * Duplicate a packet.
 */
static inline packet_t *packet_dup(packet_t *p)
{
    return p != NULL ? packet_create(p->buffer, p->length) : NULL;
}
#endif

bool packet_lock(void);
void packet_unlock(void);

typedef bool (*routed_callback_t)(bool success, packet_t *packet, void *data);
bool packet_tell_routed_callback(packet_t *packet, bool success);
packet_t *packet_set_routed_callback(packet_t *packet, routed_callback_t callback, void *data);

typedef void (*transmitted_callback_t)(packet_t *packet, void *data);
void packet_tell_transmitted_callback(packet_t *packet);
packet_t *packet_set_transmitted_callback(packet_t *packet, transmitted_callback_t callback, void *data);

#endif /* __packets_h_included */
