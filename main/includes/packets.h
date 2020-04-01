/*
 * packets.h
 *
 * Defines LastLink packet structure.
 */
#ifndef __packets_h_included
#define __packets_h_included
#include <stdbool.h>

#define MAX_PACKET_LEN       CONFIG_LASTLINK_MAX_PACKET_LENGTH

#define UNKNOWN_RADIO        -1

typedef struct packet {
    int              radio_num;                  /* Radio source of packet (and placeholder for destination when sending) */
    int              rssi;                       /* Received signal strength */
    int              snr;                        /* In .1 db */
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

/*
 * Allocate a packet.  Packet is filled with zeroes before delivery.
 */
packet_t *allocate_packet(void);

/*
 * Create a packet from a user supplied buffer of specified length.
 */
packet_t *create_packet(uint8_t *buf, size_t length);

/*
 * Duplicate a packet
 */
packet_t *duplicate_packet(packet_t *packet);

/*
 * Return number of available packets.
 */
int available_packets(void);

/*
 * Update usecount on packet.
 */
#ifdef NOTUSED
packet_t *ref_packet(packet_t *p);
#else
inline static packet_t* ref_packet(packet_t* packet)
{
    if (packet != NULL) {
        packet->ref++;
    }
    return packet;
}
#endif

/*
 * Decrement packet use count and if 0, free it.
 */
bool release_packet(packet_t *p);

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

#endif /* __packets_h_included */
