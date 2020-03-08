/*
 * packets.h
 *
 * Defines LastLink packet structure.
 */
#ifndef __packets_h_included
#define __packets_h_included
#include <stdbool.h>

#define MAX_PACKET_LEN       CONFIG_LASTLINK_MAX_PACKET_LENGTH

typedef struct packet {
    int   radio;                      /* Radio source of packet */
    int   rssi;                       /* Received signal strength */
    bool  crc_ok;                     /* True if good crc check */

    int   use;                        /* Use counter - when released is decremented; when 0 item is freed */
    int   length;                     /* Number of bytes used buffer */
    char  buffer[MAX_PACKET_LEN];     /* Buffer */
} packet_t;

/*
 * Initialize packet system.
 */
bool packet_inits(int num_packets);
int packet_deinit(void);

/*
 * Allocate a packet.  Packet is filled with zeroes before delivery.
 */
packet_t *allocate_packet(void);
packet_t *allocate_packet_from_isr(void);

/*
 * Create a packet from a user supplied buffer of specified length.
 */
packet_t *create_packet(uint8_t *buf, int length);
packet_t *create_packet_from_isr(uint8_t *buf, int length);

/*
 * Return number of available packets.
 */
int available_packets(void);
int available_packets_from_isr(void);

/*
 * Update usecount on packet.
 */
packet_t* ref_packet(packet_t *p);

/*
 * Decrement packet use count and if 0, free it.
 */
bool release_packet(packet_t *p);
bool release_packet_from_isr(packet_t *p);

/*
 * Get an integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_int_field(packet_t *p, int from, int length);
int get_uint_field(packet_t *p, int from, int length);

/*
 * Set an integer field from a integer value.
 */
bool set_int_field(packet_t *p, int from, int length, int value);
#define set_uint_field(p, from, length, value) set_int_field(p, from, length, value)

/*
 * Get a string from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_bytes_field(packet_t *p, int from, int length);

/*
 * Set a string field from a char* array.
 * If length is < 0, value is copied until end of string or buffer, whichever is first.
 */
int set_bytes_field(packet_t *p, int from, int length, const uint8_t *value);

/*
 * Get a string from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_str_field(packet_t *p, int from, int length);

/*
 * Set a string field from a char* array.
 * If length is < 0, value is copied until end of string or buffer, whichever is first.
 */
int set_str_field(packet_t *p, int from, int length, const char *value);

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
bool packet_lock_from_isr(void);
void packet_unlock_from_isr(void);

#endif /* __packets_h_included */
