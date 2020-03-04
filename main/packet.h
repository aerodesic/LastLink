/*
 * packet.h
 *
 * Defines LastLink packet structure.
 */
#ifndef __packet_h_included
#define __packet_h_included
#include <stdbool.h>

#define MAX_PACKET_LENGTH       255

typedef struct packet {
    int   use;                        /* Use counter - when released is decremented; when 0 item is freed */
    int   length;                     /* Number of bytes used buffer */
    char  buffer[MAX_PACKET_LENGTH];  /* Buffer */
} packet_t;

/*
 * Initialize packet system.
 */
bool init_packets(int num_packets);
int deinit_packets(void);

/*
 * Allocate a packet of specified length.  Packet is filled with zeroes.
 */
packet_t *allocate_packet(void);
packet_t *allocate_packet_from_isr(void);

/*
 * Create a packet from a user supplied buffer of specified length.
 */
packet_t *create_packet(char *buf, int length);
packet_t *create_packet_from_isr(char *buf, int length);

/*
 * Return number of available packets.
 */
int packets_available(void);
int packets_available_from_isr(void);

/*
 * Update usecount on packet.
 */
int packet_ref(packet_t *p) ;

/*
 * Decrement packet use count and if 0, free it.
 */
bool release_packet(packet_t *p);
bool release_packet_from_isr(packet_t *p);

/*
 * Get an integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_int_field(packet_t *p, int from, int length);

/*
 * Set an integer field from a integer value.
 */
bool set_int_field(packet_t *p, int from, int length, int value);

/*
 * Get a string from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_bytes_field(packet_t *p, int from, int length);

/*
 * Set a string field from a char* array.
 * If length is < 0, value is copied until end of string or buffer, whichever is first.
 */
bool set_bytes_field(packet_t *p, int from, int length, const char *value);

/*
 * Get a string from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_str_field(packet_t *p, int from, int length);

/*
 * Set a string field from a char* array.
 * If length is < 0, value is copied until end of string or buffer, whichever is first.
 */
bool set_str_field(packet_t *p, int from, int length, const char *value);

/*
 * Duplicate a packet.
 */
static inline packet_t *dup_packet(packet_t *p)
{
    return p != NULL ? create_packet(p->buffer, p->length) : NULL;
}


#endif /* __packet_h_included */
