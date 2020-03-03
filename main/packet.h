/*
 * packet.h
 *
 * Defines LastLink packet structure.
 */
#ifndef __packet_h_included
#define __packet_h_included
#include <stdbool.h>
#include "sdkconfig.h"

//#define CONFIG_LASTLINK_INITIAL_FREE_PACKETS     20

#define MAX_PACKET_LENGTH    		255
#define NUM_INITIAL_FREE_PACKETS	CONFIG_LASTLINK_INITIAL_FREE_PACKETS

typedef struct packet {
	union {
		struct {
			int	use;        /* Use counter - when released is decremented; when 0 item is freed */
			int     length;     /* Number of bytes used buffer */
		};
		struct packet * next;       /* Used when on free queue */
	};
	char	buffer[MAX_PACKET_LENGTH];  /* Buffer */
} packet;

/*
 * Allocate a packet of specified length.  Packet is filled with zeroes.
 */
packet *allocate_packet(int length);

/*
 * Create a packet from a user supplied buffer of specified length.
 */
packet *create_packet(char *buf, int length);

/*
 * Update usecount on packet.
 */
static inline bool packet_ref(packet *p) 
{
	if (p != NULL) {
		p->use++;
	}
	return p != NULL;
}

/*
 * Decrement packet use count and if 0, free it.
 */
void release_packet(packet *p);

/*
 * Get an integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_int_field(packet *p, int from, int length);

/*
 * Set an integer field from a integer value.
 */
bool set_int_field(packet *p, int from, int length, int value);

/*
 * Get a string from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_bytes_field(packet *p, int from, int length);

/*
 * Set a string field from a char* array.
 * If length is < 0, value is copied until end of string or buffer, whichever is first.
 */
bool set_bytes_field(packet *p, int from, int length, const char *value);

/*
 * Get a string from a field.  If length < 0, field is to end of buffer.
 * Returns a freshly allocated char* array which must be freed by the caller.
 */
const char* get_str_field(packet *p, int from, int length);

/*
 * Set a string field from a char* array.
 * If length is < 0, value is copied until end of string or buffer, whichever is first.
 */
bool set_str_field(packet *p, int from, int length, const char *value);

/*
 * Duplicate a packet.
 */
static inline packet *dup_packet(packet *p)
{
	return p != NULL ? create_packet(p->buffer, p->length) : NULL;
}


#endif /* __packet_h_included */
