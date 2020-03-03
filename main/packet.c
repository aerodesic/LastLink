/*
 * packet.c
 *
 * Defines LastLink packet structure.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "packet.h"

/* Where freed packets go - where we look first */
static packet* free_packets;

void init_packets(void)
{
	for (int count = 0; count < NUM_INITIAL_FREE_PACKETS; ++count) {
		packet *p = (packet *) malloc(sizeof(packet));
		p->use = 1;
		release_packet(p);
	}
}

/*
 * Validate the from and length fields of packet.
 */
static bool validate_field(packet *p, int from, int length)
{
	return p != NULL && from < p->length && from + length <= p->length;
}

/*
 * Allocate a packet of specified length.  Packet is filled with zeroes.
 */
packet *allocate_packet(int length)
{
	packet *p;

	if (free_packets != NULL) {
		p = free_packets;
		free_packets = p->next;
	} else {
		p = (packet *) malloc(sizeof(packet) + length - 1);
	}
	if (p != NULL) {
		memset(p, 0, length);
		p->length = length;
		p->use = 1;
	}

	return p;
}

/*
 * Create a packet from a user supplied buffer of specified length.
 */
packet *create_packet(char *buf, int length)
{
	packet *p = allocate_packet(length);
	memcpy(p->buffer, buf, length);
	return p;
}

/*
 * Decrement packet use count and if 0, free it.
 */
void release_packet(packet *p)
{
	if (p != NULL && --(p->use) == 0) {
		p->next = free_packets;
		free_packets = p;
	}
}

/*
 * Get an integer value from a field.  Bytes are packed to an integer in big endian format.
 */
int get_int_field(packet *p, int from, int length)
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
bool set_int_field(packet *p, int from, int length, int value)
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
const char* get_bytes_field(packet *p, int from, int length)
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
bool set_bytes_field(packet *p, int from, int length, const char *value)
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

bool set_str_field(packet *p, int from, int length, const char* value)
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

