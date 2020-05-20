/*
 * duplicate_packets.h
 *
 * Detect duplicate packets with a sequence number cache by origin address.
 */
#ifndef __duplicate_packets_h
#define __duplicate_packets_h

#include "packets.h"
#include "listops.h"

#define NUM_ORIGIN_MAP   53   /* Small prime */

typedef struct duplicate_packet_list {
    list_head_t  origin_map[NUM_ORIGIN_MAP];
} duplicate_packet_list_t;

typedef struct duplicate_sequence duplicate_sequence_t;

typedef struct duplicate_sequence {
    duplicate_sequence_t *next;
    duplicate_sequence_t *prev;
    int                  origin;
    int                  sequence;    
} duplicate_sequence_t;

bool is_duplicate_packet(duplicate_packet_list_t* duplist, const packet_t* packet);

#endif /* __duplicate_packets_h */
