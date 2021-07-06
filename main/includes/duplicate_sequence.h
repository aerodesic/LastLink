/*
 * duplicate_sequence.h
 *
 * Detect duplicate sequence number cache by address.
 */
#ifndef __duplicate_sequence_h
#define __duplicate_sequence_h

#include "listops.h"

#define NUM_ORIGIN_MAP   53   /* Small prime */

typedef struct duplicate_sequence_list {
    list_head_t  address_map[NUM_ORIGIN_MAP];
} duplicate_sequence_list_t;

typedef struct duplicate_sequence duplicate_sequence_t;

typedef struct duplicate_sequence {
    duplicate_sequence_t *next;
    duplicate_sequence_t *prev;
    int                  address;
    int                  sequence;
    int                  count;
} duplicate_sequence_t;

void reset_duplicate(duplicate_sequence_list_t *duplist, int address);
bool is_duplicate(duplicate_sequence_list_t* duplist, int address, int sequence);

#endif /* __duplicate_sequence_h */
