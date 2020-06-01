/*
 * packet_window.h
 *
 * Implements the packet window mechanism for the streaming sockets.
 *
 *
 */
#ifndef __packet_window_h_included
#define __packet_window_h_included

#include "os_specific.h"

typedef struct packet packet_t;    /* Foward reference - internals are not  needed */

typedef struct packet_slot {
    int                sequence;
    packet_t           *packet;
} packet_slot_t;

typedef struct packet_window {
    os_mutex_t          lock;              /* Exclusivity lock for thread safety */
    os_semaphore_t      available;         /* Number of sequentially available packets from queue[0] */
    os_semaphore_t      room;              /* 'released' when a slot may have become available */
    bool                shutdown;          /* Shuts down input stream when queue is empty */
    int                 sequence;          /* Sequence number of first packet in queue */
    int                 length;            /* Number of slots in queue */
    int                 next_in;           /* Number of sequential packets from beginning */
    int                 num_in_queue;      /* Slots in use */
    packet_slot_t       queue[1];          /* Queue of packets with sequence numbers */
} packet_window_t;

void init_packet_window(void);
void deinit_packet_window(void);

/* Create a new packet window with <slots> entries in the queue */
packet_window_t *create_packet_window(int slots);

/* Free all subordinate information and free a packet window */
void release_packet_window(packet_window_t *window, bool (*release_packet)(packet_t*));

/* Put a packet into the window at the sequence number position. */
bool insert_packet_into_window(packet_window_t *window, packet_t *packet, int sequence, int timeout);

/* Remove the next packet from the front of the window. */
bool remove_packet_from_window(packet_window_t *window, packet_t **packet,  int timeout);

/* Get a list of all packets in the window.  Caller needs to release when done */
int get_packets_in_window(packet_window_t *window, packet_t *packets[], size_t num_packets);

/* Get the list of accepted and processed packets in the queue to provide acknowledgement */
void get_accepted_packet_sequence_numbers(packet_window_t *window, int *sequence, uint32_t *packet_mask);

/* Release and trim queue for all packets processed. */
int release_accepted_packets_in_window(packet_window_t *window, int sequence, uint32_t packet_mask, bool (*release_packet)(packet_t*));

void shutdown_window(packet_window_t *window);

bool is_shutdown(packet_window_t *window);

int packets_in_window(packet_window_t *window);

#endif /* __packet_window_h_include */
