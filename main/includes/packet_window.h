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
    int                sequence;           /* Sequence of packet (for verification and easy access) */
    bool               inuse;              /* true if the slot represents a (possibly freed) packet */
    packet_t           *packet;            /* The packet if it hasn't been processed */
} packet_slot_t;

typedef struct packet_window {
    os_mutex_t          lock;              /* Exclusivity lock for thread safety */
    os_semaphore_t      available;         /* Number of sequentially available packets from queue[0] */
    os_semaphore_t      room;              /* 'released' when a slot may have become available */
    bool                shutdown;          /* Shuts down input stream when queue is empty */
    int                 sequence;          /* Sequence number of first packet in queue */
    int                 length;            /* Number of slots in queue */
    int                 num_in_queue;      /* Slots in use */
    bool                reader_busy;       /* True when reader is blocked on waiting for packet */
    packet_slot_t       queue[1];          /* Queue of packets with sequence numbers */
} packet_window_t;

void packet_window_init(void);
void packet_window_deinit(void);

/* Create a new packet window with <slots> entries in the queue */
packet_window_t *packet_window_create(int slots);

/* Free all subordinate information and free a packet window */
void packet_window_release(packet_window_t *window, bool (*release_packet)(packet_t*));

/* Put a packet into the window at the sequence number position. */
bool packet_window_add_packet(packet_window_t *window, packet_t *packet, int sequence, int timeout);

/* Remove the next packet from the front of the window. */
bool packet_window_remove_packet(packet_window_t *window, packet_t **packet,  int timeout);

/* Get a list of all packets in the window.  Caller needs to release when done */
int packet_window_get_all_packets(packet_window_t *window, packet_t *packets[], size_t num_packets);

/* Get the list of accepted and processed packets in the queue to provide acknowledgement */
void packet_window_get_processed_packets(packet_window_t *window, int *sequence, uint32_t *packet_mask);

/* Release and trim queue for all packets processed. */
int packet_window_release_processed_packets(packet_window_t *window, int sequence, uint32_t packet_mask, bool (*release_packet)(packet_t*));

void packet_window_shutdown_window(packet_window_t *window);

bool packet_window_is_window_shutdown(packet_window_t *window);

int packet_window_packet_count(packet_window_t *window);

int packet_window_next_sequence(packet_window_t *window);

bool packet_window_is_reader_busy(packet_window_t *window);


#endif /* __packet_window_h_include */