/*
 * packet_window.h
 *
 * Implements the packet window mechanism for the streaming sockets.
 *
 *
 */
#ifndef __packet_window_h_included
#define __packet_window_h_included #undef PACKET_WINDOW_LOCKING_DEBUG

#include "packets.h"
#include "os_specific.h"

typedef struct packet_slot {
    int                sequence;           /* Sequence of packet (for verification and easy access) */
    bool               inuse;              /* true if the slot represents a (possibly freed) packet */
    packet_t           *packet;            /* The packet if it hasn't been processed */
} packet_slot_t;

typedef struct packet_window {
    os_mutex_t          lock;              /* Exclusivity lock for thread safety */
#ifdef PACKET_WINDOW_LOCKING_DEBUG
    const char         *last_lock_file;
    int                 last_lock_line;
    int                 lock_count;
#endif /* PACKET_WINDOW_LOCKING_DEBUG */
    os_semaphore_t      available;         /* Number of sequentially available packets from queue[0] */
    os_semaphore_t      room;              /* 'released' when a slot may have become available */
    bool                shutdown;          /* Shuts down window when queue is empty */
    int                 sequence;          /* Sequence number of first packet in queue */
    int                 length;            /* Number of slots to use in queue */
    int                 size;              /* Number physical slots available in queue */
    int                 num_in_queue;      /* Slots in use */
    int                 next_in_queue;     /* Next contiguous slot in queue */
    bool                user_blocked;      /* True when user is blocked on waiting for window */
    packet_slot_t       queue[1];          /* Queue of packets with sequence numbers */
} packet_window_t;

#ifdef PACKET_WINDOW_LOCKING_DEBUG
bool packet_window_lock_debug(packet_window_t *window, const char *file, int line);
bool packet_window_unlock_debug(packet_window_t *window, const char *file, int line);
#define packet_window_lock(window)        packet_window_lock_debug(window, __FILE__, __LINE__)
#define packet_window_unlock(window)      packet_window_unlock_debug(window, __FILE__, __LINE__)
#else
inline bool packet_window_lock(packet_window_t *window)
{
    return os_acquire_recursive_mutex(window->lock);
}
inline bool packet_window_unlock(packet_window_t *window)
{
    return os_release_recursive_mutex(window->lock);
}
#endif /* PACKET_WINDOW_LOCKING_DEBUG */

void packet_window_init(void);
void packet_window_deinit(void);

/* Create a new packet window with <slots> entries in the queue */
packet_window_t *packet_window_create(int slots);

/* Free all subordinate information and free a packet window */
void packet_window_release(packet_window_t *window);

/* Add a packet into the window at it's sequence number position. */
int packet_window_add_random_packet(packet_window_t *window, packet_t *packet);

/* Remove the next packet from the front of the window. */
bool packet_window_remove_sequential_packet(packet_window_t *window, packet_t **packet,  int timeout);

/* Add a sequential packet to the queue, giving it a sequence number in the process */
bool packet_window_add_sequential_packet(packet_window_t *window, packet_t *packet, int timeout);

/* Get a list of all packets in the window.  Caller needs to release when done */
int packet_window_get_all_packets(packet_window_t *window, packet_t *packets[], int num_packets);

/* Get the list of accepted and processed packets in the queue to provide acknowledgement */
bool packet_window_get_processed_packets(packet_window_t *window, int *sequence, uint32_t *packet_mask);

/* Release and trim queue for all packets processed. */
int packet_window_release_processed_packets(packet_window_t *window, int sequence, uint32_t packet_mask);

void packet_window_shutdown_window(packet_window_t *window);

bool packet_window_is_shutdown(packet_window_t *window);

int packet_window_packet_count(packet_window_t *window);

bool packet_window_user_is_blocked(packet_window_t *window);

void packet_window_trigger_available(packet_window_t *window);

void packet_window_trigger_room(packet_window_t *window);

#endif /* __packet_window_h_include */
