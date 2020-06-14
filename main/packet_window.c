/*
 * packet_window
 *
 * Implements the packet window mechanism for the streaming sockets.
 *
 *
 */

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "esp_system.h"
#include "esp_log.h"

#include "os_specific.h"

#include "linklayer.h"
#include "lsocket_internal.h"

#include "packet_window.h"
#include "packets.h"

#define TAG "packet_window"

#ifdef PACKET_WINDOW_LOCKING_DEBUG
bool packet_window_lock_debug(packet_window_t *window, const char *file, int line)
{
    /* Try without waiting */
    bool ok = os_acquire_recursive_mutex_with_timeout(window->lock, 0);
    if (!ok) {
        /* Didn't get it, so go ahead and wait and show message */
        ESP_LOGI(TAG, "%s: **********************************************************************", __func__);
        ESP_LOGI(TAG, "%s: waiting for window lock [%s:%d] last locked at %s:%d", __func__,
                           file, line, window->last_lock_file ? window->last_lock_file : "<NULL>", window->last_lock_line);
        ESP_LOGI(TAG, "%s: **********************************************************************", __func__);
        ok = os_acquire_recursive_mutex(window->lock);
        ESP_LOGI(TAG, "%s: **********************************************************************", __func__);
        ESP_LOGI(TAG, "%s: got window lock", __func__);
        ESP_LOGI(TAG, "%s: **********************************************************************", __func__);
    }

    if (ok) {
        window->last_lock_file = file;
        window->last_lock_line = line;
    }

    return ok;
}

bool packet_window_unlock_debug(packet_window_t *window, const char *file, int line)
{
    (void) file;
    (void) line;
    return os_release_recursive_mutex(window->lock);
}
#endif

/*
 * Discard the top slot in the queue.
 */
static void packet_window_discard_top_packet(packet_window_t *window)
{
    if (window != NULL) {

        assert(window->queue[0].inuse);
        /*
         * Move up list of packets to overright the top item.
         * Continue the process for all NULL but 'inuse' placeholders until we run into a
         * active packet or the end of list list.
         */
        do {
            /* Release the top packet if not NULL */
            if (window->queue[0].packet != NULL) {
                release_packet(window->queue[0].packet);
                /* One fewer element in queue */
                assert(window->num_in_queue != 0);
                window->num_in_queue--;
            }

            memcpy(window->queue, window->queue + 1, sizeof(window->queue[0]) * (window->length - 1));
            window->queue[window->length -  1].packet = NULL;
            window->queue[window->length -  1].inuse = false;
            window->queue[window->length -  1].sequence = 0;

            /* Adjust to the first sequence number in the list */
            window->sequence++;

            /* Next entry position is moved up */
            assert(window->next_in_queue != 0);
            window->next_in_queue--;

            /* Release the slot for reuse */
            os_release_counting_semaphore(window->room);

        } while (window->queue[0].inuse && window->queue[0].packet == NULL);
    }
}

packet_window_t *packet_window_create(int slots)
{
    size_t size = sizeof(packet_window_t) + sizeof(packet_slot_t) * (slots - 1);

    packet_window_t *window = (packet_window_t *) malloc(size);
    if (window != NULL) {

        memset(window, 0, size);

        window->size = slots;
        window->length = slots;
        window->lock = os_create_recursive_mutex();

        /*
         * These semaphores are primarily used for the sequential packet process involved
         * with the user-level reading (available) and writing (room) of packets.
         */

        /*
         * The 'available' semaphore tracks the number of sequentially available packets in queue
         * relative to the first slot.  If, for example, the packets arrived in the order:
         *     3, 2, 1, 0
         * the 'available' semaphore would not be triggered until packet 0, as the slots were not
         * contiquous, and at that time it would be triggered four times, thus making all four
         * packets available for sequential removal.
         *
         * 'available' is 'released' by the add_random_packet insertion process and 'acquired'
         * by the remove_sequential_packet method for user-level reading.
         */
        window->available = os_create_counting_semaphore(slots, 0);

        /*
         * The 'room' semaphore tracks the number of sequentially available slots beyond the
         * last sequentially written slot in the queue.
         *
         * It is 'released' when the queue is rolled up upon packet removal and 'acquired'
         * when the user-level add_sequential_packet is storing a new packet.
         */
        window->room = os_create_counting_semaphore(slots, slots);

        if (window->lock == NULL || window->available == NULL || window->room == NULL) {
            packet_window_release(window);
            window = NULL;
        }
    }

    return window;
}

/*
 * Release a packet window.
 *
 * Entry:
 *    window                   The window to release
 */
void packet_window_release(packet_window_t *window)
{
    if (window != NULL) {
        packet_window_lock(window);

        /* Release packets in slots */
        for (int slot = 0; slot < window->length; ++slot) {
            if (window->queue[slot].packet != NULL) {
                release_packet(window->queue[slot].packet);
            }
        }

        if (window->lock != NULL) {
            os_delete_mutex(window->lock);
        }

        if (window->available != NULL) {
            os_delete_semaphore(window->available);
        }

        if (window->room != NULL) {
            os_delete_semaphore(window->room);
        }

        packet_window_unlock(window);
    
        os_delete_mutex(window->lock);

        free((void*) window);
    }
}

void packet_window_trigger_available(packet_window_t *window)
{
    os_release_counting_semaphore(window->available);
}

void packet_window_trigger_room(packet_window_t *window)
{
    os_release_counting_semaphore(window->room);
}


/*
 * packet_window_add_sequential_packet
 *
 * Add the packet to the next sequential slot in the queue.
 *
 * The sequence number of the packet will be assigned as it's placed in the queue.
 *
 * Only called from user level code at the moment.
 *
 * Entry:
 *    window        packet window receiving the packet
 *    packet        the packet to be added
 *    timeout       timeout value (<0 for indefinite) to wait until success.
 *
 * returns true if success or false if failure
 */
bool packet_window_add_sequential_packet(packet_window_t *window, packet_t *packet, int timeout)
{
    bool success = false;

    if (window != NULL) {

        /* Wait for room */
        if (os_acquire_counting_semaphore_with_timeout(window->room, timeout)) {

            packet_window_lock(window);

            /* Ensure the queue isn't full */
            assert(window->next_in_queue != window->length);

            /* And the slot is not in use */
            assert(!window->queue[window->next_in_queue].inuse);
        
            /* Allocate a sequence number for this packet */
            int sequence = window->sequence + window->next_in_queue;

            /* Assign the correct sequence number to the packet */
            set_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);

//printf("%s: adding packet %d to slot %d of %d to %d\n", __func__, sequence, window->next_in_queue, window->sequence, window->sequence + window->length - 1);

            /* Add the packet to the queue */
            window->queue[window->next_in_queue].packet = ref_packet(packet);
            window->queue[window->next_in_queue].sequence = sequence;
            window->queue[window->next_in_queue].inuse = true;

            /* Update to the next available sequential slot */
            window->next_in_queue++;

            /* Keep track of how many packets are in queue */
            window->num_in_queue++;
 
            packet_window_unlock(window);

            success = true;
        }
    }

    return success;
}

/*
 * packet_window_add_random_packet
 *
 * Randomly insert a packet into the window.  It's sequence number must be between
 * <sequence> and <sequence> + <length>.  If not, it returns false.
 *
 * If it's sequence number is one beyond the length and 'timeout' is < 0,
 * this function will block until successful.
 *
 * Entry:
 *    window           The packet window object.
 *    packet           The packet to store
 *
 * Returns 0 if success; <0 if full and >0 if duplicate packet
 */
int packet_window_add_random_packet(packet_window_t *window, packet_t *packet)
{
    int results = -1;

    if (window != NULL) {

        int sequence = get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN);

        packet_window_lock(window);

        /* Highest sequence number permitted in this queue at the moment */
        int highest_sequence = window->sequence + window->length - 1;

        if (sequence >= window->sequence && sequence <= highest_sequence) {

            int slot = sequence - window->sequence;

//printf("%s: adding packet %d to slot %d of %d to %d\n", __func__, sequence, slot, window->sequence, window->sequence + window->length - 1);

            /* Put the packet into the window slot if not already in use */
            if (! window->queue[slot].inuse) {
                window->queue[slot].packet = ref_packet(packet);
                window->queue[slot].sequence = sequence;
                window->queue[slot].inuse = true;

                /*
                 * Release this packet plus consecutive packets beyond it to available state.
                 */
                while (window->next_in_queue < window->length && window->queue[window->next_in_queue].inuse) {
                    os_release_counting_semaphore(window->available);
                    window->next_in_queue++;
                }

                window->num_in_queue++;

                results = 0;
            } else {
                /* Packet already there - it must have the same sequence as expected */
                assert(sequence == window->queue[slot].sequence);
                /* Not using, so discard */
                results = 1;
            }
        }

        packet_window_unlock(window);
    }

    return results;
}

/*
 * packet_window_remove_sequential_packet
 *
 * Always removes packets from window from front of queue.
 *
 * Entry:
 *     window                 packet window
 *     packet                 pointer to cell to receive packet address
 *     timeout                0 to test; <0 to wait forever; else number of mS to wait
 *
 * Returns true if a packet removed with the packet in the 'packet' variable.
 */
bool packet_window_remove_sequential_packet(packet_window_t *window, packet_t **packet, int timeout)
{
    bool ok = false;

    if (window != NULL) {
        do {
            packet_window_lock(window);

            /* If shutdown is set when the queue is empty, return a NULL packet */
            if (packet_window_is_shutdown(window) && window->num_in_queue == 0) {
                *packet = NULL;
                ok = true;

            } else {

                /* Wait for data to appear in window */
                window->user_blocked = true;
                packet_window_unlock(window);

                os_acquire_counting_semaphore_with_timeout(window->available, timeout);

                packet_window_lock(window);
                window->user_blocked = false;

                if (window->queue[0].inuse) {
                    assert(window->queue[0].packet != NULL);
                    *packet = ref_packet(window->queue[0].packet);
//printf("%s: removed from slot 0: %d\n", __func__, window->queue[0].sequence);
                    packet_window_discard_top_packet(window);
                    ok = true;
                } else {
                    ok = false;
                }

            }

            packet_window_unlock(window);

        } while (!ok && timeout < 0);
    }

    return ok;
}

/*
 * packet_window_get_all_packets
 *
 * Returns a list of the extant packets in the queue.  The number
 * of packets returns is limited by num_packets.  The packets
 * have been ref'd.
 *
 * Returns the number of packets placed in the user's packet list.
 */
int packet_window_get_all_packets(packet_window_t *window, packet_t *packets[], int num_packets)
{
    int packet_count = 0;

    if (window != NULL) {
        packet_window_lock(window);

        for (int slot = 0; slot < window->length && packet_count < num_packets; ++slot) {
            if (window->queue[slot].inuse && window->queue[slot].packet != NULL) {
                packets[packet_count++] = ref_packet(window->queue[slot].packet);
            }
        }

        packet_window_unlock(window);
    }

    return packet_count;
}

/*
 * packet_window_get_processed_packets
 *
 * Return the next expected packet sequence number plus a mask of
 * non-contiguous packets that have been queued but not processed.
 *
 * Returns:
 *    *sequence      The next sequence number expected to be added to queue
 *    *packet_mask   A bit mask of additional packets in que, with the LSB
 *                   corresponding to '*sequence' + 1 and so forth.
 *                   We skip sequence + 1, because had been in the queue,
 *                   the 'sequence' would be updated as well.
 *
 * Returns true if successful, other false to indicate window is shutdown.
 */
bool packet_window_get_processed_packets(packet_window_t *window, int *sequence, uint32_t *packet_mask)
{
    bool ok = false;

    if (window != NULL) {
        packet_window_lock(window);

        ok = true;

        int slot;

        bool done = false;

        int window_sequence = window->sequence;

        /* Find the first packet that hasn't been received */
        for (slot = 0; !done && slot < window->length; ++slot) {
            if (window->queue[slot].inuse && window->queue[slot].packet != NULL) {
                /* The sequence number of the packet must be the window sequence number + slot number */
                assert(window->queue[slot].sequence == window_sequence);
                window_sequence++;
            } else {
                /* Found a hole */
                done = true;
            }
        }

        *sequence = window_sequence;   /* Next *expected* sequence number */
        *packet_mask = 0;

        unsigned int bit = 1;

        /* Now set a bit for all packets that HAVE been received past this point */
        while (slot < window->length) {
            if (window->queue[slot].inuse) {
                /* Bitmask of extra packets that have been received */
                *packet_mask |= bit;
            }
            bit <<= 1;
            ++slot;
        }
    
        packet_window_unlock(window);
    }

    return ok;
}

/*
 * packet_window_release_processed_packets
 *
 *  Release and trim queue for all packets processed.
 *
 * Entry:
 *    sequence     - Acknowledge all sequence numbers less than this number.
 *    packet_mask  - Acknowledge additional packets by bit mask starting at sequence+1.
 *
 * Returns number of packets remaining in window
 */
int packet_window_release_processed_packets(packet_window_t *window, int sequence, uint32_t packet_mask)
{
    if (window != NULL) {

        packet_window_lock(window);
    
        /* Remove all directly acknowledged packets */
        while (window->queue[0].inuse && window->queue[0].sequence < sequence) {
//printf("%s: removed from slot 0: %d\n", __func__, window->queue[0].sequence);
            packet_window_discard_top_packet(window);
        }
   
        /* Go through the remaining inuse items and acknowledge them according to the packet mask */
        int ack_sequence = sequence;
        while (packet_mask != 0) {
            ++ack_sequence;  /* Sequence number corresponding to LSB of mask */
            if ((packet_mask & 1) != 0) {
                bool found = false;

                /* TODO: It might be smarter to go through this sequentially rather than starting from the top each time... */
                for (int slot = 1; !found && slot < window->length; ++slot) {        

                    if (window->queue[slot].inuse && window->queue[slot].sequence == ack_sequence) {
                        found = true;

                        /* Release the packet to the pool */
                        if (window->queue[slot].packet != NULL) {
//printf("%s: removed from slot %d: %d\n", __func__, slot, window->queue[slot].sequence);
                            release_packet(window->queue[slot].packet);
    
                            /* Count the packet as processed (but leave slot 'in use') */
                            window->queue[slot].packet = NULL;

                            /* Fix number of actual packets left in queue */
                            assert(window->num_in_queue != 0);
                            window->num_in_queue--;

#if 0
                        } else {
                            printf("%s: releasing already released packet %d in window %d to %d\n",
                                   __func__, window->queue[slot].sequence, window->sequence, window->sequence + window->length - 1);
#endif
                        }
                    }
                }

                if (!found) {
                    printf("%s: releasing non-existant packet %d in window %d to %d\n", __func__, ack_sequence, window->sequence, window->sequence + window->length - 1);
                }
            }
            packet_mask >>= 1;
        }

        assert(packet_mask == 0);
    
        packet_window_unlock(window);
    }

    return packet_window_packet_count(window);
}

void packet_window_shutdown_window(packet_window_t *window)
{
    if (window != NULL) {
        window->shutdown = true;

        /* Give it at least one more signal to wake up the output side */
        os_release_counting_semaphore(window->available);
    }
}

bool packet_window_is_shutdown(packet_window_t *window)
{
    return window ? window->shutdown : true;
}

int packet_window_packet_count(packet_window_t *window)
{
    return window ? window->num_in_queue : 0;
}

bool packet_window_user_is_blocked(packet_window_t *window)
{
    return window ? window->user_blocked : false;
}

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
#include "commands.h"

/* The last # has to be the largest */
static int sequence_number_order[] = {
    3,2,1,0,6,7,5,4,8,9,10,12,13,14,15,16,17,18,11,19
};

#define TEST_WINDOW_SIZE 8

os_thread_t consumer_thread;

/*
 * This thread accepts packets out of order and receives them in sequence number order.
 */
void packet_window_receive_sequential(void *param)
{
    packet_window_t *window = (packet_window_t*) param;

    ESP_LOGI(TAG, "%s: running", __func__);

    int sequence = -1;

    do {
        packet_t *packet;

        if (packet_window_remove_sequential_packet(window, &packet, /* timeout */ -1)) {
            printf("%s: got a packet %p\n", __func__, packet);
            sequence = get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN);
            const char* data = get_str_field(packet, STREAM_PAYLOAD, packet->length - STREAM_PAYLOAD);
            printf("%s: packet %d: \"%s\"\n", __func__, sequence, data);
            free((void*) data);
            release_packet(packet);

#if 0
            int sequence;
            uint32_t packet_mask;
            packet_window_get_processed_packets(window, &sequence, &packet_mask);
            printf("%s: accepted sequence numbers: %d %04x\n", __func__, sequence, packet_mask);

            packet_t *packets[TEST_WINDOW_SIZE];
            int num = packet_window_get_all_packets(window, packets, ELEMENTS_OF(packets));
            for (int n = 0; n < num; ++n) {
                if (packets[n] != NULL) {
                    printf("%s: packet %d is %d\n", __func__, n, get_uint_field(packets[n], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN));
                    release_packet(packets[n]);
                } else {
                    printf("%s: slot %d is empty\n", __func__, n);
                }
            }
#endif
        } else {
            printf("%s: no data\n", __func__);
        }
    } while (sequence != sequence_number_order[ELEMENTS_OF(sequence_number_order)-1]);

    printf("%s: exiting\n", __func__);

    consumer_thread = NULL;

    os_exit_thread();
}

/*
 * This thread accepts sequentially generated packets and 'sends' and acknowledges out of order.
 */
void packet_window_receive_random(void *param)
{
    packet_window_t *window = (packet_window_t*) param;

    printf("%s: running\n", __func__);

    int num_packets;

    do {
        os_delay(1000);

        packet_t *packets[TEST_WINDOW_SIZE];
        num_packets = packet_window_get_all_packets(window, packets, ELEMENTS_OF(packets));

#if 0
        for (int n = 0; n < num; ++n) {
            if (packets[n] != NULL) {
                printf("%s: packet %d is %d\n", __func__, n, get_uint_field(packets[n], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN));
                release_packet(packets[n]);
            } else {
                printf("%s: packet %d is NULL\n", __func__, n);
            }
        }
#endif
        if (num_packets != 0) {
            /* Pick a random one to remove and 'process' */
            int packet_num = esp_random() % num_packets;
            int sequence = get_uint_field(packets[packet_num], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN);

            printf("%s: processing packet %d window is %d to %d\n", __func__, sequence, window->sequence, window->sequence + window->length - 1);

            uint32_t packet_mask;
            if (sequence > window->sequence) {
                packet_mask = 1 << (sequence - window->sequence - 1);
                sequence = window->sequence;
            } else {
                /* First in window, so just pull the top element */
                sequence = sequence + 1;
                packet_mask = 0;
            }

            //printf("%s: releasing sequence %d mask %04x\n", __func__, sequence, packet_mask);
            packet_window_release_processed_packets(window, sequence, packet_mask);

#if 0
            packet_window_get_processed_packets(window, &sequence, &packet_mask);
            printf("%s: window sequence is %d mask %04x\n", __func__, sequence, packet_mask);

            packet_t *packets[TEST_WINDOW_SIZE];
            int num = packet_window_get_all_packets(window, packets, ELEMENTS_OF(packets));
            for (int n = 0; n < num; ++n) {
                if (packets[n] != NULL) {
                    printf("%s: slot %d is %d\n", __func__, n, get_uint_field(packets[n], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN));
                    release_packet(packets[n]);
                } else {
                    printf("%s: slot %d is empty\n", __func__, n);
                }
            }
#endif
        } else {
            printf("%s: no data; window sequence is %d\n", __func__, window->sequence);
        }
    } while (num_packets != 0 && !packet_window_is_shutdown(window));

    printf("%s: exiting\n", __func__);

    consumer_thread = NULL;

    os_exit_thread();
}


int run_packet_window_test(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "r/t", "Run packet_window tests as receiver or transmitter");
    } else if (argc > 1) {
        /* Create input window */
        packet_window_t *window = packet_window_create(TEST_WINDOW_SIZE);

        if (argv[1][0] == 'r') {
            /* Create consumer */
            consumer_thread = os_create_thread(packet_window_receive_sequential, "rcv_sequential", 8192, 0, (void *) window);

            /* Produce packets out of order and see they are consumed properly */
            for (int count = 0; count < ELEMENTS_OF(sequence_number_order); ++count) {
                packet_t *packet = allocate_packet();
                if (packet != NULL) {
                    packet->length = STREAM_PAYLOAD;

                    int sequence = sequence_number_order[count];

                    set_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN, STREAM_PROTOCOL);
                    set_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
                    packet->length += sprintf((char*) (packet->buffer + packet->length), "Packet %d", sequence);

                    
                    int pwa_results = packet_window_add_random_packet(window, packet);

                    if (pwa_results < 0) {
                        /* Failed so loop a few times to retry */
                        int tries = 10;
                        do {
                             printf("%s: failed insert - retrying %d\n", __func__, sequence);
                             os_delay(500);
                             --tries;
                        } while (tries != 0 && !packet_window_add_random_packet(window, packet));

                        if (tries == 0) {
                            printf("%s: failed to insert random %d: %d\n", __func__, count, sequence);
                        }
                    } else if (pwa_results > 0) {
                        printf("%s: packet %d is duplicate\n", __func__, sequence);
                    } else {
                        /* All good */
                    }

                } else {
                    printf("%s: no packets left\n", __func__);
                    count = ELEMENTS_OF(sequence_number_order);
                }
            }
        } else {
            consumer_thread = os_create_thread(packet_window_receive_random, "rcv_random", 8192, 0, (void *) window);

            /* Produce packets in sequential order and see they are consumed properly */
            for (int count = 0; count < ELEMENTS_OF(sequence_number_order); ++count) {
                packet_t *packet = allocate_packet();
                if (packet != NULL) {
                    packet->length = STREAM_PAYLOAD;

                    set_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN, STREAM_PROTOCOL);
                    packet->length += sprintf((char*) (packet->buffer + packet->length), "Packet %d", count);

                    
                    if (!packet_window_add_sequential_packet(window, packet, -1)) {
                        printf("%s: failed to insert sequential  %d\n", __func__, count);
                    } else {
                        printf("%s: inserted sequential %d\n", __func__, count);
                    }

                } else {
                    printf("%s: no packets left\n", __func__);
                    count = ELEMENTS_OF(sequence_number_order);
                }
            }
        }

        packet_window_shutdown_window(window);

        printf("%s: exiting...\n", __func__);
        while (consumer_thread != NULL) {
            os_delay(5000);
        }
        packet_window_release(window);
    }

    return 0;
}
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

void packet_window_init(void)
{
#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    add_command("pwt", run_packet_window_test);
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */
}

void packet_window_deinit(void)
{
#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    remove_command("pwt");
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */
}

