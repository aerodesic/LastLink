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

static bool dummy_release(packet_t* packet)
{
    ESP_LOGE(TAG, "%s: called when shouldn't be for %p", __func__, packet);
    return false;
}

static void remove_top_packet(packet_window_t *window)
{
    assert(window->queue[0].inuse);

ESP_LOGI(TAG, "%s: removing packet %d", __func__, window->queue[0].sequence);

    /* Roll up the queue to remove this element */
    memcpy(window->queue, window->queue + 1, sizeof(window->queue[0]) * (window->length - 1));
    window->queue[window->length - 1].packet = NULL;
    window->queue[window->length - 1].inuse = false;
    window->queue[window->length - 1].sequence = 0;

    /* The first packet sequence number is now + 1 */
    window->sequence++;

    os_release_counting_semaphore(window->room, 1);
}

packet_window_t *create_packet_window(int slots)
{
    size_t size = sizeof(packet_window_t) + sizeof(packet_slot_t) * (slots - 1);

    packet_window_t *window = (packet_window_t *) malloc(size);
    if (window != NULL) {
        memset(window, 0, size);

        window->length = slots;
        window->lock = os_create_recursive_mutex();
        window->available = os_create_counting_semaphore(slots, 0);
        window->room = os_create_counting_semaphore(slots, 0);

        if (window->lock == NULL || window->available == NULL || window->room == NULL) {
            release_packet_window(window, dummy_release);
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
 *    release_packet           A function to release a packet.
 */
void release_packet_window(packet_window_t *window, bool (*release_packet)(packet_t*))
{
    os_acquire_recursive_mutex(window->lock);

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


    os_release_recursive_mutex(window->lock);

    os_delete_mutex(window->lock);

    free((void*) window);
}


/*
 * add_packet_to_window
 *
 * Insert a packet into the window.  It's sequence number must be between
 * <sequence> and <sequence> + <length>.  If not, it returns false.
 *
 * If it's sequence number is one beyond the length and 'timeout' is < 0,
 * this function will block until successful.
 *
 * Entry:
 *    window           The packet window object.
 *    packet           The packet to store
 *    sequence         The packet's sequence number.
 *    timeout          -1 to wait forever, otherwise ms to wait for data; 0 is test and return immediately.
 *
 * Returns true if successfully inserted into queue.
 */
bool add_packet_to_window(packet_window_t *window, packet_t *packet, int sequence, int timeout)
{
    bool ok = false;
    bool fail = false;

ESP_LOGI(TAG, "%s: called with sequence %d of %d to %d", __func__, sequence, window->sequence, window->sequence + window->length - 1);
    os_acquire_recursive_mutex(window->lock);

    while (!ok && !fail) {

        if (sequence >= window->sequence && sequence < window->sequence + window->length) {

ESP_LOGI(TAG, "%s: adding %d window %d to %d", __func__, sequence, window->sequence, window->sequence + window->length - 1);

            int slot = sequence - window->sequence;

            /* if slot is already occupied, just say we did it */
            if (! window->queue[slot].inuse) {
                window->queue[slot].packet = packet;
                window->queue[slot].sequence = sequence;
                window->queue[slot].inuse = true;
                window->num_in_queue++;
            } else {
                /* Packet already there - it must have the same sequence as expected */
                assert(sequence == window->queue[slot].sequence);
                /* Don't need to pack this one */
                release_packet(packet);
            }

            os_release_counting_semaphore(window->available, 1);

            ok = true;

        /* If a timeout was specified, wait that amount and then try again but don't wait second time if not -1 */
        } else if (timeout != 0) {
ESP_LOGI(TAG, "%s: no room for %d in %d to %d; waiting", __func__, sequence, window->sequence, window->sequence + window->length - 1);

            /* No place for packet, so wait until room */
            os_release_recursive_mutex(window->lock);

            /* Wait for something to release some room */
            os_acquire_counting_semaphore_with_timeout(window->room, timeout);

            /* Acquire the window and scan again */
            os_acquire_recursive_mutex(window->lock);

            /* Don't wait second time through if not a forever wait */
            if (timeout > 0) {
                timeout = 0;
            }
        } else {
            fail = true;
        }
    }

    os_release_recursive_mutex(window->lock);

    return ok;
}

/*
 * remove_packet_from_window
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
bool remove_packet_from_window(packet_window_t *window, packet_t **packet, int timeout)
{
    bool ok = false;

    do {
        /* If shutdown is set when the queue is empty, return a NULL packet */
        if (is_shutdown(window) && window->num_in_queue == 0) {
            *packet = NULL;
            ok = true;
        } else {
            if (window->queue[0].packet == NULL) {
                /* Wait for a release */
                os_acquire_counting_semaphore_with_timeout(window->available, timeout);
            }

            if (window->queue[0].inuse && window->queue[0].packet != NULL) {
                os_acquire_recursive_mutex(window->lock);

                if (window->queue[0].inuse && window->queue[0].packet != NULL) {
                    *packet = window->queue[0].packet;
                    window->num_in_queue--;
                    remove_top_packet(window);
                    ok = true;
                } else {
                    ok = false;
                }

                os_release_recursive_mutex(window->lock);
            }
        }

    } while (!ok && timeout < 0);

    return ok;
}

/*
 * get_packets_in_window
 *
 * Returns a list of the extant packets in the queue.  The number
 * of packets returns is limited by num_packets.
 *
 * Returns the number of packets placed in the user's packet list.
 */
int get_packets_in_window(packet_window_t *window, packet_t *packets[], size_t num_packets)
{
    os_acquire_recursive_mutex(window->lock);

    int packet_count = 0;
    for (int slot = 0; slot < window->length && packet_count < num_packets; ++slot) {
        if (window->queue[slot].inuse && window->queue[slot].packet != NULL) {
            packets[packet_count++] = window->queue[slot].packet;
        }
    }

    os_release_recursive_mutex(window->lock);

    return packet_count;
}

/*
 * get_accepted_packet_sequence_numbers
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
 */
void get_accepted_packet_sequence_numbers(packet_window_t *window, int *sequence, uint32_t *packet_mask)
{
    os_acquire_recursive_mutex(window->lock);

    int slot;

    bool done = false;

    int window_sequence = window->sequence;

    /* Find first packet out of sequence and ack to include that */
    for (slot = 0; !done && slot < window->length; ++slot) {
        if (window->queue[slot].inuse) {
            /* The sequence number of the packet must be the window sequence number + slot number */
            assert(window->queue[slot].sequence == window_sequence);
            window_sequence++;
        } else {
            done = true;
        }
    }

    *sequence = window_sequence;   /* Next *expected* sequence number */
    *packet_mask = 0;

    while (slot < window->length) {
        if (window->queue[slot].inuse && window->queue[slot].packet == NULL) {
            /* Bitmask of extra packets that have been removed */
            *packet_mask |= (1 << (slot - 1));
        }
        ++slot;
    }

    os_release_recursive_mutex(window->lock);
}

/* release_accepted_packets_in_window
 *
 *  Release and trim queue for all packets processed.
 *
 * Entry:
 *    sequence     - Acknowledge all sequence numbers less than this number.
 *    packet_mask  - Acknowledge additional packets by bit mask starting at sequence+1.
 *
 * Returns number of packets released.
 */
int release_accepted_packets_in_window(packet_window_t *window, int sequence, uint32_t packet_mask, bool (*release_packet)(packet_t*))
{
    int packets_released = 0;

    os_acquire_recursive_mutex(window->lock);

    /* Remove all directly acknowledged packets */
    while (window->queue[0].inuse && window->queue[0].sequence < sequence) {
        window->num_in_queue--;
        if (window->queue[0].packet != NULL) {
            release_packet(window->queue[0].packet);
        }
        remove_top_packet(window);
        packets_released++;
    }

    /* Release packets for all bits in the packet_mask */
    for (int slot = 1; packet_mask != 0 && slot < window->length; ++slot) {
        if ((packet_mask & 1) != 0) {
            if (window->queue[slot].inuse) {
                /* Release the packet to the pool */
                window->num_in_queue--;
                if (window->queue[slot].packet != NULL) {
                    release_packet(window->queue[slot].packet);
                }
                /* Count the packet as processed */
                ++packets_released;
                window->queue[slot].packet = NULL;
            } else {
                ESP_LOGE(TAG, "%s: releasing already released packet %d in window %d to %d",
                         __func__, window->queue[slot].sequence, window->sequence, window->sequence + window->length - 1);
            }
        }
        packet_mask >>= 1;
    }

    /* Now roll up all empty slots and adjust window counts */
    /* Remove all directly acknowledged packets */
    while (window->queue[0].inuse && window->queue[0].packet == NULL) {
        remove_top_packet(window);
    }

    os_release_recursive_mutex(window->lock);

    return packets_released;
}

void shutdown_window(packet_window_t *window)
{
    window->shutdown = true;

    /* Give it at least one more signal to wake up the reader */
    os_release_counting_semaphore(window->available, 1);
}

bool is_shutdown(packet_window_t *window)
{
    return window->shutdown;
}

int packets_in_window(packet_window_t *window)
{
    return window->num_in_queue;
}

/* The next available sequence number for outbound packets */
int next_sequence(packet_window_t *window)
{
    return window->sequence + packets_in_window(window);
}

#if CONFIG_LASTLINK_TABLE_COMMANDS
#include "commands.h"

static int sequence_number_order[] = {
    3,2,1,0,6,7,5,4,8,9,10
};

#define TEST_WINDOW_SIZE 5

os_thread_t consumer_thread;

/*
 * This thread accepts packets out of order and receives them in sequence number order.
 */
void packet_window_receiver(void *param)
{
    packet_window_t *window = (packet_window_t*) param;

    ESP_LOGI(TAG, "%s: running", __func__);

    int sequence = -1;

    do {
        packet_t *packet;

        if (remove_packet_from_window(window, &packet, /* timeout */ -1)) {
            ESP_LOGI(TAG, "%s: got a packet %p", __func__, packet);
            sequence = get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN);
            const char* data = get_str_field(packet, STREAM_PAYLOAD, packet->length - STREAM_PAYLOAD);
            ESP_LOGE(TAG, "%s: packet %d: \"%s\"", __func__, sequence, data);
            free((void*) data);
            release_packet(packet);

            int sequence;
            uint32_t packet_mask;
            get_accepted_packet_sequence_numbers(window, &sequence, &packet_mask);
            ESP_LOGI(TAG, "%s: accepted sequence numbers: %d %04x", __func__, sequence, packet_mask);

            packet_t *packets[TEST_WINDOW_SIZE];
            int num = get_packets_in_window(window, packets, ELEMENTS_OF(packets));
            for (int n = 0; n < num; ++n) {
                if (packets[n] != NULL) {
                    ESP_LOGI(TAG, "%s: packet %d is %d", __func__, n, get_uint_field(packets[n], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN));
                } else {
                    ESP_LOGI(TAG, "%s: slot %d is empty", __func__, n);
                }
            }
        } else {
            ESP_LOGI(TAG, "%s: no data", __func__);
        }
    } while (sequence != sequence_number_order[ELEMENTS_OF(sequence_number_order)-1]);

    ESP_LOGI(TAG, "%s: exiting", __func__);

    consumer_thread = NULL;

    os_exit_thread();
}

/*
 * This thread accepts sequentially generated packets and 'sends' and acknowledges out of order.
 */
void packet_window_transmitter(void *param)
{
    packet_window_t *window = (packet_window_t*) param;

    ESP_LOGI(TAG, "%s: running", __func__);

    int sequence = -1;

    do {
        os_delay(1000);
        packet_t *packets[TEST_WINDOW_SIZE];
        int num = get_packets_in_window(window, packets, ELEMENTS_OF(packets));
        for (int n = 0; n < num; ++n) {
            if (packets[n] != NULL) {
                ESP_LOGI(TAG, "%s: packet %d is %d", __func__, n, get_uint_field(packets[n], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN));
            } else {
                ESP_LOGI(TAG, "%s: packet %d is NULL", __func__, n);
            }
        }

        if (num != 0) {
            /* Pick a random one to remove and 'process' */
            int packet_num = esp_random() % num;
            int sequence = get_uint_field(packets[packet_num], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN);

            ESP_LOGI(TAG, "%s: processing packet %d window is %d to %d", __func__, sequence, window->sequence, window->sequence + window->length - 1);

            uint32_t packet_mask;
            if (sequence > window->sequence) {
                packet_mask = 1 << (sequence - window->sequence - 1);
                sequence = window->sequence;
            } else {
                /* First in window, so just pull the top element */
                sequence = sequence + 1;
            }

            ESP_LOGI(TAG, "%s: releasing sequence %d mask %04x", __func__, sequence, packet_mask);
            release_accepted_packets_in_window(window, sequence, packet_mask, release_packet_plain);

            get_accepted_packet_sequence_numbers(window, &sequence, &packet_mask);
            ESP_LOGI(TAG, "%s: window sequence is %d mask %04x", __func__, sequence, packet_mask);

            packet_t *packets[TEST_WINDOW_SIZE];
            int num = get_packets_in_window(window, packets, ELEMENTS_OF(packets));
            for (int n = 0; n < num; ++n) {
                if (packets[n] != NULL) {
                    ESP_LOGI(TAG, "%s: slot %d is %d", __func__, n, get_uint_field(packets[n], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN));
                } else {
                    ESP_LOGI(TAG, "%s: slot %d is empty", __func__, n);
                }
            }

        } else {
            ESP_LOGI(TAG, "%s: no data; window sequence is %d", __func__, window->sequence);
        }
    } while (sequence != sequence_number_order[ELEMENTS_OF(sequence_number_order)-1]);

    ESP_LOGI(TAG, "%s: exiting", __func__);

    consumer_thread = NULL;

    os_exit_thread();
}


int run_packet_window_test(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "r/t", "Run packet_window tests as receiver or transmitter");
    } else if (argc > 1) {
        /* Create input window */
        packet_window_t *window = create_packet_window(TEST_WINDOW_SIZE);
        bool sequential;

        if (argv[1][0] == 'r') {
            /* Create consumer */
            sequential = false;
            consumer_thread = os_create_thread(packet_window_receiver, "consumer", 8192, 0, (void *) window);
        } else {
            sequential = true;
            consumer_thread = os_create_thread(packet_window_transmitter, "transmitter", 8192, 0, (void *) window);
        }

        /* Produce packets out of order and see they are consumed properly */
        for (int count = 0; count < ELEMENTS_OF(sequence_number_order); ++count) {
            packet_t *packet = allocate_packet();
            if (packet != NULL) {
                packet->length = STREAM_PAYLOAD;

                int sequence = sequential ? count : sequence_number_order[count];

                set_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN, STREAM_PROTOCOL);
                set_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
                packet->length += sprintf((char*) (packet->buffer + packet->length), "Packet %d", sequence);

                if (sequential) {
                    ESP_LOGE(TAG, "%s: inserting packet %d", __func__, sequence);
                } else {
                    ESP_LOGI(TAG, "%s: inserting packet %d", __func__, sequence);
                }

                if (!add_packet_to_window(window, packet, sequence, /* timeout */ -1)) {
                    ESP_LOGI(TAG, "%s: failed to insert %d", __func__, sequence);
                }

            } else {
                ESP_LOGI(TAG, "%s: no packets left", __func__);
                count = ELEMENTS_OF(sequence_number_order);
            }
        }
        ESP_LOGI(TAG, "%s: exiting...", __func__);
        if (consumer_thread != NULL) {
            os_delay(5000);
        }
        if (consumer_thread != NULL) {
            os_delete_thread(consumer_thread);
        }
        release_packet_window(window, release_packet_plain);
    }

    return 0;
}
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */

void init_packet_window(void)
{
#if CONFIG_LASTLINK_TABLE_COMMANDS
    add_command("pwt", run_packet_window_test);
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */
}

void deinit_packet_window(void)
{
#if CONFIG_LASTLINK_TABLE_COMMANDS
    remove_command("pwt");
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */
}

