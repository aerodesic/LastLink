/*
 * All packets generated should go through a test for 'has the previous one been transmitted'
 * in order to avoid flooding the packet transmit queue with redundant messages that haven't
 * been sent.
 *
 * Give packet assembly takes care of packets that are out of order where order matters, the
 * packet transmit queue should be built such that a new packet of type 'x' send to radio 'y'
 * will purge the queue of any older messages of this type for the radio before inserting
 * a new one.  Further, it would be useful to simply ignore attempts to send packets with the
 * same data as before, just leaving them in the queue instead of removing and inserting at
 * the end of the queue.
 *
 * Should be simple enough to check if packet is has same address, as data packets are not
 * duplicated for transmission.
 *
 * Control packets are duplicated, but this could be relaxed by using a holder for the packet
 * until transmitted and retransmissions would just use the same packet and not requeue
 * if not yet transmitted.
 * 
 * All packets have a 'queued' flag so we can use this to detemine if they are on any radio
 * queue.
 *
 * This needs thought...
 */
 
 /*
 * lsocket.c
 *
 * Lightweight Socket layer.
 *
 * Example DATAGRAM:
 *      // Create socket
 *      int s = ls_socket(LS_DATAGRAM);
 *      // Bind port
 *      ls_bind(s, 1234)
 *      ls_connect(s, <destination address>, <destination port>)
 *      int len = ls_read(s, buf, buflen);
 *          -or-
 *      int len = ls_write(s, buf, buflen);
 *          ... more activity ...
 *      ls_close(s);
 *
 *
 * Example OUTBOUND STREAM:
 *      // Create socket
 *      int s = ls_socket(LS_STREAM);
 *      // Bind port
 *      ls_bind(s, 1234)
 *      int rc = ls_connect(s, <destination address>, <destination port>)
 *      if (rc == LS_NO_ERROR) {
 *          int len = ls_read(s, buf, buflen);
 *              -or-
 *          int len = ls_write(s, buf, buflen);
 *          ... more activity ...
 *          ls_close(s);
 *      }
 *
 * Example OUTBOUND STREAM:
 *      // Create socket
 *      int listen_socket = ls_socket(LS_STREAM);
 *      // Bind port
 *      ls_bind(socket, 1234)
 *      int new_connection = ls_listen(listen_socket, 5, 0);
 *         ( when connection arrives, ls_listen returns with allocated socket )
 *      if (new_connection >= 0) {
 *          int len = ls_read(new_connection, buf, buflen);
 *              -or-
 *          int len = ls_write(new_connection, buf, buflen);
 *          ... more activity ...
 *          ls_close(new_connection);
 *      }
 *      ... more listen ...
 *      ls_close(listen_socket);
 *
 */

#include <string.h>

#include "sdkconfig.h"

#if CONFIG_LASTLINK_ENABLE_SOCKET_LAYER
#include <stdarg.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_vfs.h"
#include "esp_vfs_dev.h"
//#include "esp_attr.h"

#include "esp_system.h"
#include "esp_log.h"

#include "os_specific.h"
#include "lsocket_internal.h"
#include "packets.h"
#include "linklayer.h"
#include "routes.h"
#include "packet_window.h"

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
#include "commands.h"
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

#define TAG "lsocket"

/* PING support */
#define PING_RETRY_STACK_SIZE    6000

static const char* ping_packet_format(const packet_t *packet);
static const char* pingreply_packet_format(const packet_t *packet);
static bool ping_packet_process(packet_t *packet);
static bool pingreply_packet_process(packet_t *packet);
static int find_ping_table_entry(int sequence);

static ls_error_t ls_write_helper(ls_socket_t *socket, const char* buf, size_t len, bool eor, ls_address_t address, ls_port_t port);

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
static void stream_shutdown_windows(ls_socket_t *socket);

/* Statemachine and methods */
static void       state_machine_start(const char* ident, ls_socket_t *socket, ls_socket_state_t state, int (*action)(ls_socket_t *socket), void (*results)(ls_socket_t* socket, ls_error_t error),  int timeout, int retries);
static void       state_machine_cancel(ls_socket_t *socket);
static void       state_machine_timeout(ls_socket_t *socket);
static void       state_machine_send_results(ls_socket_t *socket, ls_error_t response);
static ls_error_t state_machine_get_results(ls_socket_t *socket);
static bool       state_machine_packet_route_complete(bool success, packet_t *packet, void *data, radio_t *radio);
static void       state_machine_packet_transmit_complete(packet_t *packet, void *data, radio_t *radio);
static int        state_machine_send_add_callback(ls_socket_t *socket, packet_t *packet);
/* action_callback items */
static int        state_machine_action_send_connect(ls_socket_t *socket);
static int        state_machine_action_send_connect_ack(ls_socket_t *socket);
static int        state_machine_action_send_disconnect(ls_socket_t *socket);
static int        state_machine_action_send_disconnected(ls_socket_t *socket);
static int        state_machine_action_send_output_window(ls_socket_t *socket);
static int        state_machine_action_linger_check(ls_socket_t *socket);
static void       state_machine_results_linger_finish(ls_socket_t *socket, ls_error_t error);
static void       state_machine_results_shutdown_socket(ls_socket_t *socket, ls_error_t err);

#if CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLE
static void stream_send_keepalive(ls_socket_t *socket);
#endif /* CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLE */

static void stream_send_connect_ack(ls_socket_t *socket);

#ifdef NOTUSED
static void stream_send_disconnect(ls_socket_t *socket);
#endif /* NOTUSED */

static void stream_send_disconnected(ls_socket_t *socket);

static bool stream_packet_process(packet_t *packet);
static const char* stream_packet_format(const packet_t *packet);

static packet_t *add_data_ack(packet_window_t *window, packet_t *packet);
static int ls_ioctl_r_wrapper(int fd, int cmd, va_list args);

static packet_t *stream_packet_create_from_packet(const packet_t *packet, uint8_t flags, int sequence);
static packet_t *stream_packet_create_from_socket(const ls_socket_t *socket, uint8_t flags, int sequence);

#define SOCKET_SCANNER_STACK_SIZE          6000
#define SOCKET_SCANNER_DEFAULT_INTERVAL    100
static os_thread_t socket_scanner_thread_id;
static int socket_scanner_running;
static void socket_scanner_thread(void*);

static void ack_output_window(ls_socket_t *socket, int ack_sequence, uint32_t ack_mask);
static bool send_output_window(ls_socket_t *socket, bool disconect_on_error);
static bool socket_flush_current_packet(ls_socket_t *socket, bool wait);

static ls_socket_t *find_listening_socket_from_packet(const packet_t *packet);
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

static const char* socket_type_of(const ls_socket_t *socket);
static ls_error_t ls_dump_socket_ptr(const char* msg, const ls_socket_t *socket);

static bool datagram_packet_process(packet_t *packet);
static const char* datagram_packet_format(const packet_t *packet);

static packet_t *datagram_packet_create_from_socket(const ls_socket_t *socket);

static ls_socket_t *find_socket_from_packet(const packet_t *packet, ls_socket_type_t type);

static ls_error_t release_socket(ls_socket_t* socket);

static ls_error_t ls_close_ptr(ls_socket_t *socket);

typedef struct ping_table_entry {
    os_queue_t       queue;
    int              sequence;
    packet_t         *packet;
    os_thread_t      thread_id;
    bool             waiting;
    int              retries;
    bool             routed;
    ls_error_t       error;
    simpletimer_t    pingtime;
} ping_table_entry_t;

ping_table_entry_t   ping_table[CONFIG_LASTLINK_MAX_OUTSTANDING_PINGS];

static os_mutex_t    global_socket_lock;

static ls_socket_t   socket_table[CONFIG_LASTLINK_NUMBER_OF_SOCKETS];

#ifdef CONFIG_LASTLINK_SOCKET_LOCKING_DEBUG

#define GLOBAL_SOCKET_LOCK() \
    do {                                                            \
        ESP_LOGI(TAG, "%s: locking", __func__);                     \
        os_acquire_recursive_mutex(global_socket_lock);             \
        ESP_LOGI(TAG, "%s: locked", __func__);                      \
    } while(0)

#define GLOBAL_SOCKET_UNLOCK() \
    do {                                                            \
        ESP_LOGI(TAG, "%s: unlocking", __func__);                   \
        os_release_recursive_mutex(global_socket_lock);             \
    } while(0)

static inline bool lock_socket_debug(ls_socket_t* socket, bool try, const char *file, int line)
{
    /* Try without waiting */
    bool ok = os_acquire_recursive_mutex_with_timeout(socket->lock, 0);
    if (!ok) {
        /* Didn't get it, so if we are not just 'trying', go ahead and wait and show message */
        if (!try) {
            ESP_LOGI(TAG, "%s: **********************************************************************", __func__);
            ESP_LOGI(TAG, "%s: waiting for socket %d at lock [%s:%d] last locked at %s:%d", __func__, socket - socket_table, file, line, socket->last_lock_file ? socket->last_lock_file : "<NULL>", socket->last_lock_line);
            ESP_LOGI(TAG, "%s: **********************************************************************", __func__);
            ok = os_acquire_recursive_mutex(socket->lock);
            ESP_LOGI(TAG, "%s: **********************************************************************", __func__);
            ESP_LOGI(TAG, "%s: got socket %d lock", __func__, socket - socket_table);
            ESP_LOGI(TAG, "%s: **********************************************************************", __func__);
        }
    }

    if (ok) {
        socket->lock_count++;
        socket->last_lock_file = file;
        socket->last_lock_line = line;
    }

    return ok;
}

#define LOCK_SOCKET(socket)      lock_socket_debug(socket, false, __FILE__, __LINE__)
#define LOCK_SOCKET_TRY(socket)  lock_socket_debug(socket, true, __FILE__, __LINE__)

static inline void unlock_socket_debug(ls_socket_t *socket, const char *file, int line)
{
    if (socket->lock_count == 0) {
       printf("%s: socket not locked; last lock %s:%d\n", __func__, socket->last_lock_file, socket->last_lock_line);
    } else {
       socket->lock_count--;
    }
    os_release_recursive_mutex(socket->lock);
}
#define UNLOCK_SOCKET(socket)    unlock_socket_debug(socket, __FILE__, __LINE__)

#else /* !CONFIG_LASTLINK_SOCKET_LOCKING_DEBUG */

#define GLOBAL_SOCKET_LOCK()     os_acquire_recursive_mutex(global_socket_lock)
#define GLOBAL_SOCKET_UNLOCK()   os_release_recursive_mutex(global_socket_lock)
#define LOCK_SOCKET(socket)      os_acquire_recursive_mutex((socket)->lock)
#define LOCK_SOCKET_TRY(socket)  os_acquire_recursive_mutex_with_timeout((socket)->lock, 0)
#define UNLOCK_SOCKET(socket)    os_release_recursive_mutex((socket)->lock)
#endif /* CONFIG_LASTLINK_SOCKET_LOCKING_DEBUG */

/* For external use - note these don't help much in tracking locking/unlocking. */
bool lock_socket(ls_socket_t* socket)
{
    return LOCK_SOCKET(socket);
}

void unlock_socket(ls_socket_t* socket)
{
    UNLOCK_SOCKET(socket);
}

static const char* socket_type_of(const ls_socket_t *socket)
{
    const char* type = "UNDEFINED";
    if (socket != NULL) {
        switch (socket->socket_type) {
            case LS_UNUSED:   type = "Not Used";  break;
            case LS_INUSE:    type = "In Use";    break;
            case LS_DATAGRAM: type = "Datagram";  break;
            case LS_STREAM:   type = "Stream";    break;
            default:          type = "Unknown";   break;
        }
    }

    return type;
}

static const char* socket_state_of(const ls_socket_t *socket)
{
    switch (socket->state) {
        case LS_STATE_IDLE:                         return "IDLE";
        case LS_STATE_INUSE:                        return "INUSE";
        case LS_STATE_SOCKET:                       return "SOCKET";
        case LS_STATE_BOUND:                        return "BOUND";
        case LS_STATE_CONNECTED:                    return "CONNECTED";
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
        case LS_STATE_LISTENING:                    return "LISTENING";
        case LS_STATE_INBOUND_CONNECT:              return "INBOUND_CONNECT";
        case LS_STATE_OUTBOUND_CONNECT:             return "OUTBOUND_CONNECT";
        case LS_STATE_DISCONNECTING_FLUSH_START:    return "DISCONNECTING_FLUSH_START";
        case LS_STATE_DISCONNECTING_FLUSHING:       return "DISCONNECTING_FLUSHING";
        case LS_STATE_INBOUND_DISCONNECTING:        return "INBOUND_DISCONNECTING";
        case LS_STATE_OUTBOUND_DISCONNECTING:       return "OUTBOUND_DISCONNECTING";
        case LS_STATE_LINGER:                       return "LINGER";
        case LS_STATE_DISCONNECTED:                 return "DISCONNECTED";
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
        case LS_STATE_CLOSING:                      return "CLOSING";
        case LS_STATE_CLOSED:                       return "CLOSED";
        default:                                    return "UNKNOWN";
    }
}

/*
 * Ping packet create
 */
packet_t *ping_packet_create(int dest, int ping_sequence)
{
    packet_t* packet = linklayer_create_generic_packet(dest, PING_PROTOCOL, PING_LEN);
    if (packet != NULL) {
        set_uint_field(packet, PING_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN, ping_sequence);
    }

    return packet;
}

static bool ping_packet_process(packet_t *packet)
{
    bool handled = false;

    if (packet != NULL) {

        linklayer_lock();

        /* Sanity check - reuse last entry if full */
        if (packet->length >= MAX_PACKET_LEN) {
            packet->length = MAX_PACKET_LEN - ADDRESS_LEN;
        }

        /* Add our address to the chain of addresses */
        packet->length += ADDRESS_LEN;
        set_uint_field(packet, packet->length - ADDRESS_LEN, ADDRESS_LEN, linklayer_node_address);

        /* If for us, turn packet into PING_REPLY */
        if (linklayer_packet_is_for_this_node(packet)) {

            /* Alter the protocol */
            set_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN, PINGREPLY_PROTOCOL);

            /* Turn the packet around */
            set_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN, get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN));
            set_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
            set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);   /* Force packet to be routed */

            /* Rewind Metric */
            set_uint_field(packet, HEADER_METRIC, METRIC_LEN, 0);

            (void) linklayer_route_packet(ref_packet(packet));

            handled = true;
        }

        linklayer_unlock();

        release_packet(packet);
    }

    return handled;
}

static const char* ping_format_routes(const packet_t *packet)
{
    int num_routes = (packet->length - PING_ROUTE_TABLE) / ADDRESS_LEN;

    /* Allocate worst case length */
    char *routes = (char*) malloc(2 + (8 + 1) * num_routes + 2);
    char *routep = routes;

    *routep++ = '[';
    *routep++ = ' ';

    for (int route = 0; route < num_routes; ++route) {
        int len = sprintf(routep, "%X ", get_uint_field(packet, PING_ROUTE_TABLE + ADDRESS_LEN * route, ADDRESS_LEN));
        routep += len;
    }
    *routep++ = ']';
    *routep++ = '\0';

    return routes;
}

static const char* ping_packet_format(const packet_t *packet)
{
    char* info;

    int sequence = get_uint_field(packet, PING_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN);
    const char* routes = ping_format_routes(packet);

    asprintf(&info, "Ping: Seq %d Routes %s", sequence, routes);

    free((void*) routes);

    return info;
}

static const char* pingreply_packet_format(const packet_t *packet)
{
    char* info;

    int sequence = get_uint_field(packet, PING_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN);
    const char* routes = ping_format_routes(packet);

    asprintf(&info, "Ping Reply: Seq %d Routes %s", sequence, routes);

    free((void*) routes);

    return info;
}

static bool pingreply_packet_process(packet_t *packet)
{
    bool handled = false;

    if (packet != NULL) {

        linklayer_lock();

        if (linklayer_packet_is_for_this_node(packet)) {
            /* Look for ping request and deliver packet to process queue */
            int slot = find_ping_table_entry(get_uint_field(packet, PING_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN));
            if (slot >= 0 && ping_table[slot].queue != NULL && ping_table[slot].waiting) {
                if (ping_table[slot].thread_id != NULL) {
                    os_delete_thread(ping_table[slot].thread_id);
                    ping_table[slot].thread_id = NULL;
                }
                ping_table[slot].waiting = false;
                ref_packet(packet);
                if (!os_put_queue(ping_table[slot].queue, (os_queue_item_t) &packet)) {
                    ESP_LOGE(TAG, "%s: No room for ping reply", __func__);
                    release_packet(packet);
                }
            }

            handled = true;

        } else {
            /* Sanity check - reuse last entry if full */
            if (packet->length >= MAX_PACKET_LEN) {
                packet->length = MAX_PACKET_LEN - ADDRESS_LEN;
            }

            packet->length += ADDRESS_LEN;
            set_uint_field(packet, packet->length - ADDRESS_LEN, ADDRESS_LEN, linklayer_node_address);
        }

        linklayer_unlock();

        release_packet(packet);
    }

    return handled;
}

static int find_ping_table_entry(int sequence)
{
    int slot = -1;

    for (int index = 0; slot < 0 && index < ELEMENTS_OF(ping_table); ++index) {
        if (ping_table[index].sequence == sequence) {
            slot = index;
        }
    }

    return slot;
}

static int register_ping(packet_t *packet)
{
    int slot = find_ping_table_entry(0);
    if (slot >= 0) {
        /* Remember we are waiting on this sequence return */
        ping_table[slot].queue = os_create_queue(1, sizeof(packet_t*));
        ping_table[slot].sequence = get_uint_field(packet, PING_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN);
        ping_table[slot].packet = ref_packet(packet);
        ping_table[slot].error = 0;
    }

    return slot;
}

static packet_t *wait_for_ping_reply(int slot)
{
    packet_t *packet = NULL;

    if (slot >= 0 && slot < ELEMENTS_OF(ping_table)) {
        os_get_queue(ping_table[slot].queue, (os_queue_item_t) &packet);
        /* Allow the thread to die */
        ping_table[slot].waiting = false;
    }

    return packet;
}

static void release_ping_table_entry(int slot)
{
    linklayer_lock();

    if (ping_table[slot].thread_id != NULL) {
        os_delete_thread(ping_table[slot].thread_id);
        ping_table[slot].thread_id = NULL;
    }

    if (ping_table[slot].packet != NULL) {
        release_packet(ping_table[slot].packet);
        ping_table[slot].packet = NULL;
    }

    if (ping_table[slot].queue != NULL) {
        os_delete_queue(ping_table[slot].queue);
        ping_table[slot].queue = NULL;
    }

    /* Release the table */
    ping_table[slot].sequence = 0;

    linklayer_unlock();
}


/*
 * Resend the ping packet for the number of retries allowed
 *
 * (This could get recast as a single thread for all pings,
 *  and scan the ping table for active sequence numbers, reissuing
 *  new packets as necessary.)
 */
void ping_retry_thread(void* param)
{
    int slot = (int) param;

    bool done = false;

    do {
        /* Allow time for last packet to rattle around */
        if (!done) {
            os_delay(CONFIG_LASTLINK_PING_RETRY_TIMER);
        }

        linklayer_lock();

        if (ping_table[slot].waiting && ping_table[slot].retries != 0) {

            ping_table[slot].retries--;

            (void) linklayer_route_packet(ref_packet(ping_table[slot].packet));

        } else {
            /* Either timed out or was told to quit. */
            done = true;

            /* If we timed out - return error */
            if (ping_table[slot].waiting) {
                ping_table[slot].error = LSE_TIMEOUT;
                void *null = NULL;
                os_put_queue(ping_table[slot].queue, (os_queue_item_t) &null);
                ping_table[slot].waiting = false;
            }
        }

        linklayer_unlock();

    } while (!done);

    os_exit_thread();
}


/*
 * ping_has_been_routed.
 *
 * The original ping is passed to this function in case we need it.
 *
 * A NULL packet indicates the route failed.
 */
static bool ping_has_been_routed(bool success, packet_t* packet, void* data, radio_t *radio)
{
    int slot = (int) slot;

    linklayer_lock();

    if (success) {
        /* Create a timer to retry trasmission for a while */
        ping_table[slot].retries = CONFIG_LASTLINK_PING_RETRIES;
        ping_table[slot].waiting = true;
        ping_table[slot].thread_id = os_create_thread(ping_retry_thread, "ping_retry", PING_RETRY_STACK_SIZE, 0, (void*) slot);
        ping_table[slot].routed = true;
        simpletimer_start(&ping_table[slot].pingtime, CONFIG_LASTLINK_PING_RETRY_TIMER * 2);

    } else if (ping_table[slot].queue != NULL) {
        /* No route */
        void *null = NULL;
        os_put_queue(ping_table[slot].queue, (os_queue_item_t) &null);
        ping_table[slot].error = LSE_NO_ROUTE;
    }

    linklayer_unlock();

    /* We are done with it */
    release_packet(packet);

    /* Allow packet to be sent if it can be */
    return success;
}

/*
 * Ping an address and return it's pathlist
 */
ls_error_t ping(int address, uint32_t *elapsed, int *pathlist, int pathlistlen)
{
    ls_error_t ret = 0;

    static int ping_sequence_number;

    packet_t *packet = ping_packet_create(address, ++ping_sequence_number);

    if (packet != NULL) {
        linklayer_lock();

        int slot = register_ping(packet);

        if (slot >= 0) {

            packet_set_routed_callback(packet, ping_has_been_routed, (void*) slot);

            (void) linklayer_route_packet(packet);

            linklayer_unlock();

            packet_t *reply = wait_for_ping_reply(slot);

            linklayer_lock();

            if (reply != NULL) {
                /* Pass information back to caller */
                int num_routes = (reply->length - PING_ROUTE_TABLE) / ADDRESS_LEN;

                for (int route = 0; pathlistlen > 0 && route < num_routes; ++route) {
                    pathlist[route] = get_uint_field(reply, PING_ROUTE_TABLE + ADDRESS_LEN * route, ADDRESS_LEN);
                    pathlistlen--;
                }

                ret = num_routes;

                *elapsed = simpletimer_elapsed(&ping_table[slot].pingtime);

                release_packet(reply);

            } else {
                ret = ping_table[slot].error;
            }

            release_ping_table_entry(slot);

            linklayer_unlock();

        } else {
            /* No ping table entry */
            release_packet(packet);
            ret = LSE_NO_MEM;
        }

    } else {
        ret = LSE_NO_MEM;
    }

    return ret;
}


/*
 * Finds a free socket and returns it, locked.
 */
static ls_socket_t *find_free_socket(void)
{
    ls_socket_t *socket = NULL;
    static int socket_index = -1;

    /* This locks all table searchs until we choose something */
    GLOBAL_SOCKET_LOCK();

    for (int index = 0; socket == NULL && index < ELEMENTS_OF(socket_table); ++index) {

        /* Rotate equitably through all socket table entries */
        socket_index = (socket_index + 1) % ELEMENTS_OF(socket_table);

        if (socket_table[socket_index].socket_type == LS_UNUSED) {

            /* Zero the table entry */
            memset(&socket_table[socket_index], 0, sizeof(socket_table[socket_index]));

            /* Give it a lock */
            socket_table[socket_index].lock = os_create_recursive_mutex();

            socket = &socket_table[socket_index];
        }
    }


    if (socket != NULL) {
        socket_table[socket_index].socket_type = LS_INUSE;
    }

    GLOBAL_SOCKET_UNLOCK();

    if (socket != NULL) {
        LOCK_SOCKET(socket);
    }

    /* Return the locked socket if we found one */
    return socket;
}

static int next_src_port = 5000;

static int find_free_src_port(void) {

    int found = -1;

    GLOBAL_SOCKET_LOCK();

    bool collision;

    do {
        collision = false;

        /* Increment local port number pool */
        if (++next_src_port >= 65536) {
            next_src_port = 5000;
        }

        /* Go through all sockets to see if we have used this port */
        for (int s = 0; !collision && s < ELEMENTS_OF(socket_table); ++s) {
            if (next_src_port == socket_table[s].src_port) {
                collision = true;
            }
        }
    } while (collision);

    /* We will use this one */
    found = next_src_port;

    GLOBAL_SOCKET_UNLOCK();

    return found;
}


/*
 * Look in socket table for socket matching the connection and type.
 *
 * Returns LOCKED socket.
 */
static ls_socket_t *find_socket_from_packet(const packet_t *packet, ls_socket_type_t type)
{
    ls_socket_t *socket = NULL;

    ls_port_t dest_port = get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN);
    ls_port_t src_port  = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN);
    // int dest_addr       = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);

    for (int index = 0; socket == NULL && index < CONFIG_LASTLINK_NUMBER_OF_SOCKETS; ++index) {
        if (socket_table[index].socket_type == type) {
            // ESP_LOGI(TAG, "%s: socket type %d wanted %d", __func__, socket_table[index].socket_type, type);
            // ESP_LOGI(TAG, "%s: socket src_port %d dest_port %d", __func__, socket_table[index].src_port, dest_port);
            // ESP_LOGI(TAG, "%s: socket dest_port %d src_port %d socket dest_addr %d", __func__, socket_table[index].dest_port, src_port, socket_table[index].dest_addr);
            // ESP_LOGI(TAG, "%s: test 1 %d", __func__, (socket_table[index].src_port == dest_port));
            // ESP_LOGI(TAG, "%s: test 2 %d", __func__, (socket_table[index].dest_port == 0 || socket_table[index].dest_port == src_port || socket_table[index].dest_addr == BROADCAST_ADDRESS)); 

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
            if ( !(socket_table[index].state == LS_STATE_LISTENING) &&
#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
            if (
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
                // (socket_table[index].dest_addr == 0 || socket_table[index].dest_addr == BROADCAST_ADDRESS) &&
                (socket_table[index].src_port == dest_port) &&
                (socket_table[index].dest_port == 0 || socket_table[index].dest_port == src_port || socket_table[index].dest_addr == BROADCAST_ADDRESS)) {

                socket = &socket_table[index];
// ESP_LOGI(TAG, "%s: locking socket %p", __func__, socket);
                LOCK_SOCKET(socket);
// ESP_LOGI(TAG, "%s: found socket %p", __func__, socket);
            } else {
// ESP_LOGI(TAG, "%s: not found socket %p", __func__, &socket_table[index]);
            }
        }
    }

// ESP_LOGI(TAG, "%s: returning socket %p", __func__, socket);

    /* Return socket if we found it */
    return socket;
}

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
static ls_socket_t *find_listening_socket_from_packet(const packet_t *packet)
{
    ls_socket_t *socket = NULL;

    ls_port_t dest_port = get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN);

    for (int index = 0; socket == NULL && index < ELEMENTS_OF(socket_table); ++index) {
        /* A socket listening on the specified port is all we need */
        if (socket_table[index].socket_type == LS_STREAM && socket_table[index].state == LS_STATE_LISTENING && socket_table[index].src_port == dest_port) {
            socket = &socket_table[index];
        }
    }

    /* Return socket if we found it */
    return socket;
}
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

#ifdef NOTUSED
static packet_t *datagram_packet_create_from_packet(const packet_t *packet)
{
    packet_t *new_packet = linklayer_create_generic_packet(get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                                           DATAGRAM_PROTOCOL, DATAGRAM_LEN);

    if (new_packet != NULL) {
        set_uint_field(new_packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN, get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN));
        set_uint_field(new_packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN, get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN));
    }

    return new_packet;
}
#endif /* NOTUSED */

static packet_t *datagram_packet_create_from_socket(const ls_socket_t *socket)
{
    packet_t *packet = linklayer_create_generic_packet(socket->dest_addr, DATAGRAM_PROTOCOL, DATAGRAM_LEN);

    if (packet != NULL) {
        set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN, socket->dest_port);
        set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN, socket->src_port);
    }

    return packet;
}

static bool datagram_packet_process(packet_t *packet)
{
    bool handled = false;

    if (packet != NULL) {
        bool specifically_for_us = linklayer_packet_is_for_this_node(packet);

        if (specifically_for_us || get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN) == BROADCAST_ADDRESS) {

// ESP_LOGI(TAG, "%s: checking packet", __func__);

            ls_socket_t *socket = find_socket_from_packet(packet, LS_DATAGRAM);

// ESP_LOGI(TAG, "%s: socket is %p", __func__, socket);

            if (socket != NULL) {

// ls_dump_socket_ptr("handled", socket);

                /* Send the packet to the datagram input queue */
                ref_packet(packet);
                if (!os_put_queue(socket->datagram_packets, (os_queue_item_t) &packet)) {
                    release_packet(packet);
                }
                UNLOCK_SOCKET(socket);
            }

            /* say 'handled' it if directed to us (i.e. don't retransmit it) */
            handled = specifically_for_us;
        }

        release_packet(packet);
    }

    return handled;
}

static const char* datagram_packet_format(const packet_t *packet)
{
    char* info;

    const char* data = linklayer_escape_raw_data(packet->buffer + DATAGRAM_PAYLOAD, packet->length - DATAGRAM_PAYLOAD);

    asprintf(&info, "Datagram: Src Port %d Dest Port %d \"%s\"",
            get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN),
            get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN),
            data);

     free((void*) data);

     return info;
}

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
static packet_t *stream_packet_create_from_packet(const packet_t *packet, uint8_t flags, int sequence)
{
    packet_t *new_packet = linklayer_create_generic_packet(get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                                           STREAM_PROTOCOL, STREAM_LEN);

    if (new_packet != NULL) {
        set_uint_field(new_packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN, get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN));
        set_uint_field(new_packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN, get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN));
        set_uint_field(new_packet, STREAM_FLAGS, FLAGS_LEN, flags);
        set_uint_field(new_packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
    }

    return new_packet;
}

/*
 * Create a stream packet
 */
static packet_t *stream_packet_create_from_socket(const ls_socket_t *socket, uint8_t flags, int sequence)
{
    packet_t *packet = linklayer_create_generic_packet(socket->dest_addr, STREAM_PROTOCOL, STREAM_LEN);

    if (packet != NULL) {
        set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN, socket->dest_port);
        set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN, socket->src_port);
        set_uint_field(packet, STREAM_FLAGS, FLAGS_LEN, flags);
        set_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
//printf("%s: packet %p\n", __func__, packet);
    }

    return packet;
}

static void stream_control_packet_transmit_complete(packet_t *packet, void *data, radio_t *radio)
{
    ls_socket_t *socket = (ls_socket_t *) data;

    int cmd = get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN) & STREAM_FLAGS_CMD;

    /* Not used for data */
    if (cmd != STREAM_FLAGS_CMD_DATA) {
        if (LOCK_SOCKET(socket)) {
            if (packet != socket->packet_cache[cmd]) {
                /* Bugger! Not the same packet */
                printf("%s: cached and transmitted packets not the same:\n", __func__);
                linklayer_print_packet("Cached", socket->packet_cache[cmd]);
                linklayer_print_packet("Transmitted", packet);
            } else {
                release_packet(socket->packet_cache[cmd]);
                socket->packet_cache[cmd] = NULL;
            }
            UNLOCK_SOCKET(socket);
        }
    }

    release_packet(packet);
}

/*
 * create a stream packet from any cached copy of that command type.
 * If not exists, create one from whole cloth.
 * If reusing a packet, return with the transmit queue locked and set user "*locked" to true
 */
static packet_t *stream_packet_create_from_socket_cached(ls_socket_t *socket, uint8_t flags, int sequence)
{
    int cmd = flags & STREAM_FLAGS_CMD;

    LOCK_SOCKET(socket);

    if (socket->packet_cache[cmd] == NULL || ! packet_lock(socket->packet_cache[cmd])) {

        /* Either it didn't exist or we couldn't lock it - create a new one */
        if (socket->packet_cache[cmd] != NULL) {
            /* In queue still so release it */
            release_packet(socket->packet_cache[cmd]);
        }

        /* Create a new one */
        socket->packet_cache[cmd] = stream_packet_create_from_socket(socket, flags, sequence);
        
        packet_set_transmitted_callback(socket->packet_cache[flags & STREAM_FLAGS_CMD], stream_control_packet_transmit_complete, (void*) socket);
    } else {
        /* Add flags and sequence to this packet */
        set_uint_field(socket->packet_cache[cmd], STREAM_FLAGS, FLAGS_LEN, flags);
        set_uint_field(socket->packet_cache[cmd], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
    }

    /*
     * If reusing a packet, it is now locked and won't be transmitted until unlocked.
     * Do this quicly because it doesn't stop the transmit processes.  As long as a
     * packet on the transmit queue is locked, it will be recycled to the end of the
     * queue each time it pops to the top.  If the queue is otherwise empty, this will
     * occur on every CAD interrupt cycle (every few milliseconds.)
     */

    UNLOCK_SOCKET(socket);

    return ref_packet(socket->packet_cache[cmd]);
}
  
static void stream_shutdown_windows(ls_socket_t *socket)
{
    packet_window_shutdown_window(socket->input_window);
    packet_window_shutdown_window(socket->output_window);
}

static void stream_packet_transmit_complete(packet_t *packet, void *data, radio_t *radio)
{
    ls_socket_t *socket = (ls_socket_t*) data;

    /* Packet has been launched - set retry time */
    simpletimer_start(&socket->output_window_timer, socket->output_retry_time);

    /* Next time wait twice as long (up to a limit) */
    socket->output_retry_time <<= 1;
    if (socket->output_retry_time > CONFIG_LASTLINK_STREAM_TRANSMIT_MAXIMUM_RETRY_TIME) {
        socket->output_retry_time = CONFIG_LASTLINK_STREAM_TRANSMIT_MAXIMUM_RETRY_TIME;
    }

    /* Do the control queue stuff (also releases packet) */
    stream_control_packet_transmit_complete(packet, socket, radio);
}

#if CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLE
/*
 * This should only happen if there is no inbound traffic on socket */
static void stream_send_keepalive(ls_socket_t *socket)
{
    packet_t *packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_NOP, 0);

    /* Add sequence info to packet just because we can */
    add_data_ack(socket->input_window, packet);

    /* There is one ref for the cache and one for us; May be more for on-radio queue holdings */
    assert(packet->ref >= 2);

    linklayer_route_packet(packet);

    packet_unlock(packet);
}
#endif /* CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLE */

static void stream_send_connect_ack(ls_socket_t *socket)
{
    packet_t * packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_CONNECT_ACK, 0);
    linklayer_route_packet(packet); 
    packet_unlock(packet);
}

#ifdef NOTUSED
static void stream_send_disconnect(ls_socket_t *socket)
{
    packet_t *packet =stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_DISCONNECT, 0);
    linklayer_route_packet(packet); 
    packet_unlock(packet);
}
#endif /* NOTUSED */

static void stream_send_disconnected(ls_socket_t *socket)
{
    packet_t * packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_DISCONNECTED, 0);
    linklayer_route_packet(packet); 
    packet_unlock(packet);
}

static bool stream_packet_process(packet_t *packet)
{
    bool handled = false;

    bool reject = true;

    if (packet != NULL) {
//linklayer_print_packet("STREAM IN ", packet);
        if (linklayer_packet_is_for_this_node(packet)) {
            /* Read the flags field for later use */
            uint8_t flags = get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN);

            /* Get the socket for this packet if there is one */
            ls_socket_t *socket = find_socket_from_packet(packet, LS_STREAM);

            int cmd = flags & STREAM_FLAGS_CMD;

            switch (cmd) {
                default: {
                    linklayer_print_packet("BAD STREAM", packet);
                    /* Ignore noise */
                    reject = false;
                    break;
                }

                case STREAM_FLAGS_CMD_NOP: {
                    if (socket != NULL) {
                       reject = false;
                    }
                    break;
                }

                case STREAM_FLAGS_CMD_DATA: {
                    /*
                     * Pass the data through the assembly phase.  This might require modifying the current outbound packet
                     * to update ack sequence numbers, etc.  In some cases a brand new packet will be generated.
                     */
                    if (socket != NULL) {
                        /* Attempt to insert packet into window (0=ok 1=duplicate -1=error-out of sequence) */
                        int results = packet_window_add_random_packet(socket->input_window, packet);

                        /* If accepted or queue full, restart timer */
                        if (results <= 0) {
                            /* Restart ack delay timer - immediately if queue was full */
                            simpletimer_start(&socket->ack_delay_timer, results < 0 ? 0 : CONFIG_LASTLINK_STREAM_ACK_DELAY);
                        }

                        reject = false;
                    }
                    break;
                }

                case STREAM_FLAGS_CMD_CONNECT: {
                    /* First see if this is a connect on a socket that is already connecting */
                    if (socket != NULL) {
                        /* Cancel current state machine retry */
                        state_machine_cancel(socket);
                        if (socket->state == LS_STATE_INBOUND_CONNECT) {
                            /* Redundant CONNECT gets a CONNECT_ACK */
                            stream_send_connect_ack(socket);
                            reject = false;
                        }
                    } else {
                        /* Didn't find actual socket, so see if this is a new listen */
                        ls_socket_t *listen_socket = find_listening_socket_from_packet(packet);

                        if (listen_socket != NULL) {
                            ls_socket_t *new_connection = validate_socket(ls_socket(LS_STREAM));

                            if (new_connection != NULL) {

                                /* Create a new connection that achievs a local endpoint for the new socket */
                                new_connection->state         = LS_STATE_INBOUND_CONNECT;
                                new_connection->src_port      = listen_socket->src_port;
                                new_connection->dest_port     = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN);
                                new_connection->dest_addr     = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
                                new_connection->parent        = listen_socket;
                                new_connection->input_window  = packet_window_create(CONFIG_LASTLINK_STREAM_WINDOW_SIZE);
                                new_connection->output_window = packet_window_create(CONFIG_LASTLINK_STREAM_WINDOW_SIZE);

                                /* Send a CONNECT ACK and go to INBOUND CONNECT state waiting for full acks */
                                state_machine_start("new connection",
                                                    new_connection,
                                                    LS_STATE_INBOUND_CONNECT,               /* Pending inbound connect */
                                                    state_machine_action_send_connect_ack,  /* Send connect ack */
                                                    STATE_MACHINE_RESULTS_TO_QUEUE,         /* When we receive connectack, send to new_connection */
                                                    -1,                                     /* Use action result to set timeout */
                                                    STREAM_CONNECT_RETRIES);

                                UNLOCK_SOCKET(new_connection);

                                reject = false;
                            }
                        }
                    }

                    break;
                }

                case STREAM_FLAGS_CMD_CONNECT_ACK: {
                    /* If are in connecting, respond with connect ack */
                    if (socket != NULL) {

                        if (socket->state == LS_STATE_OUTBOUND_CONNECT) {

                            if (socket->input_window == NULL) {
                                /* Finish building socket by adding queues */
                                socket->input_window = packet_window_create(CONFIG_LASTLINK_STREAM_WINDOW_SIZE);
                            }
                            if (socket->output_window == NULL) {
                                socket->output_window = packet_window_create(CONFIG_LASTLINK_STREAM_WINDOW_SIZE);
                            }

                            /* Receiving a CONNECT ACK in the OUTBOUND connect, so send a CONNECT ACK and go the CONNECTED state */
                            stream_send_connect_ack(socket);
                            socket->state = LS_STATE_CONNECTED;

                            /* Deliver success to caller of original connect */
                            state_machine_send_results(socket, LSE_NO_ERROR);
                            reject = false;

                        } else if (socket->state == LS_STATE_INBOUND_CONNECT) {
                            socket->state = LS_STATE_CONNECTED;
                            /* Send the socket number to the waiting listener */
                            if (socket->parent != NULL) {
                                /* Put in the connection queue but if overflowed, reject the connection */
                                if(os_put_queue_with_timeout(socket->parent->connections, (os_queue_item_t) &socket, 0)) {
                                    reject = false;
                                    /* Let ls_listen finish it's work */
                                    state_machine_send_results(socket, LSE_NO_ERROR);
                                }
                            } else {
                                /* ERROR connection has no parent.  Just release the socket and reject connection attempt. */
                                release_socket(socket);
                            }

                        /* If an extra connect-ack in connected state, don't reject and send the expected connect ack */
                        } else if (socket->state == LS_STATE_CONNECTED) {
                            reject = false;
                        }

                        if (!reject && socket->output_window != NULL) {
                            int length = get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN);
                            if (length != 0 && length < socket->output_window->length) {
// ESP_LOGI(TAG, "%s: output_window changed from %d to %d for socket %d", __func__, socket->output_window->length, length, socket - socket_table);
                                socket->output_window->length = length;
                            }
                        }
                    }
                    break;
                }

                case STREAM_FLAGS_CMD_DISCONNECT: {
                    /* If still in connecting, respond with connect ack */
                    if (socket != NULL) {
                        if (socket->state == LS_STATE_CONNECTED) {

                            /* Start sending disconnect until we get a disconnected back */
                            state_machine_start("inbound disconnect",
                                                socket,
                                                LS_STATE_INBOUND_DISCONNECTING,          /* respond to inbound disconnect request */
                                                state_machine_action_send_disconnected,  /* Persist until complete or timeout */
                                                state_machine_results_shutdown_socket,   /* Action is to shutdown socket */
                                                -1,                                      /* Use action results to set timeout */
                                                STREAM_DISCONNECT_RETRIES);

                            /* This will shut down the reader when it rises to the top of the packet list */
                            packet_window_shutdown_window(socket->input_window);

                        } else if (socket->state == LS_STATE_OUTBOUND_DISCONNECTING) {
//printf("%s: OUTBOUND_DISCONNECTING received DISCONNECTED: sending to socket\n", __func__);
                            state_machine_send_results(socket, LSE_NO_ERROR);
                        } else if (socket->state == LS_STATE_DISCONNECTED) {
                            stream_shutdown_windows(socket);
                        } else {
                            stream_send_disconnected(socket);
                        }

                    } else {
                        /* Give single disconnect response if we don't have a socket for thisone */
                        (void) linklayer_route_packet(stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_DISCONNECTED, 0));
                    }

                    reject = false;

                    break;
                }

                case STREAM_FLAGS_CMD_DISCONNECTED: {
                    /* If are in connecting, respond with connect ack */
                    if (socket != NULL) {
                        if (socket->state == LS_STATE_INBOUND_DISCONNECTING || socket->state == LS_STATE_OUTBOUND_DISCONNECTING) {

                            if (socket->state == LS_STATE_OUTBOUND_DISCONNECTING) {
                                /* An answer to our disconnect request */
                                /* Stop the disconnect machine */
                                state_machine_send_results(socket, LSE_NO_ERROR);

                                /* Just echo back disconnected */
                                stream_send_disconnected(socket);

#if 0
                            } else if (socket->state == LS_STATE_INBOUND_DISCONNECTING) {
                                /* Final answer to our answer to an inbound disconnect */
                                /* Cancel our outbound acknowledgements */
                                state_machine_cancel(socket);
#endif
                            } else if (socket->state == LS_STATE_DISCONNECTED) {
                                /* A redundant answer to our outbound acknowledgement - other end didn't hear our reply */
                                /* Just send again until socket is done */
                                stream_send_disconnected(socket);
                            }

                            socket->state = LS_STATE_DISCONNECTED;
                        }
                    }
                    reject = false;
                    break;
                }

                case STREAM_FLAGS_CMD_REJECT: {
                    /* If are in connecting, respond with connect ack */
//ls_dump_socket_ptr("REJECTED", socket);
                    if (socket != NULL) {

                        switch (socket->state) {
                            case LS_STATE_OUTBOUND_CONNECT: {
                                 state_machine_send_results(socket, LSE_CONNECT_REJECTED);
                                 break;
                            }
#if 0
                            case LS_STATE_DISCONNECT: {
                                 release_socket(socket);
                                 break;
                            }
#endif
                            default: {
                                /* An error so tear down the connection */
                                stream_shutdown_windows(socket);
                                state_machine_start("closing",
                                                    socket,
                                                    LS_STATE_CLOSING,                       /* Final closing state */
                                                    state_machine_action_send_disconnect,   /* Send disconnect until done */
                                                    state_machine_results_shutdown_socket,  /* Finally shutdown socket */
                                                    -1,                                     /* Use action results to set timeout */
                                                    STREAM_DISCONNECT_RETRIES);
                            }
                        }
                    }
                    reject = false;
                    break;
                }
            }

            if (reject) {
                //linklayer_print_packet("REJECTED", packet);
                /* Send a reject packet that overrides sequence numbers past seen */
                packet_t *reject_packet = stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_REJECT, 0);
                set_bits_field(reject_packet, HEADER_FLAGS, FLAGS_LEN, HEADER_FLAGS_RESET_SEQUENCE);
                (void) linklayer_route_packet(reject_packet);

            } else if (socket != NULL) {
                /* Ack the output window if acknum is present */
                if  ((flags & STREAM_FLAGS_ACKNUM) != 0) {
                    int sequence      = get_uint_field(packet, STREAM_ACK_SEQUENCE, SEQUENCE_NUMBER_LEN);
                    uint32_t ack_mask = get_uint_field(packet, STREAM_ACK_WINDOW, ACK_WINDOW_LEN);

//if (ack_mask != 0) {
//   printf("%s: socket %d ack sequence %d mask %04x\n", __func__, socket - socket_table, sequence, ack_mask);
//}
                    ack_output_window(socket, sequence, ack_mask);
                }
            }

            if (socket != NULL) {
                UNLOCK_SOCKET(socket);
            }

            handled = true;
        }

        release_packet(packet);
    }

    return handled;
}

static const char* stream_packet_format(const packet_t *packet)
{
    char* info;

    const char* data = linklayer_escape_raw_data(packet->buffer + STREAM_PAYLOAD, packet->length - STREAM_PAYLOAD);

    asprintf(&info, "Stream: Src Port %d Dest Port %d Ack %d Window %02x Seq %d Flags %02x \"%s\"",
            get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN),
            get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN),
            get_uint_field(packet, STREAM_ACK_SEQUENCE, SEQUENCE_NUMBER_LEN),
            get_uint_field(packet, STREAM_ACK_WINDOW, ACK_WINDOW_LEN),
            get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN),
            get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN),
            data);

     free((void*) data);

     return info;
}
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */


static ls_error_t release_socket(ls_socket_t* socket)
{
    ls_error_t err = LSE_NO_ERROR;

    LOCK_SOCKET(socket);

    switch (socket->socket_type) {
        default:
        case LS_UNUSED: {
            err = LSE_NOT_OPENED;
            break;
        }

        case LS_INUSE: {
            socket->socket_type = LS_UNUSED;
            break;
        }

        case LS_DATAGRAM: {
            /* Close packet receive queue */
            linklayer_release_packets_in_queue(socket->datagram_packets);
            if (socket->datagram_packets != NULL) {
                os_delete_queue(socket->datagram_packets);
                socket->datagram_packets = NULL;
            }
            socket->socket_type = LS_UNUSED;
            break;
        }

        case LS_STREAM: {
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
            if (socket->state == LS_STATE_LISTENING) {
                /* Go through all sockets and bust the parent link to this listener */
                for (int socknum = 0; socknum < ELEMENTS_OF(socket_table); ++socknum) {
                    if (socket_table[socknum].socket_type == LS_STREAM && socket_table[socknum].parent == socket) {
                        socket_table[socknum].parent = NULL;
                    }
                }

                /* Close listener queue */
                ls_socket_t *c;
                while (os_get_queue_with_timeout(socket->connections, (os_queue_item_t) &c, 0)) {
                    ls_close_ptr(c);
                }

                os_delete_queue(socket->connections);
                socket->connections = NULL;

            } else {
                simpletimer_stop(&socket->state_machine_timer);
                simpletimer_stop(&socket->socket_flush_timer);
                simpletimer_stop(&socket->output_window_timer);
                simpletimer_stop(&socket->ack_delay_timer);
                simpletimer_stop(&socket->input_window_timer);
#if CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLED
                simpletimer_stop(&socket->keepalive_timer);
#endif /* CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLED */

                /* Otherwise forcibly shut everything down */
                if (socket->input_window != NULL) {
                    packet_window_release(socket->input_window);
                    socket->input_window = NULL;
                }
                if (socket->output_window != NULL) {
                    packet_window_release(socket->output_window);
                    socket->output_window = NULL;
                }

                if (socket->state_machine_results_queue != NULL) {
                    os_delete_queue(socket->state_machine_results_queue);
                    socket->state_machine_results_queue = NULL;
                }

                if (socket->current_read_packet != NULL) {
                    release_packet(socket->current_read_packet);
                    socket->current_read_packet = NULL;
                }

                if (socket->current_write_packet != NULL) {
                    release_packet(socket->current_write_packet);
                    socket->current_write_packet = NULL;
                }
            }

            socket->socket_type = LS_UNUSED;

#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

            err = LSE_NOT_IMPLEMENTED;

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
            break;
        }
    }

    socket->state = LS_STATE_IDLE;

    UNLOCK_SOCKET(socket);

    return err;
}

/*
 * Create a socket for sending or listening (determined by ls_connect or ls_listen)
 *
 * Entry:
 *      socket_type         LS_DATAGRAM or LS_STREAM
 *
 * Returns:
 *      int                 if < 0 an error, otherwise socket number.
 */
int ls_socket(ls_socket_type_t socket_type)
{
    ls_error_t  ret;

    /* Validate parameters */
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
    if (socket_type == LS_DATAGRAM || socket_type == LS_STREAM) {
#else /* ! CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
    if (socket_type == LS_STREAM) {
        ret = LSE_NOT_IMPLEMENTED;
    } else if (socket_type == LS_DATAGRAM) {
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

        ls_socket_t *socket = find_free_socket();

        if (socket != NULL) {
            socket->socket_type = socket_type;
            socket->state = LS_STATE_SOCKET;
            socket->src_port = -1;

            /* Return the socket index in the table plus the offset */
            ret = socket - socket_table + FIRST_LASTLINK_FD;

            UNLOCK_SOCKET(socket);

        } else {
            ret = LSE_NO_MEM;
        }

    } else {
        ret = LSE_BAD_TYPE;
    }

    return ret;
}

/*
 * ls_bind
 *
 * Bind an port to the local port.
 */
ls_error_t ls_bind(int s, ls_port_t src_port)
{
    ls_error_t err = LSE_INVALID_SOCKET;

    ls_socket_t *socket = validate_socket(s);

    /* Must be a fresh socket */
    if (socket != NULL) {
        if (socket->state == LS_STATE_SOCKET) {

            socket->state = LS_STATE_BOUND;

            /* Assign local port from free pool if not specified by user */
            socket->src_port = src_port;

            err = LSE_NO_ERROR;
        }

        UNLOCK_SOCKET(socket);
    }

    return err;
}

/*
 * Listen for connections on this socket.
 *
 * Entry:
 *      s                   The socket to listen on
 *      max_queue           Maximum number of simultaneous connections allowed
 *      timeout             How many mS to wait for a connection (0 is infinite)
 *
 * Returns:
 *      socket number if >= 0 else ls_error_t
 *
 */
ls_error_t ls_listen(int s, int max_queue, int timeout)
{
    ls_error_t err;

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
    ls_socket_t *socket = validate_socket(s);

    /* Validate paramters */
    if (socket != NULL) {

        bool release = true;

        if (socket->busy != 0) {
            err = LSE_SOCKET_BUSY;
        } else {
            socket->busy++;

            /* Has to be a stream socket in freshly-created mode */
            if (socket->socket_type == LS_STREAM && (socket->state == LS_STATE_BOUND || socket->state == LS_STATE_LISTENING)) {

                if (max_queue > 0) {
                    if (socket->connections == NULL) {
                        /* Create queue for connections */
                        socket->connections = os_create_queue(max_queue, sizeof(ls_socket_t*));
                        /* Place socket in listen mode */
                        socket->state = LS_STATE_LISTENING;
                    }

                    release = false;

                    UNLOCK_SOCKET(socket);

                    /* Wait for something to arrive at connection queue */
                    ls_socket_t *connection;
                    if (os_get_queue_with_timeout(socket->connections, (os_queue_item_t) &connection, timeout)) {
                        if (connection == NULL) {
                            err = LSE_CLOSED;
                        } else {
                            /* Wait for the connect ack to finish */
                            err = state_machine_get_results(connection);

                            /* A new connection */
                            err = connection - socket_table + FIRST_LASTLINK_FD;
                        }
                    } else {
                        err = LSE_TIMEOUT;
                    }
                } else {
                    err = LSE_INVALID_MAXQUEUE;
                }
            } else {
                ESP_LOGI(TAG, "%s: type %s state %s", __func__, socket_type_of(socket), socket_state_of(socket));
                err = LSE_INVALID_SOCKET;
            }

            LOCK_SOCKET(socket);
            socket->busy--;
            UNLOCK_SOCKET(socket);
        }

        if (release) {
            UNLOCK_SOCKET(socket);
        }

    } else {
        err = LSE_INVALID_SOCKET;
    }

#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

    err = LSE_NOT_IMPLEMENTED;

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

    return err;
}

/*********************************************************************************************************
 * State machine functions.
 *********************************************************************************************************/
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS

/*
 * state_machine_start
 *
 * Used for connecting, disconnecting and outbound data transmission.
 *
 * Entry:
 *      ident        Identifier string for debug
 *      socket       Socket to run state machine
 *      state        State to assign socket
 *      action       Action to take
 *      results      Function to receive results (NULL for response queue)
 *      timeout      Timeout between retry cycles. (if <0, uses results of action to set timeout)
 *      retries      Number of times to retry packet until state changes.
 */
static void state_machine_start(const char *ident, ls_socket_t *socket, ls_socket_state_t state, int (*action)(ls_socket_t* socket), void (*results)(ls_socket_t* socket, ls_error_t error), int timeout, int retries)
{
    assert(action != NULL);

    LOCK_SOCKET(socket);

    /* Cancel any running state machine */
    state_machine_cancel(socket);

    socket->state_machine_ident = ident;

    if (state >= 0) {
        socket->state = state;
    }

    socket->state_machine_results_callback = results;

    if (results == NULL) {
        /* Results goes to response queue */
        if (socket->state_machine_results_queue != NULL) {
            os_delete_queue(socket->state_machine_results_queue);
        }
        socket->state_machine_results_queue = os_create_queue(1, sizeof(ls_error_t));
    }

    socket->state_machine_action_callback = action;
    socket->state_machine_retries = retries + 1;

    /* Start immediately */
    if (timeout > 0) {
        simpletimer_start(&socket->state_machine_timer, timeout);
    } else {
        /* Wait for route failure or transmit complete */
    }

    /* Fire it immediately so we start the bubble machine */
    simpletimer_set_expired(&socket->state_machine_timer);

    socket->state_machine_running = true;

    UNLOCK_SOCKET(socket);
}

static void state_machine_cancel(ls_socket_t *socket)
{
    simpletimer_stop(&socket->state_machine_timer);

    socket->state_machine_retries = 0;
    socket->state_machine_action_callback = NULL;
    socket->state_machine_running = false;

    /* Leave response queue in place so receiver gets the message */
}



/*
 * state_machine_timeout
 *
 * Called by repeating timer to keep machine running - retransmit data, etc.
 *
 * Entry:
 *      socket      Socket being controller.
 */
static void state_machine_timeout(ls_socket_t *socket)
{
    if (--(socket->state_machine_retries) > 0) {

        /* Timed out so retry by sending connect packet again (if still present) */
        assert(socket->state_machine_action_callback != NULL);

        if (socket->state_machine_action_callback != NULL) {
            int action_results = socket->state_machine_action_callback(socket);

            switch (action_results) {
                case STATE_MACHINE_ACTION_NONE: {
                    /* Do nothing */
                    break;
                }
                case STATE_MACHINE_ACTION_PAUSE: {
                    /* Stop timer and wait for external action */
                    simpletimer_stop(&socket->state_machine_timer);
                    break;
                }
                case STATE_MACHINE_ACTION_SUCCESS: {
                    /* Send "ok" */
                    state_machine_send_results(socket, LSE_NO_ERROR);
                    break;
                }
                case STATE_MACHINE_ACTION_RETRY: {
                    /* Restart timer from current interval */
                    simpletimer_restart(&socket->state_machine_timer);
                    break;
                }
                default: {
                    if (action_results < 0) {
                        /* A specific failure code */
                        state_machine_send_results(socket, action_results);
                    } else {
                        /* Start timer with new interval */
                        simpletimer_start(&socket->state_machine_timer, action_results);
                    }
                    break;
                }
            }
        }
    } else {

        /* Send error code to connect response queue */
        state_machine_send_results(socket, LSE_TIMEOUT);

        /* Print a second callback  */
        socket->state_machine_running = false;
    }
}

/*
 * Send state machine results back to caller.
 *
 * If the state_machine_results_callback member is NULL, then we send it to
 * a (required) response queue for caller.
 *
 * If the state_machine_results_callback is not NULL, it is called with the
 * socket and results code.
 */
void state_machine_send_results(ls_socket_t *socket, ls_error_t response)
{
    if (! socket->state_machine_running) {
        printf("%s: state machine cancelled before call\n", __func__);
    } else {
        state_machine_cancel(socket);

        if (socket->state_machine_results_callback == NULL) {

if (socket->state_machine_results_queue == NULL) {
    ls_dump_socket_ptr("queue is null", socket);
    printf("state_machine_running %s  state_machine_ident \"%s\"  socket state %s\n", socket->state_machine_running ? "YES" : "NO", socket->state_machine_ident, socket_state_of(socket));
}
            assert(socket->state_machine_results_queue != NULL);
            os_put_queue(socket->state_machine_results_queue, (os_queue_item_t) &response);
        } else {
            socket->state_machine_results_callback(socket, response);
        }
    }
}

/*
 * Used by entity starting state machine to pend waiting for
 * the results of the operation.  Not used for functional
 * finishing of the state machine action.
 */
ls_error_t state_machine_get_results(ls_socket_t *socket)
{
    ls_error_t response;

    assert(socket->state_machine_results_callback == NULL);
    assert(socket->state_machine_results_queue != NULL);

    if (socket->state_machine_results_queue != NULL) {
        if (! os_get_queue(socket->state_machine_results_queue, (os_queue_item_t) &response)) {
            response = LSE_SYSTEM_ERROR;
        }

        /* Remove the queue */
        os_delete_queue(socket->state_machine_results_queue);
        socket->state_machine_results_queue = NULL;

    } else {
        response = LSE_SYSTEM_ERROR;
    }

    return response;
}

/*
 * state_machine_packet_route_complete
 *
 * Called when the routing of a packet has been completed - just prior to transmission.
 *
 * Entry:
 *    success            true if routing successful.
 *    packet             the packing being send
 *    data               user-supplied agnostic data
 *    radio              the radio through which packet was routed (NULL if not routed)
 *
 * Return true if the packet should be transmitted; false if to discard.
 */
static bool state_machine_packet_route_complete(bool success, packet_t *packet, void *data, radio_t *radio)
{
    ls_socket_t *socket = (ls_socket_t*) data;

    if (! success) {
        /* Failed route - shutdown socket */
        linklayer_print_packet("ROUTE FAILED", packet);
        stream_shutdown_windows(socket);

        /* Start shutdown process */
        state_machine_start("outbound disconnect 1",
                            socket,
                            LS_STATE_OUTBOUND_DISCONNECTING,           /* Route failed so start disconnect */
                            state_machine_action_send_disconnect,      /* Send disconnect until complete */
                            state_machine_results_shutdown_socket,     /* Finally shut down socket */
                            -1,                                        /* Use action result to set timeout */
                            STREAM_DISCONNECT_RETRIES);
    }

    release_packet(packet);

    return success;
}

/*
 * Look up the metric to the node and calculate an appropriate delay before retry.
 */
static int calculate_retry_time(ls_socket_t *socket, int message_time)
{
    int metric      = route_metric(socket->dest_addr);
    int retry_time  = (CONFIG_LASTLINK_STREAM_TRANSMIT_RETRY_FACTOR * message_time * (metric * 4 - 2)) / 100 +
                      CONFIG_LASTLINK_STREAM_TRANSMIT_OVERHEAD_TIME;

//printf("%s: packets message_time %d adjusted by metric %d is %d\n", __func__, message_time, metric, retry_time);

    return retry_time;
}

/*
 * state_machine_packet_transmit_complete
 *
 * Packet has been sent.  This callback sets the state machine timeout for
 * a retry of the packet.
 *
 * Entry:
 *      packet         Packet transmitted
 *      data           User supplied data
 *      radio          Radio through which packet was transmitted.
 */
static void state_machine_packet_transmit_complete(packet_t *packet, void *data, radio_t *radio)
{
    ls_socket_t *socket = (ls_socket_t*) data;

    int message_time;

    /* Packet has been launched - set retry time */
    if (radio != NULL) {

        /* Message time for the transmission */
        message_time = radio->get_message_time(radio, packet->length);

        /* Based on distance, calculate expected reply delay for a retry. */
        message_time = calculate_retry_time(socket, message_time);

//printf("%s: timeout for sequence %d set to  %d mS\n", __func__, get_uint_field(packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN), message_time);

    } else {
        /* Activity is happening so use small timeout to avoid tight looping */
        message_time = 10;
    }

    /* Set the state machine timeout */
    simpletimer_start(&socket->state_machine_timer, message_time);

    release_packet(packet);
}

/*
 * state_machine_send_add_callback
 *
 * Sets the callbacks for the packet and sends it on it's way.
 *
 * Returns -1 to tell caller that timeout has not been set.
 */
static int state_machine_send_add_callback(ls_socket_t *socket, packet_t *packet)
{
    assert(packet != NULL);

    /* Callback to detect route failure */
    packet_set_routed_callback(packet, state_machine_packet_route_complete, (void*) socket);

    /* Callback to set retry time after transmission (this overrides the control_packet transmitted callback */
    packet_set_transmitted_callback(packet, state_machine_packet_transmit_complete, (void*) socket);

    int action;

    /* Send packet to radio queue */
    if (linklayer_route_packet(packet) == 0) {
        /* Queued for transfer so stop and wait */
        action = STATE_MACHINE_ACTION_PAUSE;
    } else {
        /* We've already received the transmit callback so do nothing now */
        action = STATE_MACHINE_ACTION_NONE;
    }

    /* If we had the packet locked, unlock it now */
    packet_unlock(packet);

    /* Message time will be set after transmission (or in some cases before we get back) */
    return action;
}

/*
 * State machine state functions.
 *
 * These functions take a socket as a parameter and send a specific message or do some
 * specific function.  They return a 'time to wait' for state completion.
 * If the return value is < 0, the wait is indefinite as the caller will either
 * call back with it's own delay function or will set the state machine timer by some
 * other callback at a later time.
 */
static int state_machine_action_send_connect(ls_socket_t *socket)
{
    packet_t *packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_CONNECT, 0);
    return state_machine_send_add_callback(socket, packet);
}

static int state_machine_action_send_connect_ack(ls_socket_t *socket)
{
    packet_t *packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_CONNECT_ACK, socket->input_window->length);
    return state_machine_send_add_callback(socket, packet);
}

static int state_machine_action_send_disconnect(ls_socket_t *socket)
{
    packet_t *packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_DISCONNECT, 0);
    return state_machine_send_add_callback(socket, packet);
}

static int state_machine_action_send_disconnected(ls_socket_t *socket)
{
    packet_t *packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_DISCONNECTED, 0);
    return state_machine_send_add_callback(socket, packet);
}

/*
 * state_machine_send_output_window
 *
 * Launches send_output_window if not already running and
 * monitors it's progress.
 */
static int state_machine_action_send_output_window(ls_socket_t *socket)
{
    bool results;

    if (socket->state == LS_STATE_DISCONNECTING_FLUSH_START) {
        /*
         * Launch the send_output_window and monitor on each callback from state machine.
         * We run with disabled disconnect so it only tries to flush packets.
         */
        results = send_output_window(socket, /* disconnect_on_error */ false);

        socket->state = LS_STATE_DISCONNECTING_FLUSHING;
    } else {

        /* Timed out - did we complete our task? */
        int packets = packet_window_packet_count(socket->output_window);
//printf("%s: packets remaining %d\n", __func__, packets);

        /* Return true when no more packets */
        results = packets == 0;
    }

    return results ? STATE_MACHINE_ACTION_SUCCESS : STATE_MACHINE_ACTION_RETRY;
}

/*
 * Check for socket shutdown
 */
static int state_machine_action_linger_check(ls_socket_t* socket)
{
//printf("%s: socket %d\n", __func__, socket - socket_table);
    bool results = socket->socket_type == LS_STREAM && socket->state == LS_STATE_LINGER;
    return results ? -1 : simpletimer_interval(&socket->state_machine_timer);
}

/*
 * Put socket back to ground state
 */
static void state_machine_results_linger_finish(ls_socket_t* socket, ls_error_t error)
{
    if (socket->socket_type == LS_STREAM) {
        (void) release_socket(socket);
    }
}

static void state_machine_results_shutdown_socket(ls_socket_t *socket, ls_error_t err)
{
    if (packet_window_user_is_blocked(socket->input_window)) {
        /* Input blocked, so trigger input to release user */
        packet_window_trigger_available(socket->input_window);

    } else if (packet_window_user_is_blocked(socket->output_window)) {
        /* Output blocked, so trigger output to release user */
        packet_window_trigger_room(socket->input_window);

    } else {
        /* Just release the socket - user will have to find out the hard way */
        release_socket(socket);
    }
}

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
/*********************************************************************************************************
 * End of state machine functions.
 *********************************************************************************************************/

/*
 * Connect to a remote port.
 *
 * Entry:
 *      s                   Socket from which to make the connection
 *      address             Target address of node
 *      port                Target port on node
 *
 * Returns:
 *      ls_error_t          LSE_NO_ERROR (0) if successful otherwise error code.
 */
ls_error_t ls_connect(int s, ls_address_t address, ls_port_t port)
{
    ls_error_t ret = LSE_NO_ERROR;

    ls_socket_t *socket = validate_socket(s);

    if (socket != NULL && socket->state == LS_STATE_BOUND) {

        if (socket->busy != 0) {
            ret = LSE_SOCKET_BUSY;
        } else {
            socket->busy++;
            socket->dest_port = port;
            socket->dest_addr = address;

            /* If src_port is 0, bind an unused local port to this socket */
            if (socket->src_port == 0) {
                socket->src_port = find_free_src_port();
            }

            if (socket->socket_type == LS_DATAGRAM) {
                /* Datagram connection is easy - we just say it's connected */
                socket->state = LS_STATE_CONNECTED;
                if (socket->datagram_packets == NULL) {
                    socket->datagram_packets = os_create_queue(5, sizeof(packet_t*));
                }

#ifdef NOTUSED
                /* Special case: our dest address changes to the last destination of the packet we receive.  */
                socket->rename_dest = address == NULL_ADDRESS;
#endif

            } else if (socket->socket_type == LS_STREAM) {
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
                /* Start sending connect packet and wait for complete */
                state_machine_start("outbound connect",
                                    socket,
                                    LS_STATE_OUTBOUND_CONNECT,              /* Pending outbound connect */
                                    state_machine_action_send_connect,      /* Send connect until reply */
                                    STATE_MACHINE_RESULTS_TO_QUEUE,         /* Response comes back to response queue */
                                    -1,                                     /* Use action time to calculate delay */
                                    STREAM_CONNECT_RETRIES);                /* Try this many times */


                UNLOCK_SOCKET(socket);

                /* A response of some kind will be forthcoming.  It will be NO_ERROR or some other network error */
                ret = state_machine_get_results(socket);

//printf("%s: outbound received %d\n", __func__, ret);

                LOCK_SOCKET(socket);

#if CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLE
                if (ret == LSE_NO_ERROR) {
                    /* keepalive time is expressed in seconds */
                    simpletimer_start(&socket->keepalive_timer, CONFIG_LASTLINK_STREAM_KEEP_ALIVE_TIME * 1000);
                }
#endif
#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

                ret = LSE_NOT_IMPLEMENTED;

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

            } else {
                ret = LSE_INVALID_SOCKET;
            }

            socket->busy--;
        }

        UNLOCK_SOCKET(socket);

    } else {
        ret = LSE_INVALID_SOCKET;
    }

    if (ret != LSE_INVALID_SOCKET) {
        socket->last_error = ret;
    }

    return ret;
}

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
/*
 * Flush the current output packet if one exists.
 *
 * Start a timer to send the otuput window after a delay.
 *
 * Return true if work remains to be done.
 */
static bool socket_flush_current_packet(ls_socket_t *socket, bool wait)
{
    /* Kill the automatic flush timer */
    simpletimer_stop(&socket->socket_flush_timer);

    if (socket->output_window != NULL && socket->current_write_packet != NULL) {
        packet_t *packet = socket->current_write_packet;

        packet_window_lock(socket->output_window);

        /* Try a non-blocking insertion. */
        bool inserted = packet_window_add_sequential_packet(socket->output_window, packet, 0);

        packet_window_unlock(socket->output_window);

        if (inserted) {
            /* Success, so empty our output buffer */
            release_packet(socket->current_write_packet);
            socket->current_write_packet = NULL;
        } else if (wait) {

            if (socket->output_retries == 0) {
                /* Start bubble machine to transmit while we are waiting */
                simpletimer_start(&socket->output_window_timer, CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY);
            }

            /* Do a blocking move of packet to output window.  */
            if (packet_window_add_sequential_packet(socket->output_window, packet, -1)) {
                release_packet(socket->current_write_packet);
                socket->current_write_packet = NULL;
            }
        }
    }

    /* Let caller know if there are packets waiting to go out */
    return socket->current_write_packet != NULL || (socket->output_window && packet_window_packet_count(socket->output_window) != 0);
}


/*
 * Send any packets in the output window (unacknowledged) + the ACK sequence and window flags.
 *
 * This acknowledges up to but not including the ack_sequence number + any
 * additional packets beyond in the ack_mask.
 *
 * For example:
 *     ACK 0 0  Does not ack anything
 *     ACK 3 4  Acks up to packet sequence 2 plus packet sequence 6 (2 + log2(4) + 1)
 *
 * The LSB of the ack_mask represents the first packet W + 1.
 *
 * This allows acknowledging packets received that haven't been actually processed, in
 * order to avoid resending future, already recevied packets.  The above 3/4 implies
 * that packet 0 1 and 2 have been received correctly plus packet 6.
 */
static void ack_output_window(ls_socket_t *socket, int ack_sequence, uint32_t ack_mask)
{
    /* Release these packets in window */
    packet_window_release_processed_packets(socket->output_window, ack_sequence, ack_mask);

    /* Start again to send next block (if any) */
    socket->output_retries = 0;
    simpletimer_set_expired(&socket->output_window_timer);
}

/* Add a data acknowledgement to the outbound data message.
 * Returns the packet.
 */
static packet_t *add_data_ack(packet_window_t *window, packet_t *packet)
{
    uint32_t ack_mask;
    int sequence;

    if (packet_window_get_processed_packets(window, &sequence, &ack_mask)) {

//printf("%s: sequence %d mask %04x\n", __func__, sequence, ack_mask);

        set_uint_field(packet, STREAM_ACK_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
        set_uint_field(packet, STREAM_ACK_WINDOW, ACK_WINDOW_LEN, ack_mask);
        /* Indicate an ack sequence and window are in the payload */
        clear_bits_field(packet, STREAM_FLAGS, FLAGS_LEN, STREAM_FLAGS_BITS);
        set_bits_field(packet, STREAM_FLAGS, FLAGS_LEN, STREAM_FLAGS_ACKNUM);
    }

    return packet;
}

/*
 * ack_input_window
 *
 * If input isn't full - send ack to get more data.
 */
static void ack_input_window(ls_socket_t* socket)
{
    /* If input window is being read and it is not full, send an ack to let sender know there is room */
    if (packet_window_user_is_blocked(socket->input_window) && packet_window_packet_count(socket->input_window) != socket->input_window->length) {

        packet_t *packet;

#ifdef NOTUSED
        /* If output exists in output window, send first packet instead of ACK */
        packet_t *packets[socket->output_window->length];

        int num_packets = packet_window_get_all_packets(socket->output_window, packets, ELEMENTS_OF(packets));

        if (num_packets != 0) {
            packet = packets[0];
            for (int slot = 1; slot < num_packets; ++slot) {
                release_packet(packets[slot]);
            }
        } else {
            packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_NOP, 0);
        }
#endif
        packet = stream_packet_create_from_socket_cached(socket, STREAM_FLAGS_CMD_NOP, 0);

if (packet->ref <= 0) {
  char buf[20];
  sprintf(buf, "REF %d", packet->ref);
  linklayer_print_packet(buf, packet);
}
        assert(packet->ref >= 2);

        simpletimer_restart(&socket->input_window_timer);

        /* Add sequence info to packet */
        add_data_ack(socket->input_window, packet);

        /* Send it */
        (void) linklayer_route_packet(packet);

        /* Let if fly if it was locked */
        packet_unlock(packet);
    }
}

/*
 * Send current output window with ack fields.
 *
 * If the output window has packets, send them with the last packet
 * containing the ack information to previous input packets.
 *
 * Returns true if no packets remain in output.
 */
static bool send_output_window(ls_socket_t *socket, bool disconnect_on_error)
{
    int num_packets = 0;

//printf("%s: socket %d output_window %p\n", __func__, socket - socket_table, socket->output_window);

    if (socket->output_window != NULL) {
//printf("%s: locking packet window\n", __func__);
        packet_window_lock(socket->output_window);

//printf("%s: checking packets to go\n", __func__);

        packet_t *packets[socket->output_window->length];
        num_packets = packet_window_get_all_packets(socket->output_window, packets, ELEMENTS_OF(packets));

        if (num_packets != 0) {

//printf("%s: %d packets to go\n", __func__, num_packets);

            /* Stop the ack_delay_timer since this subsumes the effort */
            simpletimer_stop(&socket->ack_delay_timer);

            /* Don't need this because it's included in output_window data */
            simpletimer_stop(&socket->input_window_timer);

            /* Make sure last packet has been sent */
            if (packets[num_packets-1]->transmitting != 0) {
                /* Last packet remains to be sent - pause 100 mS to let that happen */
                simpletimer_start(&socket->output_window_timer, 100);
            } else {

                /* If first packet of output block or block has been updated, restart state timeout */
                if (socket->output_retries == 0 || socket->output_window->sequence != socket->output_last_sequence) {

                    socket->output_last_sequence = socket->output_window->sequence;

                    /* If first time, set new timeout and retry count */
                    socket->output_retries = CONFIG_LASTLINK_STREAM_TRANSMIT_RETRIES;

                    socket->output_disconnect_on_error = disconnect_on_error;

                } else {
                    socket->output_retries -= 1;
                }

                if (socket->output_retries != 0) {
                    int to_send = num_packets;

                    if (socket->output_retries != CONFIG_LASTLINK_STREAM_TRANSMIT_RETRIES) {
                        /* For retried, only do the first packet */
                        to_send = 1;
                    }

                    packet_window_unlock(socket->output_window);

                    int total_message_time = 0;

//printf("%s: putting %d packets to queue\n", __func__, to_send);

                    /* Packets in queue - send all that are here */
                    for (int slot = 0; slot < to_send; ++slot) {
                        /* Send packet (ref'd) with input window ack fields appended */
                        /* Note: we share the state_machine_packet_route_complete as it has equivalent semantics for this window output */
                        packet_t *packet  = packet_set_routed_callback(packets[slot], state_machine_packet_route_complete, (void*) socket);
                        /* Only do on the last one in list */
                        if (slot == to_send - 1) {
                            packet_set_transmitted_callback(packet, stream_packet_transmit_complete, (void*) socket);
                        } else {
                            /* Keep the window of packets sent as a group as much as possible */
                            set_bits_field(packet, HEADER_FLAGS, FLAGS_LEN, HEADER_FLAGS_PRIORITY);
                        }
//printf("%s: sending %d to linklayer_route_packet\n", __func__, slot);
                        total_message_time += linklayer_route_packet(ref_packet(add_data_ack(socket->input_window, packet)));
                    }

                    /* Look up the route to the destination (returns default of MAX_METRIC) */
                    socket->output_retry_time = calculate_retry_time(socket, total_message_time);

//printf("%s: %d packets %d mS total is %d mS\n", __func__, to_send, total_message_time, socket->output_retry_time);

                    packet_window_lock(socket->output_window);

                } else if (socket->output_disconnect_on_error) {
                    /* Timed out */
                    /* Terminate input */
//printf("%s: disconnect on error\n", __func__);
                    stream_shutdown_windows(socket);

                    /* Start shutdown process */
                    state_machine_start("outbound disconnect 3",
                                        socket,
                                        LS_STATE_OUTBOUND_DISCONNECTING,        /* Start a disconnect */
                                        state_machine_action_send_disconnect,   /* Persist until complete */
                                        state_machine_results_shutdown_socket,  /* Finally shut down socket */
                                        -1,                                     /* Use action time to set delay */
                                        STREAM_DISCONNECT_RETRIES);
                }
            }

            /* Release the copied packets */
            for (int slot = 0; slot < num_packets; ++slot) {
                release_packet(packets[slot]);
            }
        } else {
            socket->output_retries = 0;  /* Done */
        }

        packet_window_unlock(socket->output_window);
    }

    return num_packets == 0;
}
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

/*
 * Write to a datagram or stream socket.
 *
 * Datagrams are limited to one packet (MAX_DATAGRAM_SIZE) bytes.
 * Streams are limited by user and memory for buffers.
 *
 * Entry:
 *      socket              socket to write upon
 *      buf                 buffer to write
 *      len                 length to write
 *      eor                 Write END OF RECORD if true
 *      address             Address to use if port >= 0
 #      port                Port to use of port >= 0
 *
 * Returns:
 *      ls_error_t          If >0, number of bytes written (might be less than len if error)
 *                          Otherwise is error code (< 0)
 */
static ls_error_t ls_write_helper(ls_socket_t *socket, const char* buf, size_t len, bool eor, ls_address_t address, ls_port_t port) {
    ls_error_t ret;

    if (socket == NULL) {
        ret = LSE_INVALID_SOCKET;
    } else if (socket->busy != 0) {
        ret = LSE_SOCKET_BUSY;
    } else if (socket->state != LS_STATE_CONNECTED) {
        ret = LSE_NOT_CONNECTED;
    } else {
        socket->busy++;

        switch (socket->socket_type) {
             case LS_DATAGRAM: {
                 packet_t *packet = datagram_packet_create_from_socket(socket);
                 if (packet != NULL) {
                     /* If overriding the destination, change it now */
                     if (port >= 0) {
                         set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN, port);
                         set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN, address);
                     }
                     int tomove = len;
                     if (tomove > DATAGRAM_MAX_DATA) {
                         tomove = DATAGRAM_MAX_DATA;
                     }
                     memcpy(packet->buffer + DATAGRAM_PAYLOAD, buf, tomove);
                     packet->length += tomove;
//linklayer_print_packet("DG OUT", packet);
#ifdef NOTUSED
                     /* Go back to "anything goes" for next inbound packet */
                     if (socket->rename_dest) {
                         socket->dest_addr = NULL_ADDRESS;
                         socket->dest_port = 0;
                     }
#endif

                     UNLOCK_SOCKET(socket);
                     linklayer_route_packet(packet);
                     LOCK_SOCKET(socket);

                     ret = tomove;
                 } else {
                     ret = LSE_NO_MEM;
                 }
                 break;
             }

             case LS_STREAM: {
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
                 if (socket->state == LS_STATE_CONNECTED) {
                     int written = 0;

                     if (len != 0 || eor) {
                         while (len != 0 && !socket->output_window->shutdown) {
                             if (socket->current_write_packet == NULL) {
                                 assert(socket->output_window != NULL);
                                 socket->current_write_packet = stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_DATA, 0);
                             }

                             /* Compute how much we could write to the packet buffer */
                             int towrite =  MAX_PACKET_LEN - socket->current_write_packet->length;

                             /* Limit output to the amount available */
                             if (towrite > len) {
                                 towrite = len;
                             }

                             if (towrite != 0) {
                                 /* Copy data to packet */
                                 memcpy(socket->current_write_packet->buffer + socket->current_write_packet->length, buf, towrite);
                                 socket->current_write_packet->length += towrite;
                                 len -= towrite;
                                 buf += towrite;
                                 written += towrite;
                             }

                             /* If more data to send and we've run out of room, send it now. */
                             if (len != 0 && socket->current_write_packet->length == MAX_PACKET_LEN) {
                                 UNLOCK_SOCKET(socket);
//printf("flushing socket %d with %d bytes remaining\n", socket - socket_table, len);
                                 socket_flush_current_packet(socket, /* wait */ true);
                                 LOCK_SOCKET(socket);
                             }
                         }

                         /*
                          * Last packet generated.  If eor, add EOR falg
                          */
                         if (eor || socket->current_write_packet->length == MAX_PACKET_LEN) {
                             if (eor) {
                                 set_bits_field(socket->current_write_packet, STREAM_FLAGS, FLAGS_LEN, STREAM_FLAGS_EOR);
                             }
                             UNLOCK_SOCKET(socket);
                             socket_flush_current_packet(socket, /* wait */ true);
                             LOCK_SOCKET(socket);
                         } else {
                             /* Set a timer with CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY to force transmission if nothing else is stored in packet. */
                             simpletimer_start(&socket->socket_flush_timer, CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY);
                         }
                     }
                     ret = written;
                 } else {
                     /* Must be disconnecting - give error */
                     ret = LSE_DISCONNECTING;
                 }
#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

                 ret = LSE_NOT_IMPLEMENTED;

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
                 break;
             }

             default: {
                 ret = LSE_NOT_WRITABLE;
                 break;
            }
        }
        socket->busy--;
    }

    if (socket != NULL) {
        if (ret < 0) {
            socket->last_error = ret;
        }

        UNLOCK_SOCKET(socket);
    }

    return ret;
}

ls_error_t ls_write(int s, const void* buf, size_t len)
{
    return ls_write_helper(validate_socket(s), buf, len, false, 0, -1);
}

/*
 * Same as ls_write, but delivers an 'end of record' mark at end of data.
 * End of record write does nothing special for datagram sockets.
 */
ls_error_t ls_write_eor(int s, const void* buf, size_t len)
{
    return ls_write_helper(validate_socket(s), buf, len, true, 0, -1);
}

ls_error_t ls_write_to(int s, const void* buf, size_t len, ls_address_t address, ls_port_t port)
{
    return ls_write_helper(validate_socket(s), buf, len, true, address, port);
}

/*
 * Read a datagram or stream socket.
 *
 * For datagrams, waits for a packet and returns it's size.
 * For streams, waits and delivers as much data as requested or allowed in user buffer.
 * Excess is discarded.  The number of bytes delivered is return to the caller.
 *
 * Stream packets can be delimited by record boundaries.  If one is found, it will
 * terminate the stream delivery until the next read.
 *
 * Zero bytes delivered means socket has been closed either remotely or locally.
 *
 * Entry:
 *      s                   Socket containing data
 *      buf                 Buffer to receive data
 *      maxlen              Maximum length of buffer to receive data
 *
 * Returns:
 *      ls_error_t          If >0, number of bytes read.
 *                          If =0, socket has closed.  No more data to follow.
 *                          If <0, error code.
 *
 * For datagram sockets, reads one packet from receive queue and copies data to
 * user buffer.  Returns length read.
 *
 * For stream sockets, reads up to the length specified but stops at a record mark,
 * if one written by the caller.  A 'record mark' is a NULL packet.  Two record
 * marks signal the socket has closed.
 *
 * If the <address> and <port> parameters are present, record the source address and
 * port for the user.
 *
 * If timeout == -1, wait forever.
 */
ls_error_t ls_read_with_address(int s, char* buf, size_t maxlen, int* address, int* port, int timeout)
{
    ls_error_t ret = LSE_NO_ERROR;

    ls_socket_t *socket = validate_socket(s);

    if (socket == NULL) {
        ESP_LOGI(TAG, "%s: NULL socket", __func__);
        ret = LSE_INVALID_SOCKET;
    } else if (socket->busy != 0) {
        ret = LSE_SOCKET_BUSY;
    /* Allow any connected or post connected state where data might still be available */
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
    } else if (socket->state != LS_STATE_CONNECTED
               && socket->state != LS_STATE_INBOUND_DISCONNECTING
               && socket->state != LS_STATE_OUTBOUND_DISCONNECTING
               && socket->state != LS_STATE_DISCONNECTING_FLUSH_START
               && socket->state != LS_STATE_DISCONNECTING_FLUSHING
               && socket->state != LS_STATE_DISCONNECTED) {
#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
    } else if (socket->state != LS_STATE_CONNECTED) {
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

        ret = LSE_NOT_CONNECTED;
    } else {
        /* Mark that we are inside a user function */
        socket->busy++;

        switch (socket->socket_type) {
            case LS_DATAGRAM: {
                /* Pend on a packet in the queue */
                packet_t *packet;

                UNLOCK_SOCKET(socket);

                bool success = os_get_queue_with_timeout(socket->datagram_packets, (os_queue_item_t) &packet, timeout);

                LOCK_SOCKET(socket);

                if (success) {
                    if (packet == NULL) {
                        ret = LSE_CLOSED;
                    } else {

//linklayer_print_packet("ls_read DATAGRAM", packet);
                        /* A packet with data */
                        int packet_data_length = packet->length - DATAGRAM_PAYLOAD;
                        if (packet_data_length > maxlen) {
                            packet_data_length = maxlen;
                        }
                        memcpy(buf, packet->buffer + DATAGRAM_PAYLOAD, packet_data_length);

                        int origin_address = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
                        int origin_port = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN);

                        if (address != NULL) {
                            *address = origin_address;
                        }
                        if (port != NULL) {
                            *port = origin_port;
                        }

#ifdef NOTUSED
                        /* Switch destination origin of packet so a write goes to correct location */
                        if (socket->rename_dest) {
                            socket->dest_addr = origin_address;
                            socket->dest_port = origin_port;
                        }
#endif

                        ret = packet_data_length;
                        release_packet(packet);
                    }
                } else {
                    ret = LSE_TIMEOUT;
                }
                break;
            }

            case LS_STREAM: {
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAM
                /* STREAM packets arrive on the socket stream_packet_queue after being sorted and acked as necessary */
                bool eor = false;
                int total_len = 0;

                /* Create the ACK sender machine */
                simpletimer_start(&socket->input_window_timer, CONFIG_LASTLINK_STREAM_UPDATE_TIME * route_metric(socket->dest_addr) * 2);

                /* Go until length satisfied or end of record */
                while (!eor && !socket->end_of_data && maxlen != 0) {
                    int offset = 0;

                    packet_t *packet;

                    /* If current_read_packet is defined, pick up where we left off... */
                    if (socket->current_read_packet != NULL) {
                        packet = socket->current_read_packet;
                        offset = socket->current_read_offset;
                    } else {
                        /* This blocks until packet is ready or socket is shut down remotely */
                        UNLOCK_SOCKET(socket);

//printf("%s: flushing current output packet\n", __func__);
                        if (socket_flush_current_packet(socket, /*wait*/ true)) {
                            simpletimer_set_expired(&socket->output_window_timer);
                        }

//printf("%s: waiting for input\n", __func__);
                        packet_window_remove_sequential_packet(socket->input_window, &packet, -1);

                        LOCK_SOCKET(socket);

                        /* As long as packets flow, keep the timer from refiring */
                        simpletimer_restart(&socket->input_window_timer);
                        offset = 0;
                    }

                    if (packet != NULL) {
                        int available = packet->length - STREAM_PAYLOAD - offset;

                        /* Limit to maximum user will accept */
                        if (available > maxlen) {
                            available = maxlen;
                        }

                        memcpy(buf, packet->buffer + STREAM_PAYLOAD + offset, available);

                        buf += available;
                        maxlen -= available;
                        total_len += available;
                        offset += available;

                        /* if we consumed all the data in the packet, release it */
                        if (offset == packet->length - STREAM_PAYLOAD) {
                            /* If this was end of record, stop the reading here */
                            eor = (get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN) & STREAM_FLAGS_EOR) != 0;
                            /* Release packet */
                            release_packet(packet);
                            socket->current_read_packet = NULL;
                        } else {
                            /* Otherwise save the packet and offset */
                            socket->current_read_packet = packet;
                            socket->current_read_offset = offset;
                        }
                    } else {
                        /* Socket remotely closed.  Return residue for last read. */
                        socket->end_of_data = true;
//printf("%s: end of data detected\n", __func__);
                    }
                }
                /* Return amount of data read */
                ret = total_len;
#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

                ret = LSE_NOT_IMPLEMENTED;

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
                break;
            }

            default: {
                ret = LSE_INVALID_SOCKET;
                break;
            }
        }
        socket->busy--;
    }

    if (socket != NULL) {
        if (ret < 0) {
            socket->last_error = ret;
        }

        UNLOCK_SOCKET(socket);
    }

    return ret;
}

ls_error_t ls_read(int s, void* buf, size_t maxlen)
{
    return ls_read_with_address(s, (char*) buf, maxlen, NULL, NULL, -1);
}

ls_error_t ls_read_with_timeout(int s, void* buf, size_t maxlen, int timeout)
{
    return ls_read_with_address(s, (char*) buf, maxlen, NULL, NULL, timeout);
}


/*
 * Close a socket.
 */
static ls_error_t ls_close_ptr(ls_socket_t *socket)
{
    ls_error_t err = LSE_NO_ERROR;
    static void* null = NULL;

    if (socket != NULL) {
        bool already_busy = socket->busy != 0;

        socket->busy++;

        switch (socket->socket_type) {
            default: {
                err = LSE_INVALID_SOCKET;
                break;
            }

            case LS_DATAGRAM: {
                if (already_busy) {
                    /* Just let reader know it is being shut down */
                    os_put_queue_with_timeout(socket->datagram_packets, (os_queue_item_t) &null, 0);
                } else {
                    err = release_socket(socket);
                }
                break;
            }

            case LS_STREAM: {
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
                if (socket->state == LS_STATE_LISTENING) {
                    /* In case it's another thread doing the listening, give it a shutdown notice */
                    if (already_busy) {
                        os_put_queue_with_timeout(socket->connections, (os_queue_item_t) &null, 0);
                    } else {
                        err = release_socket(socket);
                    }
                } else if (already_busy) {
                    socket->state = LS_STATE_CLOSING;

                } else {
                    UNLOCK_SOCKET(socket);
                    bool flush_needed = socket_flush_current_packet(socket, /*wait*/ true);
                    LOCK_SOCKET(socket);

// This is probably not needed for wait-type flushes
                    if (flush_needed) {
printf("%s: ******************************************************************************************************************\n", __func__);
printf("%s: flush needed; starting flushing of output window\n", __func__);
printf("%s: ******************************************************************************************************************\n", __func__);
                        /*
                         * More work so flush the output window
                         * This starts a send_output_window and posts a results
                         * back to the state machine results queue when done (error or otherwise)
                         */
                        simpletimer_stop(&socket->output_window_timer);
                        socket->output_retries = 0;

                        state_machine_start("disconnect flush",
                                            socket,
                                            LS_STATE_DISCONNECTING_FLUSH_START,
                                            state_machine_action_send_output_window,
                                            STATE_MACHINE_RESULTS_TO_QUEUE,             /* Results back to state_machine_results */
                                            STREAM_FLUSH_TIMEOUT,                       /* This state is a poll-only so it doesn't retransmit the packet each timeout */
                                            STREAM_FLUSH_RETRIES);

                        UNLOCK_SOCKET(socket);
                        err = state_machine_get_results(socket);
//printf("%s: Received flush response: %d\n", __func__, err);
                        LOCK_SOCKET(socket);
                    }

                    /* If generating the disconnect first */
                    if (socket->state != LS_STATE_INBOUND_DISCONNECTING) {

                        state_machine_start("outbound disconnect 2",
                                            socket,
                                            LS_STATE_OUTBOUND_DISCONNECTING,
                                            state_machine_action_send_disconnect,
                                            STATE_MACHINE_RESULTS_TO_QUEUE,         /* Results back to state_machine_results */
                                            -1,                                     /* Use action time to set delay */
                                            STREAM_DISCONNECT_RETRIES);

                        UNLOCK_SOCKET(socket);
                        err = state_machine_get_results(socket);
//printf("Received disconnect ack: %d\n", err);
                        LOCK_SOCKET(socket);
                    }

                    stream_shutdown_windows(socket);
                    packet_window_shutdown_window(socket->output_window);

//printf("Starting linger\n");

                    /* Start a linger to leave socket intact for a short while */
                    state_machine_start("linger",
                                        socket,
                                        LS_STATE_LINGER,
                                        state_machine_action_linger_check,
                                        state_machine_results_linger_finish,
                                        CONFIG_LASTLINK_SOCKET_LINGER_TIME / 5,
                                        5);
                }
#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

                err = LSE_NOT_IMPLEMENTED;

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

                break;
            }
        }

        socket->busy--;

        UNLOCK_SOCKET(socket);

    } else {
        err = LSE_INVALID_SOCKET;
    }

    return err;
}

ls_error_t ls_close(int s)
{
    return ls_close_ptr(validate_socket(s));
}


/*
 * Validate socket table index and return it's pointer, locked.
 */
ls_socket_t *validate_socket(int s)
{
    ls_socket_t *ret = NULL;

    if (s >= FIRST_LASTLINK_FD && s <= LAST_LASTLINK_FD) {

        s -= FIRST_LASTLINK_FD;

        if (s >= 0 && s < ELEMENTS_OF(socket_table)) {

            ls_socket_t *socket = &socket_table[s];

            LOCK_SOCKET(socket);

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
            if (socket->socket_type == LS_DATAGRAM || socket->socket_type == LS_STREAM) {
#else /* !CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
            if (socket->socket_type == LS_DATAGRAM) {
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
                ret = socket;
            } else {
                UNLOCK_SOCKET(socket);
            }
        }
    }

    return ret;
}

int ls_fstat(int fd, struct stat *st)
{
    return LSE_NOT_IMPLEMENTED;
}

int ls_fcntl(int s, int cmd, int arg)
{
    return LSE_NOT_IMPLEMENTED;
}

int ls_ioctl(int s, int cmd, ...)
{
    return LSE_NOT_IMPLEMENTED;
}

static int ls_ioctl_r_wrapper(int fd, int cmd, va_list args)
{
    return ls_ioctl(fd, cmd, va_arg(args, void *));
}

int ls_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset, struct timeval *timeout)
{
#ifdef NOTUSED
    fd_set   lreadset;
    fd_set   lwriteset;
    fd_set   lexceptset;
    fd_set   used_sockets;

    if (maxfdp1 < 0 || maxfdp1 > LASTLINK_SELECT_MAXNFDS) {
        set_error(EINVAL);
        return -1;
    }

    ls_select_sockets_used(maxfdp1, readset, writeset, exceptset, &used_sockets);

    int nready = ls_select_scan(maxfdp1, readset, writeset, exceptset, &lreadset, &lwriteset, &lexceptset);

    if (nready < 0) {
        /* One or more fd_sets were invalid */
        set_errno(EBADF);
        ls_deselect_sockets_used(maxfdp1, &used_sockets);
        return -1;
    }

    if (nready == 0) {
        /* No socket ready - we need to wait */
        if (timeout && timeout->tv_sec == 0 && timeout->tv_usec == 0) {
            /* No timeout given, so we are done. */
        } else {
            ls_select_cb select_b;
            API_SELECT_CB_VAR_ALLOC(select_cb, set_errno(ENOMEM); ls_select_sockets_used(maxfdp1, &used_sockets); return -1);
            memset(&API_SELECT_CB_VAR_REF(select_cb), 0, sizeof(struct lwip_select_cb));

            select_cb.readset = readset;
            select_cb.writeset = writeset;
            select_cb.exceptset = exceptset;
            select_cb.sem = xxx;

            ls_link_select_cb(&API_SELECT_CB_VAR_REF(select_cb));

            /* Increase select_waiting for each socket we are interested in */
            int maxfdp2 = maxfdp1;
            for (i = LWIP_SOCKET_OFFSET; i < maxfdp1; i++) {
                if ((readset && FD_ISSET(i, readset)) || (writeset && FD_ISSET(i, writeset)) || (exceptset && FD_ISSET(i, exceptset))) {
                    struct lwip_sock *sock;
                    SYS_ARCH_PROTECT(lev);
                    sock = tryget_socket_unconn_locked(i);
                    if (sock != NULL) {
                        sock->select_waiting++;
                        if (sock->select_waiting == 0) {
                            /* overflow - too many threads waiting */
                            sock->select_waiting--;
                            nready = -1;
                            maxfdp2 = i;
                            SYS_ARCH_UNPROTECT(lev);
                            done_socket(sock);
                            set_errno(EBUSY);
                            break;
                        }
                        SYS_ARCH_UNPROTECT(lev);
                        done_socket(sock);
                    } else {
                        /* Not a valid socket */
                        nready = -1;
                        maxfdp2 = i;
                        SYS_ARCH_UNPROTECT(lev);
                        set_errno(EBADF);
                        break;
                    }
                }
            }

            if (nready >= 0) {
                /* Call lwip_selscan again: there could have been events between
                   the last scan (without us on the list) and putting us on the list! */
                nready = lwip_selscan(maxfdp1, readset, writeset, exceptset, &lreadset, &lwriteset, &lexceptset);
                if (!nready) {
                    /* Still none ready, just wait to be woken */
                    if (timeout == 0) {
                        /* Wait forever */
                        msectimeout = 0;
                    } else {
                        long msecs_long = ((timeout->tv_sec * 1000) + ((timeout->tv_usec + 500) / 1000));
                        if (msecs_long <= 0) {
                            /* Wait 1ms at least (0 means wait forever) */
                            msectimeout = 1;
                        } else {
                            msectimeout = (u32_t)msecs_long;
                        }
                    }

                    waitres = sys_arch_sem_wait(SELECT_SEM_PTR(API_SELECT_CB_VAR_REF(select_cb).sem), msectimeout);
                    waited = 1;
                }
            }

            /* Decrease select_waiting for each socket we are interested in */
            for (i = LWIP_SOCKET_OFFSET; i < maxfdp2; i++) {
                if ((readset && FD_ISSET(i, readset)) || (writeset && FD_ISSET(i, writeset)) || (exceptset && FD_ISSET(i, exceptset))) {
                    struct lwip_sock *sock;
                    SYS_ARCH_PROTECT(lev);
                    sock = tryget_socket_unconn_locked(i);
                    if (sock != NULL) {
                        /* for now, handle select_waiting==0... */
                        LWIP_ASSERT("sock->select_waiting > 0", sock->select_waiting > 0);
                        if (sock->select_waiting > 0) {
                            sock->select_waiting--;
                        }
                        SYS_ARCH_UNPROTECT(lev);
                        done_socket(sock);
                    } else {
                        SYS_ARCH_UNPROTECT(lev);
                        /* Not a valid socket */
                        nready = -1;
                        set_errno(EBADF);
                    }
                }
            }

            lwip_unlink_select_cb(&API_SELECT_CB_VAR_REF(select_cb));

            if (API_SELECT_CB_VAR_REF(select_cb).sem_signalled && (!waited || (waitres == SYS_ARCH_TIMEOUT))) {
              /* don't leave the thread-local semaphore signalled */
              sys_arch_sem_wait(API_SELECT_CB_VAR_REF(select_cb).sem, 1);
            }
            API_SELECT_CB_VAR_FREE(select_cb);

            if (nready < 0) {
                /* This happens when a socket got closed while waiting */
                ls_deselect_sockets_used(maxfdp1, &used_sockets);
                return -1;
            }

            if (waitres == SYS_ARCH_TIMEOUT) {
                /* Timeout */
            } else {
                /* See what's set now after waiting */
                nready = lwip_selscan(maxfdp1, readset, writeset, exceptset, &lreadset, &lwriteset, &lexceptset);
            }
        }
    }
#else
    return LSE_NOT_IMPLEMENTED;
#endif
}

/*
 *
 * ls_get_last_error
 *
 * Return the last error code and clear it.
 *
 * Entry:
 *      socket              Socket being queried.
 *
 * Returns:
 *      last error code from socket.
 */
ls_error_t ls_get_last_error(int s)
{
    ls_socket_t *socket = validate_socket(s);

    ls_error_t err;

    if (socket != NULL) {
        err = socket->last_error;
        socket->last_error = LSE_NO_ERROR;

        UNLOCK_SOCKET(socket);
    } else {
        err = LSE_INVALID_SOCKET;
    }

    return err;
}

/*
 * Get local port of socket
 */
ls_error_t ls_get_src_port(int s)
{
    ls_socket_t *socket = validate_socket(s);

    ls_error_t err;

    if (socket != NULL) {
        err = socket->src_port;
        UNLOCK_SOCKET(socket);
    } else {
        err = LSE_INVALID_SOCKET;
    }

    return err; 
}

static ls_error_t ls_dump_socket_ptr(const char* msg, const ls_socket_t *socket)
{
    if (socket != NULL) {
        switch (socket->socket_type) {
            default:
            case LS_DATAGRAM: {
                printf("%s%s %d (%p) state %s type %s src port %d dest port %d dest addr %d\n",
                       msg ? msg : "",
                       msg ? ": " : "",
                       socket - socket_table + FIRST_LASTLINK_FD,
                       socket,
                       socket_state_of(socket),
                       socket_type_of(socket),
                       socket->src_port,
                       socket->dest_port,
                       socket->dest_addr);
                break;
            }
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
            case LS_STREAM: {
                if (socket->state == LS_STATE_LISTENING) {
                    printf("%s%s %d (%p) state %s type %s src port %d dest port %d dest addr %d\n",
                           msg ? msg : "",
                           msg ? ": " : "",
                           socket - socket_table + FIRST_LASTLINK_FD,
                           socket,
                           socket_state_of(socket),
                           socket_type_of(socket),
                           socket->src_port,
                           socket->dest_port,
                           socket->dest_addr);
                } else {
                    printf("%s%s %d (%p) state %s type %s local port %d dest port %d dest addr %d parent %d input %d[%d] output %d[%d]\n",
                           msg ? msg : "",
                           msg ? ": " : "",
                           socket - socket_table + FIRST_LASTLINK_FD,
                           socket,
                           socket_state_of(socket),
                           socket_type_of(socket),
                           socket->src_port,
                           socket->dest_port,
                           socket->dest_addr,
                           socket->parent ? (socket->parent - socket_table) : -1,
                           socket->input_window ? socket->input_window->sequence : -1,
                           packet_window_packet_count(socket->input_window),
                           socket->output_window ? socket->output_window->sequence : -1,
                           packet_window_packet_count(socket->output_window));
                           if (socket->input_window != NULL) {
                               for (int slot = 0; slot < socket->input_window->length; ++slot) {
                                   if (socket->input_window->queue[slot].inuse) {
                                       if (socket->input_window->queue[slot].packet != NULL) {
                                           printf("   In %d: %d\n", slot, get_uint_field(socket->input_window->queue[slot].packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN));
                                       } else {
                                           printf("   In %d: NULL\n", slot);
                                       }
                                   }
                               }
                           } else {
                               printf("Input window missing\n");
                           }
                           if (socket->input_window != NULL) {
                               for (int slot = 0; slot < socket->input_window->length; ++slot) {
                                   if (socket->output_window->queue[slot].inuse) {
                                       if (socket->output_window->queue[slot].packet != NULL) {
                                           printf("   Out %d: %d\n", slot, get_uint_field(socket->output_window->queue[slot].packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN));
                                       } else {
                                           printf("   Out %d: NULL\n", slot);
                                       }
                                   }
                               }
                           } else {
                               printf("   Output window missing\n");
                           }
                }
                break;
            }
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
        }
#ifdef CONFIG_LASTLINK_SOCKET_LOCKING_DEBUG
        if (socket->lock_count != 0) {
            printf("   Lock count %d last by %s:%d\n", socket->lock_count, socket->last_lock_file, socket->last_lock_line);
        }
#endif
    } else {
        printf("%s: invalid socket\n", msg);
    }

    return 0;
}

ls_error_t ls_dump_socket(const char* msg, int s)
{
    ls_socket_t *socket = validate_socket(s);

    ls_error_t err = ls_dump_socket_ptr(msg, socket);

    UNLOCK_SOCKET(socket);

    return err;
}

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS

/* Called by timer to work on the sockets.
 * Rechedules itself when new time interval is needed.
 */
static void socket_scanner_thread(void* param)
{
    while (true) {

        socket_scanner_running++;
 
        uint32_t next_time = SOCKET_SCANNER_DEFAULT_INTERVAL;

        for (int socket_index = 0; socket_index < ELEMENTS_OF(socket_table); ++socket_index) {
            ls_socket_t *socket = &socket_table[socket_index];

            /* Attempt to acquire the socket - if not, move to the next one and get it next time */
            /* Currently we only examine active STREAM sockets */
            if (socket->socket_type == LS_STREAM) {
                if (LOCK_SOCKET_TRY(socket)) {

                    /* Check the socket timers and fire action if ready */
                    if (simpletimer_is_expired_or_remaining(&socket->state_machine_timer, &next_time)) {
                        simpletimer_stop(&socket->state_machine_timer);
                        state_machine_timeout(socket);
                    }

                    /* This fires to flush the data from the output buffer to the window */
                    if (simpletimer_is_expired_or_remaining(&socket->socket_flush_timer, &next_time)) {
                        simpletimer_stop(&socket->socket_flush_timer);
                        socket_flush_current_packet(socket, /* wait */ false);
                    }

                    /* This one runs on demand to flush the output_window as needed - runs until successful or timesout closing connection */
                    if (simpletimer_is_expired_or_remaining(&socket->output_window_timer, &next_time)) {
                        simpletimer_stop(&socket->output_window_timer);
                        send_output_window(socket, /* disconnect_on_error */ true);
                    }

                    /* This one fires when we've received data that needs an ack */
                    if (simpletimer_is_expired_or_remaining(&socket->ack_delay_timer, &next_time)) {

                        /* Finished with timer */
                        simpletimer_stop(&socket->ack_delay_timer);

                        ///* Delay input window ack until next time */
                        ack_input_window(socket);
                    }

#if 1
                    /* This one runs continually and only works when the socket is input-busy */
                    if (simpletimer_is_expired_or_remaining(&socket->input_window_timer, &next_time)) {
                        simpletimer_stop(&socket->input_window_timer);
                        ack_input_window(socket);
                    }
#endif

#if CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLE
                    if (simpletimer_is_expired_or_remaining(&socket->keepalive_timer, &next_time)) {
                        /* Only need to do this if socket is idle */
                        if (! simpletimer_is_running(&socket->input_window_timer) && ! simpletimer_is_running(&socket->ack_delay_timer)) {
                            /* Send a keep-alive packet */
                            stream_send_keepalive(socket);
                        }
                        simpletimer_restart(&socket->keepalive_timer);
                    }
#endif

                    UNLOCK_SOCKET(socket);
                }
else printf("%s: socket %d locked by %s:%d\n", __func__, socket - socket_table, socket->last_lock_file, socket->last_lock_line);
            }
        }

        os_delay(next_time);
    }
}
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

/*
 * User interface commands.
 */
#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
typedef struct {
    int         queue;
    int         sequence;
    int         to;
    bool        thread;
    int         retries;
    bool        routed;
    ls_error_t  error;
} ping_info_table_t;

static int print_ping_table(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "[ -a ]", "Show ping table");
    } else {
        ping_info_table_t  pings[CONFIG_LASTLINK_MAX_OUTSTANDING_PINGS];

        int num_pings = 0;

        if (linklayer_lock()) {
            while (num_pings < ELEMENTS_OF(pings)) {
                pings[num_pings].queue = (ping_table[num_pings].queue != NULL) ? os_items_in_queue(ping_table[num_pings].queue) : -1;
                pings[num_pings].sequence = ping_table[num_pings].sequence;
                pings[num_pings].to = ping_table[num_pings].packet ? get_uint_field(ping_table[num_pings].packet, HEADER_DEST_ADDRESS, ADDRESS_LEN) : 0;
                pings[num_pings].thread = ping_table[num_pings].thread_id != NULL;
                pings[num_pings].retries = ping_table[num_pings].retries;
                pings[num_pings].routed = ping_table[num_pings].routed;
                pings[num_pings].error = ping_table[num_pings].error;

                ++num_pings;
            }

            linklayer_unlock();
        }

        bool all = argc > 1 && strcmp(argv[1], "-a") == 0;

        if (num_pings != 0) {
            bool header_printed = false;

            for (int index = 0; index < num_pings; ++index) {
                if (all || pings[index].sequence != 0) {
                    if (! header_printed) {
                        printf("Sequence  Queue  To    Thread  Routed  Retries  Error\n");
                        header_printed = true;
                    }
                    printf("%-8d  %-5d  %-4d  %-6s  %-6s  %-7d  %-5d\n",
                            pings[index].sequence,
                            pings[index].queue,
                            pings[index].to,
                            pings[index].thread ? "YES" : "NO",
                            pings[index].routed ? "YES" : "NO",
                            pings[index].retries,
                            pings[index].error);
                }
            }
        }
    }

    return 0;
}

/*
 * Absorb all characters in queue looking for match with testch.
 * Return true if one seen.
 */
static bool hit_test(int testch)
{
    bool found = false;
    int ch;

    while ((ch = getchar()) >= 0) {
        if (ch == testch) {
            found = true;
        }
    }

    return found;
}

/**********************************************************************/
/* dglisten <port>                                                    */
/**********************************************************************/
static int dglisten_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<port>", "Listen for datagrams on <port>");
    } else if (argc > 1) {
        int port = strtol(argv[1], NULL, 10);
        int socket = ls_socket(LS_DATAGRAM);
        if (socket >= 0) {
            printf("listening socket %d port %d...\n", socket, port);
            int ret = ls_bind(socket, port);
            if (ret >= 0) {
                /* Accept packets from any */
                ret = ls_connect(socket, NULL_ADDRESS, 0);
                if (ret == LSE_NO_ERROR) {
                    ls_dump_socket("receiving", socket);
                    while (!hit_test('\x03')) {
                        char buf[200];
                        int address;

                        int len = ls_read_with_address(socket, buf, sizeof(buf), &address, &port, 50);
                        if (len >= 0) {
                            buf[len] = '\0';
                            printf("Data from %d/%d len %d: '%s'\n", address, port, len, buf);
                        } else if (len != LSE_TIMEOUT) {
                            printf("ls_read returned %d\n", len);
                        }
                    }
                } else {
                    printf("ls_connect returned %d\n", ret);
                }
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            ls_close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }
    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

/**********************************************************************/
/* dgsend <address> <port> <data> <burst count>                       */
/**********************************************************************/
static int dgsend_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<address> <port> <data> <burst>", "Send datagram <data> to <port> on <address>");
    } else if (argc >= 4) {
        int address = strtol(argv[1], NULL, 10);
        int port = strtol(argv[2], NULL, 10);
        const char* data = argv[3];
        int count = 1;
        if (argc > 4) {
            count = strtol(argv[4], NULL, 10);
        }
        // printf("Sending '%s' to %d/%d\n", argv[3], address, port);
        int socket = ls_socket(LS_DATAGRAM);
        if (socket >= 0) {
            int ret = ls_bind(socket, 0);
            if (ret >= 0) {
                ret = ls_connect(socket, address, port);
                //ls_dump_socket("sending", socket);
                if (ret >= 0) {
                    if (count != 1) {
                        while (ret >= 0 && count != 0) {
                            char tempbuf[strlen(data) + 10];
                            sprintf(tempbuf, "%s: %d", data, count);
                            ret = ls_write(socket, tempbuf, strlen(tempbuf));
                            printf("ls_write returned %d\n", ret);
                            if (--count != 0) {
                                os_delay(50);
                            }
                        }
                    } else {
                        ret = ls_write(socket, data, strlen(data));
                        printf("ls_write returned %d\n", ret);
                    }
                } else {
                    printf("ls_connect returned %d\n", ret);
                }
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            ls_close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }

    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

/**********************************************************************/
/* dgcon <address> <port>                                             */
/* Enter a loop and take commands entered and send to the socket,     */
/* while displaying packets received from the socket.                 */
/* Control-C breaks out                                               */
/**********************************************************************/
typedef struct dgcon_data {
    os_thread_t thread_id;
    int socket;
    int running;
} dgcon_data_t;

static void dgcon_reader(void* data)
{
    dgcon_data_t* dgdata = (dgcon_data_t*) data;

    dgdata->running = 1;

    while (dgdata->running > 0) {
        /* Read data from socket and echo to terminal */
        char buffer[300];

        int address;
        int port;

        int len = ls_read_with_address(dgdata->socket, buffer, sizeof(buffer) - 1, &address, &port, 500);

        if (len > 0) {
            buffer[len] = '\0';
            printf("%s", buffer); 
        }
    }

    dgdata->running = 0;

    os_exit_thread();
}


static int dgcon_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<address> <port>", "Send and receive datagram date");
    } else if (argc == 3) {
        int address = strtol(argv[1], NULL, 10);
        int port = strtol(argv[2], NULL, 10);

        int socket = ls_socket(LS_DATAGRAM);
        if (socket >= 0) {
            int ret = ls_bind(socket, 0);
            if (ret >= 0) {
                ret = ls_connect(socket, address, port);
                //ls_dump_socket("sending", socket);
                if (ret >= 0) {
                    dgcon_data_t dgdata;
                    dgdata.socket = socket;

                    /* Spawn a reader socket to echo back input packets to the control */
                    dgdata.thread_id = os_create_thread(dgcon_reader, "dgcon_reader", 4095, 0, (void*) &dgdata);

                    /* Loop reading but terminate on control-c (-1 return) */
                    int len;

                    do {
                        char buffer[80];

                        len = readline(buffer, sizeof(buffer));
                        if (len > 0) {
                            printf("Writing \"%s\"\n", buffer);
                            int rc = ls_write(socket, buffer, len);
                            if (rc < 0) {
                                printf("rc %d\n", rc);
                            }
                                 
                        }
                    } while (len > 0);
   
                    /* Kill reader thread */
                    dgdata.running = -1;
                    while (dgdata.running != 0) {
                        os_delay(100);
                    }
                } else {
                    printf("ls_connect returned %d\n", ret);
                }
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            ls_close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }

    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
/**********************************************************************/
/* stconsumer <port>                                                  */
/**********************************************************************/
static int stconsumer_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<port>", "Listen for stream connection on <port> and consume data");
    } else if (argc > 1) {
        int port = strtol(argv[1], NULL, 10);
        int socket = ls_socket(LS_STREAM);
        printf("socket returned %d\n", socket);
        if (socket >= 0) {
            int ret = ls_bind(socket, port);
            printf("bind returned %d\n", ret);
            if (ret >= 0) {
                /* Accept packets from any */
                bool done = false;
                printf("listening socket %d port %d...\n", socket, port);
                do {
                    int connection = ls_listen(socket, 5, 50);
                    if (connection >= 0) {
                        printf("got connection on %d\n",connection);
                        char buffer[80];
                        int len;
                        while ((len = ls_read(connection, buffer, sizeof(buffer))) > 0) {
                            fwrite(buffer, len, 1, stdout);
                        }
                        printf("--end--\n");
                        ls_close(connection);
                        done = hit_test('\x03');
                    } else if (connection != LSE_TIMEOUT) {
                        printf("ls_listen returned %d\n", connection);
                        done = true;
                    } else {
                        done = hit_test('\x03');
                    }
                } while (!done);
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            ls_close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }
    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

/**********************************************************************/
/* stproducer <port> [ 'data to send' ]                               */
/**********************************************************************/
static int stproducer_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<port> ['data']", "Listen for stream connection on <port> and produce data");
    } else if (argc > 1) {
        int port = strtol(argv[1], NULL, 10);
        const char *data = "this is a line of test data";
        if (argc > 2)  {
            data = argv[2];
        }

        int socket = ls_socket(LS_STREAM);
        printf("socket returned %d\n", socket);
        if (socket >= 0) {
            int ret = ls_bind(socket, port);
            printf("bind returned %d\n", ret);
            if (ret >= 0) {
                /* Accept packets from any */
                bool done = false;
                printf("listening socket %d port %d...\n", socket, port);
                do {
                    int connection = ls_listen(socket, 5, 50);
                    if (connection >= 0) {
                        char buffer[80];
                        printf("got connection on %d\n",connection);

                        /* Read one line to get number of items to produce */
                        int len = read(connection, buffer, sizeof(buffer));
                        int count = strtol(buffer, NULL, 10);

                        printf("read %d bytes value %d\n", len, count);

                        for (int line = 1; line <= count; ++line) {
                            sprintf(buffer, "%d: %s\n", line, data);
                            int towrite = strlen(buffer);
                            int len = write(connection, buffer, towrite);
                            if (len != towrite) {
                                printf("%s: wrote %d but write returned %d\n", __func__, towrite, len);
                            }
                        }

                        int ret = close(connection);
                        printf("connection closed %d\n", ret);

                    } else if (connection != LSE_TIMEOUT) {
                        printf("ls_listen returned %d\n", connection);
                        done = true;
                    } else {
                        done = hit_test('\x03');
                    }
                } while (!done);
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }
    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

/**********************************************************************/
/* stwrite <address> <port> [ 'data' ]                                */
/**********************************************************************/
static int stwrite_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<address> <port>", "Connect to stream <port> on <address>");
    } else if (argc >= 3) {
        int address = strtol(argv[1], NULL, 10);
        int port = strtol(argv[2], NULL, 10);
        const char *message = "this is a test message";
        if (argc >= 4) {
            message = argv[3];
        }

        int socket = ls_socket(LS_STREAM);
        if (socket >= 0) {
            int ret = ls_bind(socket, 0);
            if (ret >= 0) {
                ret = ls_connect(socket, address, port);
                if (ret >= 0) {
                    int total = 0;
                    simpletimer_t timer;
                    simpletimer_start(&timer, 0xFFFFFFFFL);

                    for (int line = 1; line <= 100; ++line) {
                        char buffer[80];
                        sprintf(buffer, "%s: line %d\n", message, line);
                        total += strlen(buffer);
                        ret = write(socket, buffer, strlen(buffer));
                        printf("write returned %d\n", ret);
                    }

                    int seconds = (simpletimer_elapsed(&timer) + 500) / 1000;
                    if (seconds < 1) {
                        seconds = 1;
                    }

                    printf("%d bytes in %d seconds;  %d bytes/sec\n", total, seconds, total / seconds);
                } else {
                    printf("ls_connect returned %d\n", ret);
                }
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            printf("closing socket\n");
            ret = ls_close(socket);
            printf("close returned %d\n", ret);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }

    } else {
        printf("Insufficient params\n");
    }
    return 0;
}


/**********************************************************************/
/* stread <address> <port>                                            */
/**********************************************************************/
static int stread_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<address> <port>", "Starts thread to connect to stream <port> on <address> and read to end");
    } else if (argc >= 3) {
        int address = strtol(argv[1], NULL, 10);
        int port = strtol(argv[2], NULL, 10);
        int count = 100;

        if (argc >= 4) {
            count = strtol(argv[3], NULL, 10);
        }

        int socket = ls_socket(LS_STREAM);

        if (socket >= 0) {
            int ret = ls_bind(socket, 0);
            if (ret >= 0) {
                ret = ls_connect(socket, address, port);
printf("%s: ls_connect returned %d\n", __func__, ret);
                if (ret >= 0) {
                    simpletimer_t timer;
                    simpletimer_start(&timer, 0xFFFFFFFFL);

                    char buffer[80];
                    int len;
                    int total = 0;

                    /* First write how many cycles we want */
                    sprintf(buffer, "%d\n", count);
                    ls_write_eor(socket, buffer, strlen(buffer));

                    while ((len = read(socket, buffer, sizeof(buffer))) > 0) {
                        fwrite(buffer, len, 1, stdout);
                        total += len;
                    }

                    int seconds = (simpletimer_elapsed(&timer) + 500) / 1000;
                    if (seconds < 1) {
                        seconds = 1;
                    }

                    printf("ret %d:  %d bytes in %d seconds;  %d bytes/sec\n", len, total, seconds, total / seconds);

                } else {
                    printf("ls_connect returned %d\n", ret);
                }
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            printf("closing socket\n");
            ret = ls_close(socket);
            printf("close returned %d\n", ret);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }
    } else {
        printf("Insufficient params\n");
    }

    return 0;
}

/**********************************************************************/
/* stconcycle <address> <port>                                        */
/* connect/disconnect loop with delay                                 */
/**********************************************************************/
static int stconcycle_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<address> <port>", "Cycle connect/disconnect");
    } else if (argc >= 3) {
        int address = strtol(argv[1], NULL, 10);
        int port = strtol(argv[2], NULL, 10);

        for (int count = 0; !hit_test('\x03') && count < 20; ++count) {
            int socket = ls_socket(LS_STREAM);
            if (socket >= 0) {
                int ret = ls_bind(socket, 0);
                if (ret == LSE_NO_ERROR) {
                    ret = ls_connect(socket, address, port);
                    if (ret == LSE_NO_ERROR) {
                        os_delay(5000);
                    } else {
                        printf("ls_connect returned %d\n", ret);
                    }
                    os_delay(5000);
                } else {
                    printf("ls_bind returned %d\n", ret);
                }
                ls_close(socket);
            } else {
                printf("Unable to create socket: %d\n", socket);
            }

            os_delay(5000);
        }

    } else {
        printf("Insufficient params\n");
    }
    return 0;
}
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

/**********************************************************************/
/* ping <node address>                                                */
/**********************************************************************/
static int ping_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<node address> [<count>]", "Find and report path to a node");
    } else {
        int address = strtol(argv[1], NULL, 10);
        int count = 1;

        if (argc > 2) {
           count = strtol(argv[2], NULL, 10);
           if (count < 0) {
               count = 1;
           }
        }

        int  fail = 0;
        int  good = 0;
        int  total = 0;

        int paths[MAX_METRIC + 1];

        int sequence = 0;
        while (!hit_test('\x03') && count != 0) {
            sequence++;
            uint32_t  elapsed;

            int path_len = ping(address, &elapsed, paths, ELEMENTS_OF(paths));
            ++total;

            if (path_len < 0) {
                ++fail;
                printf("%d: Ping error %d\n", sequence, path_len);
            } else {
                ++good;
                printf("%d: %d mS Path", sequence, elapsed);
                for (int path = 0; path < path_len; ++path) {
                    printf(" %d", paths[path]);
                }

                printf("\n");
            }

            count--;

            if (count != 0) {
                os_delay(1000);
            }
        }
        printf("%d fail  %d good  %d total\n", fail, good, total);
    }

    return 0;
}

int print_sockets(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Dump sockets");
    } else {
        for (ls_socket_t *socket = &socket_table[0]; socket < &socket_table[ELEMENTS_OF(socket_table)]; ++socket) {
            //LOCK_SOCKET(socket);
            if (socket->socket_type != LS_UNUSED) {
                ls_dump_socket_ptr(NULL, socket);
            }
            //UNLOCK_SOCKET(socket);
        }
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
        printf("socket_scanner_running %d\n", socket_scanner_running);
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
    }
    return 0;
}

#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

/*
 * Initialize ls_socket layer.
 */
ls_error_t ls_socket_init(void)
{
    ls_error_t err = 0;

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
    packet_window_init();
#endif

    /* Initialize socket_table */
    memset(&socket_table, 0, sizeof(socket_table));

    /* Register stream and datagram protocols */
    if (linklayer_register_protocol(PING_PROTOCOL,           ping_packet_process,          ping_packet_format)        &&
        linklayer_register_protocol(PINGREPLY_PROTOCOL,      pingreply_packet_process,     pingreply_packet_format)   &&
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
        linklayer_register_protocol(STREAM_PROTOCOL,         stream_packet_process,        stream_packet_format)      &&
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
        linklayer_register_protocol(DATAGRAM_PROTOCOL,       datagram_packet_process,      datagram_packet_format)) {

        global_socket_lock = os_create_recursive_mutex();

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
        /* Create the socket_scanner thread */
        socket_scanner_thread_id = os_create_thread(socket_scanner_thread, "socket_scanner", SOCKET_SCANNER_STACK_SIZE, 0, NULL);
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

        esp_vfs_t vfs = {
            .flags    = ESP_VFS_FLAG_DEFAULT,
            .read = &ls_read,
            .write = &ls_write,
            .open = NULL,
            .close = &ls_close,
            .fstat = &ls_fstat,
            .fcntl = &ls_fcntl,
            .ioctl = &ls_ioctl_r_wrapper,
            // 'select' / 'poll' will come later
            // .socket_select = &lwip_select,
            // .get_socket_select_semaphore = &lwip_get_socket_select_semaphore,
            // .stop_socket_select = &lwip_stop_socket_select,
            // .stop_socket_select_isr = &lwip_stop_socket_select_isr,
        };

        /* Initialize vfs for socket range */
        err = esp_vfs_register_fd_range(&vfs, NULL, FIRST_LASTLINK_FD, LAST_LASTLINK_FD);

    } else {
        err = LSE_CANNOT_REGISTER;
    }

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    add_command("dglisten",   dglisten_command);
    add_command("dgsend",     dgsend_command);
    add_command("dgcon",      dgcon_command);
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
    add_command("stconsumer", stconsumer_command);
    add_command("stproducer", stproducer_command);
    add_command("stwrite",    stwrite_command);
    add_command("stread",     stread_command);
    add_command("stconcycle", stconcycle_command);
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
    add_command("ping",       ping_command);
    add_command("pt",         print_ping_table);
    add_command("s",          print_sockets);
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

    return err;
}

ls_error_t ls_socket_deinit(void)
{
#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    remove_command("dglisten");
    remove_command("dgsend");
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
    remove_command("stconsumer");
    remove_command("stproducer");
    remove_command("stwrite");
    remove_command("stread");
    remove_command("stconcycle");
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
    remove_command("ping");
    remove_command("pt");
    remove_command("s");
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
    /* Kill socket scanner */
    os_delete_thread(socket_scanner_thread_id);
    socket_scanner_thread_id = NULL;
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

    /* Purge all waiting pings */
    for (int ping = 0; ping < ELEMENTS_OF(ping_table); ++ping) {
        if (ping_table[ping].sequence != 0) {
            ping_table[ping].sequence = 0;
            if (ping_table[ping].queue != NULL) {
                os_delete_queue(ping_table[ping].queue);
                ping_table[ping].queue = NULL;
            }
        }
    }

    /* Close all sockets */
    for (int s = 0; s < ELEMENTS_OF(socket_table); ++s) {
        (void) release_socket(&socket_table[s]);
    }

    /* De-register protocols */
    ls_error_t err = 0;

    /* Register stream and datagram protocols */
    if (linklayer_unregister_protocol(PING_PROTOCOL)        &&
        linklayer_unregister_protocol(PINGREPLY_PROTOCOL)   &&
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
        linklayer_unregister_protocol(STREAM_PROTOCOL)      &&
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
        linklayer_unregister_protocol(DATAGRAM_PROTOCOL)) {

        /* OK */

    } else {
        err = LSE_CANNOT_REGISTER;
    }

    GLOBAL_SOCKET_LOCK();
    /* Delete extra stuff on socket */
    for (int s = 0; s < ELEMENTS_OF(socket_table); ++s) {
        os_delete_mutex(socket_table[s].lock);
        socket_table[s].lock = NULL;
    }
    GLOBAL_SOCKET_UNLOCK();

    os_delete_mutex(global_socket_lock);
    global_socket_lock = NULL;

    packet_window_deinit();

    return err;
}

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_LAYER */
