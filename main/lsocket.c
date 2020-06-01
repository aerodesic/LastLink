/*
 * TODO:
 *  When receiving an empty non-eor packet, don't bother putting it into the receive window.
 *  Just pretend it went in by advancing sequence numbers.  This might be only possible if it
 *  is empty.
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

#ifdef CONFIG_LASTLINK_ENABLE_SOCKET_LAYER
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

#if CONFIG_LASTLINK_TABLE_COMMANDS
#include "commands.h"
#endif

#define TAG "lsocket"

/* PING support */
#define PING_RETRY_STACK_SIZE    6000

static const char* ping_packet_format(const packet_t *packet);
static const char* pingreply_packet_format(const packet_t *packet);
static bool ping_packet_process(packet_t *packet);
static bool pingreply_packet_process(packet_t *packet);
static int find_ping_table_entry(int sequence);

static ls_error_t ls_write_helper(ls_socket_t *socket, const char* buf, size_t len, bool eor);
static ls_socket_t * validate_socket(int s);

static bool stream_packet_process(packet_t *packet);
static const char* stream_packet_format(const packet_t *packet);
#if 0
static packet_t *assemble_socket_data(packet_t *packet_to_send, packet_t *packet);
#endif

static int ls_ioctl_r_wrapper(int fd, int cmd, va_list args);

static const char* socket_type_of(const ls_socket_t *socket);
static ls_error_t ls_dump_socket_ptr(const char* msg, const ls_socket_t *socket);

static bool datagram_packet_process(packet_t *packet);
static const char* datagram_packet_format(const packet_t *packet);

#ifdef NOTUSED
static packet_t *datagram_packet_create_from_packet(const packet_t *packet);
#endif
static packet_t *datagram_packet_create_from_socket(const ls_socket_t *socket);

static packet_t *stream_packet_create_from_packet(const packet_t *packet, uint8_t flags, int sequence);
static packet_t *stream_packet_create_from_socket(const ls_socket_t *socket, uint8_t flags, int sequence);

static ls_socket_t *find_socket_from_packet(const packet_t *packet, ls_socket_type_t type);
static ls_socket_t *find_listening_socket_from_packet(const packet_t *packet);

static ssize_t release_socket(ls_socket_t* socket);

#define SOCKET_SCANNER_STACK_SIZE          6000
#define SOCKET_SCANNER_DEFAULT_INTERVAL    100
static os_thread_t socket_scanner_thread_id;
static void socket_scanner_thread(void*);

static void start_state_machine(ls_socket_t *socket, ls_socket_state_t state, bool (*action)(ls_socket_t *socket), void (*results)(ls_socket_t* socket, ls_error_t error),  int timeout, int retries);
static void cancel_state_machine(ls_socket_t *socket);
static void state_machine_timeout(ls_socket_t *socket);
static void send_state_machine_results(ls_socket_t *socket, ls_error_t response);
static ls_error_t get_state_machine_results(ls_socket_t *socket);
static void ack_output_window(ls_socket_t *socket, int ack_sequence, unsigned int ack_window);
static void send_output_window(ls_socket_t *socket, bool forceack);
static bool socket_linger_check(ls_socket_t *socket);
static void socket_linger_finish(ls_socket_t *socket, ls_error_t error);

static ls_error_t ls_close_ptr(ls_socket_t *socket);

/* These two deal with incoming packets net -> packet -> window -> user */
static bool receive_packet_to_window(packet_window_t *window, packet_t *packet);
static packet_t *read_packet_from_window(packet_window_t *window);

/* These two deal with outgoing packets user -> packet -> window -> net */
static bool write_packet_to_window(packet_window_t *window, packet_t *packet, bool wait);
static bool release_packets_in_window(packet_window_t *window, int sequence, unsigned int packet_mask);

static bool socket_flush_current_packet(ls_socket_t *socket, bool wait);

/* Compute the ack sequence and residue to acknowledge all packets consumed by window */
static bool get_ack_window(packet_window_t *window, int* sequence, unsigned int *ack_window);

static packet_window_t *allocate_packet_window(int length, int available);
static bool release_packet_window(packet_window_t *window);

typedef struct ping_table_entry {
    os_queue_t       queue;
    int              sequence;
    packet_t         *packet;
    os_thread_t      thread;
    bool             waiting;
    int              retries;
    bool             routed;
    ls_error_t       error;
    simpletimer_t    pingtime;
} ping_table_entry_t;

ping_table_entry_t   ping_table[CONFIG_LASTLINK_MAX_OUTSTANDING_PINGS];

static os_mutex_t    global_socket_lock;

static ls_socket_t   socket_table[CONFIG_LASTLINK_NUMBER_OF_SOCKETS];

#if 1
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
#else
#define GLOBAL_SOCKET_LOCK()    os_acquire_recursive_mutex(global_socket_lock)
#define GLOBAL_SOCKET_UNLOCK()  os_release_recursive_mutex(global_socket_lock)
#endif


#if 1
static inline bool lock_socket_debug(ls_socket_t* socket, bool try, const char *file, int line)
{
    /* Try without waiting */
    bool ok = os_acquire_recursive_mutex_with_timeout(socket->lock, 0);
    if (!ok) {
        /* Didn't get it, so if we are not just 'trying', go ahead and wait and show message */
        if (!try) {
            ESP_LOGI(TAG, "%s: waiting for socket %d at lock [%s:%d] last locked at %s:%d", __func__, socket - socket_table, file, line, socket->last_lock_file ? socket->last_lock_file : "<NULL>", socket->last_lock_line);
            ok = os_acquire_recursive_mutex(socket->lock);
        }
    }

    if (ok) {
        socket->last_lock_file = file;
        socket->last_lock_line = line;
    }

    return ok;
}
#define LOCK_SOCKET(socket)      lock_socket_debug(socket, false, __FILE__, __LINE__)
#define LOCK_SOCKET_TRY(socket)  lock_socket_debug(socket, true, __FILE__, __LINE__)

static inline void unlock_socket_debug(ls_socket_t *socket, const char *file, int line)
{
    os_release_recursive_mutex(socket->lock);
}
#define UNLOCK_SOCKET(socket)    unlock_socket_debug(socket, __FILE__, __LINE__)
#else
#define LOCK_SOCKET(socket)      os_acquire_recursive_mutex((socket)->lock)
#define LOCK_SOCKET_TRY(socket)  os_acquire_recursive_mutex_with_timeout((socket)->lock)
#define UNLOCK_SOCKET(socket)    os_release_recursive_mutex((socket)->lock)
#endif

 
static const char* socket_type_of(const ls_socket_t *socket)
{
    const char* type = "UNDEFINED";
    if (socket != NULL) {
        switch (socket->socket_type) {
            case LS_UNKNOWN:  type = "Unknown";  break;
            case LS_DATAGRAM: type = "Datagram"; break;
            case LS_STREAM:   type = "Stream";   break;
            default:          type = "Unknown";  break;
        }
    }

    return type;
}
        
static const char* socket_state_of(const ls_socket_t *socket)
{
    switch (socket->state) {
        case LS_STATE_IDLE:                     return "LS_STATE_IDLE";
        case LS_STATE_INBOUND_CONNECT:          return "LS_STATE_INBOUND_CONNECT";
        case LS_STATE_OUTBOUND_CONNECT:         return "LS_STATE_OUTBOUND_CONNECT";
        case LS_STATE_CONNECTED:                return "LS_STATE_CONNECTED";
        case LS_STATE_DISCONNECTING_FLUSH:      return "LS_STATE_DISCONNECTING_FLUSH";
        case LS_STATE_INBOUND_DISCONNECTING:    return "LS_STATE_INBOUND_DISCONNECTING";
        case LS_STATE_OUTBOUND_DISCONNECTING:   return "LS_STATE_OUTBOUND_DISCONNECTING";
        case LS_STATE_DISCONNECTED:             return "LS_STATE_DISCONNECTED";
        case LS_STATE_LINGER:                   return "LS_STATE_LINGER";
        default:                                return "UNKNOWN STATE";
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

#if 0
            /* Give it a new serial number */
            set_uint_field(packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());
#endif

            /* Turn the packet around */
            set_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN, get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN));
            set_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
            set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);   /* Force packet to be routed */

            /* Rewind Metric */
            set_uint_field(packet, HEADER_METRIC, METRIC_LEN, 0);

            linklayer_route_packet(ref_packet(packet));

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
                os_delete_thread(ping_table[slot].thread);
                ping_table[slot].waiting = false;
                ping_table[slot].thread = NULL;
                if (!os_put_queue(ping_table[slot].queue, ref_packet(packet))) {
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
        os_get_queue(ping_table[slot].queue, (os_queue_item_t*) &packet);
        /* Allow the thread to die */
        ping_table[slot].waiting = false;
    }

    return packet;
}

static void release_ping_table_entry(int slot)
{
    linklayer_lock();

    if (ping_table[slot].thread != NULL) {
        os_delete_thread(ping_table[slot].thread);
        ping_table[slot].thread = NULL;
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

#if 0
            /* Retransmit packet with new sequence number */
            set_uint_field(ping_table[slot].packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());
#endif

            linklayer_route_packet(ref_packet(ping_table[slot].packet));

        } else {
            /* Either timed out or was told to quit. */
            done = true;

            /* If we timed out - return error */
            if (ping_table[slot].waiting) {
                ping_table[slot].error = LSE_TIMEOUT;
                os_put_queue(ping_table[slot].queue, (os_queue_t) NULL);
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
static bool ping_has_been_routed(packet_t* packet, void* data)
{
    int slot = (int) slot;

    linklayer_lock();

    // ESP_LOGI(TAG, "%s: for slot %d", __func__, slot);

    if (packet != NULL) {
        /* Create a timer to retry trasmission for a while */
        ping_table[slot].retries = CONFIG_LASTLINK_PING_RETRIES;
        ping_table[slot].waiting = true;
        ping_table[slot].thread = os_create_thread(ping_retry_thread, "ping_retry", PING_RETRY_STACK_SIZE, 0, (void*) slot);
        ping_table[slot].routed = true;
        simpletimer_start(&ping_table[slot].pingtime, CONFIG_LASTLINK_PING_RETRY_TIMER * 2);

        /* We are done with it */
        release_packet(packet);

    } else if (ping_table[slot].queue != NULL) {
        /* No route */
        os_put_queue(ping_table[slot].queue, (os_queue_t) NULL);
        ping_table[slot].error = LSE_NO_ROUTE;
    }

    linklayer_unlock();

    /* Allow packet to be sent */
    return true;
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

            linklayer_route_packet(packet);

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

    GLOBAL_SOCKET_LOCK();

    for (int index = 0; socket == NULL && index < ELEMENTS_OF(socket_table); ++index) {

        socket_index = (socket_index + 1) % ELEMENTS_OF(socket_table);

        os_acquire_recursive_mutex(socket_table[socket_index].lock);

        if (!socket_table[socket_index].inuse && socket_table[socket_index].state == LS_STATE_IDLE) {
            socket = &socket_table[socket_index];
            socket->inuse = true;
        } else {
            os_release_recursive_mutex(socket_table[socket_index].lock);
        }
    }

    GLOBAL_SOCKET_UNLOCK();

    return socket;
}

static int next_local_port = 5000;

static int find_free_local_port(void) {

    int found = -1;

    GLOBAL_SOCKET_LOCK();

    bool collision;

    do {
        collision = false;

        /* Increment local port number pool */
        if (++next_local_port >= 65536) {
            next_local_port = 5000;
        }

        /* Go through all sockets to see if we have used this port */
        for (int s = 0; !collision && s < ELEMENTS_OF(socket_table); ++s) {
            if (next_local_port == socket_table[s].local_port) {
                collision = true;
            }
        }
    } while (collision);

    /* We will use this one */
    found = next_local_port;

    GLOBAL_SOCKET_UNLOCK();

    return found;
}


/* Look in socket table for socket matching the connection and type */
static ls_socket_t *find_socket_from_packet(const packet_t *packet, ls_socket_type_t type)
{
    ls_socket_t *socket = NULL;

    ls_port_t dest_port = get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN);
    ls_port_t src_port  = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN);
    int dest_addr       = get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN);

//printf("find_socket_from_packet: addr %d src port %d dest port %d\n", dest_addr, src_port, dest_port);

    for (int index = 0; socket == NULL && index < CONFIG_LASTLINK_NUMBER_OF_SOCKETS; ++index) {
        if (socket_table[index].socket_type == type) {
//ls_dump_socket_ptr("testing", socket_table + index);
            if ( !socket_table[index].listen &&
                (socket_table[index].dest_addr == 0 || socket_table[index].dest_addr == BROADCAST_ADDRESS || socket_table[index].dest_addr == dest_addr) &&
                socket_table[index].local_port == dest_port &&
                (socket_table[index].dest_port == 0 || socket_table[index].dest_port == src_port)) {

//ls_dump_socket_ptr("found", socket_table + index);
                socket = &socket_table[index];
            }
        }
    }

    /* Return socket if we found it */
    return socket;
}

static ls_socket_t *find_listening_socket_from_packet(const packet_t *packet)
{
    ls_socket_t *socket = NULL;

    ls_port_t dest_port = get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN);

    for (int index = 0; socket == NULL && index < ELEMENTS_OF(socket_table); ++index) {
        /* A socket listening on the specified port is all we need */
        if (socket_table[index].listen && socket_table[index].local_port == dest_port) {
            socket = &socket_table[index];
        }
    }

    /* Return socket if we found it */
    return socket;
}

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
#endif

static packet_t *datagram_packet_create_from_socket(const ls_socket_t *socket)
{
    packet_t *packet = linklayer_create_generic_packet(socket->dest_addr, DATAGRAM_PROTOCOL, DATAGRAM_LEN);

    if (packet != NULL) {
        set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN, socket->dest_port);
        set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN, socket->local_port);
    }

    return packet;
}

static bool datagram_packet_process(packet_t *packet)
{
    bool handled = false;

    if (packet != NULL) {
//linklayer_print_packet("dgprocess", packet);
        if (linklayer_packet_is_for_this_node(packet) || get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN) == BROADCAST_ADDRESS) {
//ESP_LOGD(TAG, "%s: finding socket for this packet", __func__);
            ls_socket_t *socket = find_socket_from_packet(packet, LS_DATAGRAM);

//ls_dump_socket_ptr("inbound packet for", socket);

            if (socket != NULL) {

                /* Send the packet to the datagram input queue */
                if (!os_put_queue(socket->datagram_packets, (os_queue_item_t) ref_packet(packet))) {
                    release_packet(packet);
                }
            }

            /* say 'handled' it if directed to us (i.e. don't retransmit it) */
            handled = linklayer_packet_is_for_this_node(packet);;
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

static packet_t *stream_packet_create_from_socket(const ls_socket_t *socket, uint8_t flags, int sequence)
{
    packet_t *packet = linklayer_create_generic_packet(socket->dest_addr, STREAM_PROTOCOL, STREAM_LEN);

    if (packet != NULL) {
        set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUMBER_LEN, socket->dest_port);
        set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN, socket->local_port);
        set_uint_field(packet, STREAM_FLAGS, FLAGS_LEN, flags);
        set_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
    }

    return packet;
}

#if 0
static packet_t *assemble_socket_data(packet_t *packet_to_send, packet_t *packet)
{
    return packet_to_send;
}
#endif

/* State machine actions */
static bool stream_flush_output_window(ls_socket_t *socket)
{
    /* Flush output window until we are done */
    if (socket_flush_current_packet(socket, /*wait*/ false)) {

ESP_LOGI(TAG, "%s: next_in is %d", __func__, socket->output_window->next_in);

        if (socket->output_window->next_in != 0) {
            send_output_window(socket, /* force_ack */ true);
        }
    }

    /* Success when the output window is empty */
    return socket->output_window->next_in == 0;
}

static bool stream_send_connect(ls_socket_t *socket)
{
    linklayer_route_packet(stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_CONNECT, 0));
    return false;
}

static bool stream_send_connect_ack(ls_socket_t *socket)
{
    linklayer_route_packet(stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_CONNECT_ACK, socket->input_window->length));
    return false;
}

static bool stream_send_disconnect(ls_socket_t *socket)
{
    linklayer_route_packet(stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_DISCONNECT, 0));
    return false;
}

static bool stream_send_disconnected(ls_socket_t *socket)
{
    linklayer_route_packet(stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_DISCONNECTED, 0));
    return false;
}

static bool stream_packet_process(packet_t *packet)
{
    bool handled = false;

    bool reject = true;
    bool resend_output_window = false;

linklayer_print_packet("stream packet", packet);

    if (packet != NULL) {
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
                    reject = false;
                    break;
                }

                case STREAM_FLAGS_CMD_DATA_EOR:
                case STREAM_FLAGS_CMD_DATA: {
                    /*
                     * Pass the data through the assembly phase.  This might require modifying the current outbound packet
                     * to update ack sequence numbers, etc.  In some cases a brand new packet will be generated.
                     *
                     * If there is no data in this packet, it will be ignored if it is the first packet in the queue.
                     */
                    if (socket != NULL) {
                        if (receive_packet_to_window(socket->input_window, packet) &&
                            (cmd == STREAM_FLAGS_CMD_DATA) &&
                            (packet->length > STREAM_PAYLOAD)) {

                            /* A packet with data - after we process ack, send any residual window (plus acks) */
                            resend_output_window = true;
                        }
                        reject = false;
                    }
                    break;
                }

                case STREAM_FLAGS_CMD_CONNECT: {
                    /* First see if this is a connect on a socket that is already connecting */
                    if (socket != NULL) {
                        /* Cancel current state machine retry */
                        cancel_state_machine(socket);
                        if (socket->state == LS_STATE_INBOUND_CONNECT) {
                            /* Redundant CONNECT gets a CONNECT_ACK */
                            stream_send_connect_ack(socket);
                            reject = false;
                        }
                    } else {
                        /* Didn't find actual socket, so see if this is a new listen */
                        ls_socket_t *listen_socket = find_listening_socket_from_packet(packet);

                        if (listen_socket != NULL) {
                            ls_socket_t *new_connection = find_free_socket();

                            if (new_connection != NULL) {

                                /* Create a new connection that achievs a local endpoint for the new socket */
                                new_connection->socket_type = LS_STREAM;

                                new_connection->state = LS_STATE_INBOUND_CONNECT;
                                new_connection->local_port = listen_socket->local_port;
                                new_connection->dest_port = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUMBER_LEN);
                                new_connection->dest_addr = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
                                new_connection->parent = listen_socket;
                                new_connection->input_window = allocate_packet_window(CONFIG_LASTLINK_STREAM_WINDOW_SIZE, 0);
                                new_connection->output_window = allocate_packet_window(CONFIG_LASTLINK_STREAM_WINDOW_SIZE, CONFIG_LASTLINK_STREAM_WINDOW_SIZE);

ls_dump_socket_ptr("new connection", new_connection);
    
                                /* Send a CONNECT ACK and go to INBOUND CONNECT state waiting for full acks */
//ESP_LOGI(TAG, "%s: start_state_machine new connection INBOUND_CONNECT stream_send_connect_ack on socket %d", __func__, new_connection - socket_table);

                                start_state_machine(new_connection,
                                                    LS_STATE_INBOUND_CONNECT,
                                                    stream_send_connect_ack,     /* Send connect ack */
                                                    NULL,                        /* When we receive connectack, send to new_connection */
                                                    STREAM_CONNECT_TIMEOUT,
                                                    STREAM_CONNECT_RETRIES);

                                os_release_recursive_mutex(new_connection->lock);

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
                                socket->input_window = allocate_packet_window(CONFIG_LASTLINK_STREAM_WINDOW_SIZE, 0);
                            }
                            if (socket->output_window == NULL) {
                                socket->output_window = allocate_packet_window(CONFIG_LASTLINK_STREAM_WINDOW_SIZE, CONFIG_LASTLINK_STREAM_WINDOW_SIZE);
                            }

                            /* Receiving a CONNECT ACK in the OUTBOUND connect, so send a CONNECT ACK and go the CONNECTED state */
                            stream_send_connect_ack(socket);
                            socket->state = LS_STATE_CONNECTED;

                            /* Deliver success to caller of original connect */
                            send_state_machine_results(socket, LSE_NO_ERROR);
                            reject = false;

                        } else if (socket->state == LS_STATE_INBOUND_CONNECT) {
                            socket->state = LS_STATE_CONNECTED;
                            /* Send the socket number to the waiting listener */
                            if (socket->parent != NULL) {
                                /* Put in the connection queue but if overflowed, reject the connection */
                                if(os_put_queue_with_timeout(socket->parent->connections, socket, 0)) {
                                    reject = false;
                                }
                            } else {
                                /* ERROR connection has no parent.  Just release the socket and reject connection attempt. */
                                release_socket(socket);
                            }
                            /* Let ls_listen finish it's work */
                            send_state_machine_results(socket, LSE_NO_ERROR);

                        /* If an extra connect-ack in connected state, don't reject and send the expected connect ack */
                        } else if (socket->state == LS_STATE_CONNECTED) {
                            stream_send_connect_ack(socket);
                            reject = false;
                        }

                        int length = get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN);
                        if (length != 0 && length < socket->output_window->length) {
ESP_LOGI(TAG, "%s: output_window changed from %d to %d for socket %d", __func__, socket->output_window->length, length, socket - socket_table);
                            socket->output_window->length = length;
                        }
                    }
                    break;
                }

                case STREAM_FLAGS_CMD_DISCONNECT: {
                    /* If still in connecting, respond with connect ack */
                    if (socket != NULL) {
                        if (socket->state == LS_STATE_CONNECTED) {

                            /* Start a disconnect */
                            socket->state = LS_STATE_INBOUND_DISCONNECTING;
                        
                            /* This will shut down the reader when it rises to the top of the packet list */
                            receive_packet_to_window(socket->input_window, NULL);
                        }

                        linklayer_route_packet(stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_DISCONNECTED, 0));
               
                        reject = false;
                    }

                    break;
                }

                case STREAM_FLAGS_CMD_DISCONNECTED: {
                    /* If are in connecting, respond with connect ack */
                    if (socket != NULL) {
                        if (socket->state == LS_STATE_INBOUND_DISCONNECTING || socket->state == LS_STATE_OUTBOUND_DISCONNECTING) {

                            if (socket->state == LS_STATE_OUTBOUND_DISCONNECTING) {
                                /* Stop the disconnect machine */
                                send_state_machine_results(socket, LSE_NO_ERROR);
                            }

                            socket->state = LS_STATE_DISCONNECTED;

                            /* Just echo back disconnected */
                            linklayer_route_packet(stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_DISCONNECTED, 0));
                        }
                    }
                    reject = false;
                    break;
                }

                case STREAM_FLAGS_CMD_REJECT: {
                    /* If are in connecting, respond with connect ack */
                    if (socket != NULL) {

//ESP_LOGI(TAG, "%s: start_state_machine CMD_REJECT DISCONNECTED stream_send_disconnected on socket %d", __func__, socket - socket_table);
                        /* An error so tear down the connection */
                        start_state_machine(socket,
                                            LS_STATE_DISCONNECTED,
                                            stream_send_disconnected,
                                            NULL,
                                            STREAM_DISCONNECT_TIMEOUT,
                                            STREAM_DISCONNECT_RETRIES);

                        reject = false;
                    }
                    break;
                }
            }
    
            if (reject) {
                linklayer_route_packet(stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_REJECT, 0));
            } else if (socket != NULL) {
                /* Ack the output window if acknum is present */
                if  (flags & STREAM_FLAGS_ACKNUM) {
                    ack_output_window(socket,
                                      get_uint_field(packet, STREAM_ACK_SEQUENCE, SEQUENCE_NUMBER_LEN),
                                      get_uint_field(packet, STREAM_ACK_WINDOW, ACK_WINDOW_LEN));
                }
                if (resend_output_window) {
                    send_output_window(socket, /* force_ack */ true);
                }
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



static ssize_t release_socket(ls_socket_t* socket)
{
    ssize_t err = LSE_NO_ERROR;

    LOCK_SOCKET(socket);
 
    switch (socket->socket_type) {
        case LS_UNKNOWN: {
            err = LSE_NOT_OPENED;
            break;
        }

        case LS_DATAGRAM: {
            /* Close packet receive queue */
            linklayer_release_packets_in_queue(socket->datagram_packets);
            if (socket->datagram_packets != NULL) {
                os_delete_queue(socket->datagram_packets);
                socket->datagram_packets = NULL;
            }
            socket->socket_type = LS_UNKNOWN;
            break;
        }

        case LS_STREAM: {
            if (socket->listen) {
                /* Go through all sockets and bust the parent link to this listener */
                for (int socknum = 0; socknum < ELEMENTS_OF(socket_table); ++socknum) {
                    if (socket_table[socknum].socket_type == LS_STREAM && socket_table[socknum].parent == socket) {
                        socket_table[socknum].parent = NULL;
                    }
                }

                /* Close listener queue */
                ls_socket_t *c;
                while (os_get_queue_with_timeout(socket->connections, (os_queue_item_t*) &c, 0)) {
                    ls_close_ptr(c);
                }

                os_delete_queue(socket->connections);
                socket->connections = NULL;
                socket->socket_type = LS_UNKNOWN;
            } else {
ESP_LOGI(TAG, "%s: releasing socket %d", __func__, socket - socket_table);
                simpletimer_stop(&socket->state_machine_timer);
                simpletimer_stop(&socket->socket_flush_timer);
                simpletimer_stop(&socket->output_window_timer);

                /* Otherwise forcibly shut everything down */
                release_packet_window(socket->input_window);
                socket->input_window = NULL;
                release_packet_window(socket->output_window);
                socket->output_window = NULL;

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

                socket->state = LS_STATE_IDLE;
                socket->socket_type = LS_UNKNOWN;
            }
            break;
        }
    }

    socket->inuse = false;
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
    if (socket_type == LS_DATAGRAM || socket_type == LS_STREAM) {

        ls_socket_t *socket = find_free_socket();

ESP_LOGI(TAG, "find_free_socket returned %p", socket);

        if (socket != NULL) {
            socket->socket_type = socket_type;
            socket->state = LS_STATE_SOCKET;
            socket->local_port = -1;

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
ls_error_t ls_bind(int s, ls_port_t local_port)
{
    ls_error_t err = LSE_INVALID_SOCKET;

    ls_socket_t *socket = validate_socket(s);

    if (socket != NULL) {
        
        /* Assign local port from free pool if not specified by user */
        socket->local_port = local_port;

        UNLOCK_SOCKET(socket);

        err = LSE_NO_ERROR;
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

    ls_socket_t *socket = validate_socket(s);

    /* Validate paramters */
    if (socket != NULL) {

        bool release = true;

        if (socket->busy != 0) {
            err = LSE_SOCKET_BUSY;
        } else {
            socket->busy = true;

            if (socket->socket_type == LS_STREAM) {

                if (max_queue > 0 && max_queue < MAX_SOCKET_CONNECTIONS) {

                    /* Place socket in listen mode */
                    if (socket->connections == NULL) {
                        /* Create queue for connections */
                        socket->connections = os_create_queue(max_queue, sizeof(ls_socket_t*));
    
                        socket->listen = true;
                    }

                    release = false;

                    UNLOCK_SOCKET(socket);

                    /* Wait for something to arrive at connection queue */
                    ls_socket_t *connection;
                    if (os_get_queue_with_timeout(socket->connections, (os_queue_item_t*) &connection, timeout)) {
                        /* Wait for the connect ack to finish */
                        err = get_state_machine_results(connection);

                        /* A new connection */
                        err = connection - socket_table + FIRST_LASTLINK_FD;
                    } else {
                        err = LSE_TIMEOUT;
                    }
                } else {
                    err = LSE_INVALID_MAXQUEUE;
                }
            } else {
                err = LSE_INVALID_SOCKET;
            }

            socket->busy = false;
        }

        if (release) {
            UNLOCK_SOCKET(socket);
        }

    } else {
        err = LSE_INVALID_SOCKET;
    }

    return err;
}

/* Called by repeating timer to keep machine running - retransmit data, etc. */
static void state_machine_timeout(ls_socket_t *socket)
{
    if (--(socket->state_machine_retries) > 0) {

        assert(socket->state_machine_action != NULL);

        /* Timed out so retry by sending connect packet again (if still present) */
        if (socket->state_machine_action != NULL && socket->state_machine_action(socket)) {
ESP_LOGI(TAG, "%s: sending results %d to socket %d", __func__, LSE_NO_ERROR, socket - socket_table);
           send_state_machine_results(socket, LSE_NO_ERROR);
        } else {
            simpletimer_restart(&socket->state_machine_timer);
ESP_LOGI(TAG, "%s: restarted state_machine_timer for %d ms", __func__, simpletimer_remaining(&socket->state_machine_timer));
        }
    } else {
ESP_LOGI(TAG, "%s: retries failed", __func__);
        /* Send error code to connect response queue */
        send_state_machine_results(socket, LSE_CONNECT_FAILED);
    }
}

/*
 * start_state_machine
 *
 * Used for connecting, disconnecting and outbound data transmission.
 *
 * Entry:
 *      socket       Socket to run state machine
 *      state        State to assign socket
 *      action       Action to take
 *      results      Function to receive results (NULL for response queue)
 *      timeout      Timeout between retry cycles. (if 0, does not start timer)
 *      retries      Number of times to retry packet until state changes.
 */
static void start_state_machine(ls_socket_t *socket, ls_socket_state_t state, bool (*action)(ls_socket_t* socket), void (*results)(ls_socket_t* socket, ls_error_t error), int timeout, int retries)
{
    assert(action != NULL);

    LOCK_SOCKET(socket);

    /* Cancel any running state machine */
    cancel_state_machine(socket);

    if (state >= 0) {
        socket->state = state;
    }

    socket->state_machine_results = results;

    if (results == NULL) {
        /* Results goes to response queue */
        if (socket->state_machine_results_queue != NULL) {
            os_delete_queue(socket->state_machine_results_queue); 
        }
        socket->state_machine_results_queue = os_create_queue(1, sizeof(ls_error_t));
    }

    /* Do the action and if failed, start the timer and repeat until success or timeout */
    if (action(socket)) {
ESP_LOGI(TAG, "%s: state %s action succeeded on socket %d", __func__, socket_state_of(socket), socket - socket_table);
        /* Ready now, so move on */
        send_state_machine_results(socket, LSE_NO_ERROR);
    } else if (timeout != 0) {
        socket->state_machine_retries = retries;
ESP_LOGI(TAG, "%s: state %s setting timeout to %d for %d tries on socket %d", __func__, socket_state_of(socket), timeout, retries, socket - socket_table);
        simpletimer_start(&socket->state_machine_timer, timeout);
        socket->state_machine_action = action;
    }

    UNLOCK_SOCKET(socket); 
}

/*
 * Send state machine results back to caller.
 *
 * If the state_machine_results member is NULL, then we send it to
 * a (required) response queue for caller.
 *
 * If the state_machine_results is not NULL, it is called with the
 * socket and results code.
 */
void send_state_machine_results(ls_socket_t *socket, ls_error_t response)
{
    cancel_state_machine(socket);

    if (socket->state_machine_results == NULL) {
        assert(socket->state_machine_results_queue != NULL);
        os_put_queue(socket->state_machine_results_queue, (os_queue_item_t) response);
    } else {
        socket->state_machine_results(socket, response);
    }
}

/*
 * Used by entity starting state machine to pend waiting for
 * the results of the operation.  Not used for functional
 * finishing of the state machine action.
 */
ls_error_t get_state_machine_results(ls_socket_t *socket)
{
    ls_error_t response;

    assert(socket->state_machine_results == NULL);
    assert(socket->state_machine_results_queue != NULL);

    if (socket->state_machine_results_queue != NULL) {
        if (! os_get_queue(socket->state_machine_results_queue, (os_queue_item_t*) &response)) {
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

static void cancel_state_machine(ls_socket_t *socket)
{
ESP_LOGI(TAG, "%s: stopped on socket %d", __func__, socket - socket_table);
    socket->state_machine_retries = 0;
    socket->state_machine_action = NULL;

    simpletimer_stop(&socket->state_machine_timer);

    /* Leave response queue in place so receiver gets the message */
}


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
    ls_error_t err = LSE_NO_ERROR;

    ls_socket_t *socket = validate_socket(s);

    if (socket != NULL) {

        if (socket->busy) {
            err = LSE_SOCKET_BUSY;
        } else {
            socket->busy = true;
            socket->dest_port = port;
            socket->dest_addr = address;

            /* If localport is 0, bind an unused local port to this socket */
            if (socket->local_port == 0) {
                socket->local_port = find_free_local_port();
            }
    
            if (socket->socket_type == LS_DATAGRAM) {
                /* Datagram connection is easy - we just say it's connected */
                socket->state = LS_STATE_CONNECTED;
                if (socket->datagram_packets == NULL) {
                    socket->datagram_packets = os_create_queue(5, sizeof(packet_t*));
                }

                /* Special case: our dest address changes to the last destination of the packet we receive.  */
                socket->rename_dest = address == NULL_ADDRESS;

            } else if (socket->socket_type == LS_STREAM) {
//ESP_LOGI(TAG, "%s: start_state_machine ls_connect OUTBOUND_CONNECT stream_send_connect on socket %d", __func__, socket - socket_table);
                /* Start sending connect packet and wait for complete */
                start_state_machine(socket,
                                    LS_STATE_OUTBOUND_CONNECT,
                                    stream_send_connect,
                                    NULL,
                                    STREAM_CONNECT_TIMEOUT,
                                    STREAM_CONNECT_RETRIES);


                UNLOCK_SOCKET(socket);

                /* A response of some kind will be forthcoming.  It will be NO_ERROR or some other network error */
                err = get_state_machine_results(socket);

                LOCK_SOCKET(socket);
            
            } else {
                err = LSE_INVALID_SOCKET;
            }

            socket->busy = false;
        }

        UNLOCK_SOCKET(socket);

    } else {
        err = LSE_INVALID_SOCKET;
    }

    if (err != LSE_INVALID_SOCKET) {
        socket->last_error = err;
    }

    return err;
}

/*
 * Flush the current output packet if one exists.
 *
 * Start a timer to send the otuput window after a delay.
 *
 * Return true if there are unacknowledged packets in output window.
 */
static bool socket_flush_current_packet(ls_socket_t *socket, bool wait)
{
    bool more = false;

    if (socket->output_window != NULL && socket->current_write_packet != NULL) {
        if (write_packet_to_window(socket->output_window, ref_packet(socket->current_write_packet), /* wait */ wait)) {
            release_packet(socket->current_write_packet);
            socket->current_write_packet = NULL;
        }
        more = socket->output_window->next_in != 0;
ESP_LOGI(TAG, "%s: start output_window_timer", __func__);
        simpletimer_start(&socket->output_window_timer, CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY);
    }

    return more;
}


/*
 * Send any packets in the output window (unacknowledged) + the ACK sequence and window flags.
 *
 * This acknowledges up to but not including the ack_sequence number + any
 * additional packets beyond in the ack_window.
 *
 * For example:
 *     ACK 0 0  Does not ack anything
 *     ACK 3 4  Acks up to packet sequence 2 plus packet sequence 6 (2 + log2(4) + 1)
 *
 * The LSB of the ack_window represents the first packet W + 2.
 *
 * This allows acknowledging packets received that haven't been actually processed, in
 * order to avoid resending future, already recevied packets.  The above 3/4 implies
 * that packet 0 1 and 2 have been received correctly plus packet 6.
 */
static void ack_output_window(ls_socket_t *socket, int ack_sequence, unsigned int ack_window)
{
    simpletimer_stop(&socket->output_window_timer);

ESP_LOGI(TAG, "%s: waiting for output window lock...", __func__);

    os_acquire_recursive_mutex(socket->output_window->lock);

    /* Release these packets in window */
    if (release_packets_in_window(socket->output_window, ack_sequence, ack_window)) {

ESP_LOGI(TAG, "%s: released_packet_in_window done...", __func__);

        /* Packets remain in window, start the send_output_window timer again */
        socket->output_window->retry_time = 0;
        simpletimer_start(&socket->output_window_timer, CONFIG_LASTLINK_STREAM_TRANSMIT_RETRY_TIME);
    }
          
    os_release_recursive_mutex(socket->output_window->lock);
}

/* Add a data acknowledgement to the outbound data message.
 * Returns the packet.
 */
static packet_t *add_data_ack(packet_window_t *window, packet_t *packet)
{
    unsigned int ack_window;
    int sequence;

    get_ack_window(window, &sequence, &ack_window);

    set_uint_field(packet, STREAM_ACK_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
    set_uint_field(packet, STREAM_ACK_WINDOW, ACK_WINDOW_LEN, ack_window);
    /* Indicate an ack sequence and window are in the payload */
    set_bits_field(packet, STREAM_FLAGS, FLAGS_LEN, STREAM_FLAGS_ACKNUM);

    return packet;
}

/*
 * Send current output window with ack fields.
 *
 * If the output window has packets, send them with the last packet
 * containing the ack information to previous input packets.
 *
 * If no packets are present, send nothing unless 'force_ack' is
 * set, in which case send a naked 'NOP' packet with only the
 * ack information set.
 *
 * In any case, if force_ack is set, do not set repeating attempts
 * to send packets.
 */
static void send_output_window(ls_socket_t *socket, bool force_ack)
{
    simpletimer_stop(&socket->output_window_timer);

ESP_LOGI(TAG, "%s: waiting for window lock", __func__);

    os_acquire_recursive_mutex(socket->output_window->lock);

ESP_LOGI(TAG, "%s: input window next_in is %d", __func__, socket->output_window->next_in);

    /* If packets in window, then send them. */
    if (socket->output_window->next_in != 0) {
        if (!force_ack) {
            /* If first time, set new timeout and retry count */
            if (socket->output_window->retries == 0) {
                socket->output_window->retries = CONFIG_LASTLINK_STREAM_TRANSMIT_RETRIES;
                socket->output_window->retry_time = CONFIG_LASTLINK_STREAM_TRANSMIT_RETRY_TIME;

            } else {
                socket->output_window->retries -= 1;
            }
        }
 
        if (force_ack || socket->output_window->retries != 0) {
ESP_LOGI(TAG, "%s: retries is %d", __func__, socket->output_window->retries);
            int to_send = 0;
            /* Count the packet in the queue */
            for (int slot = 0; slot < socket->output_window->next_in; ++slot) {
                if (socket->output_window->slots[slot] != NULL) {
                    ++to_send;
                }
            }

            /* Packets in queue - send all that are here */
            for (int slot = 0; slot < socket->output_window->next_in; ++slot) {
                packet_t *packet = socket->output_window->slots[slot];

                if (packet != NULL) {
ESP_LOGI(TAG, "%s: sending packet in slot %d", __func__, slot);
//linklayer_print_packet("sending packet", socket->output_window->slots[slot]);

                    if (to_send == 1) {
                        /* Add the current input window ack to the last packet */
                        add_data_ack(socket->input_window, packet);
                    } else {
                        /* Not last - remove ACKNUM */
                        clear_bits_field(packet, STREAM_FLAGS, STREAM_FLAGS_ACKNUM, STREAM_FLAGS_ACKNUM);
                    }

                    linklayer_route_packet(ref_packet(packet));
                    --to_send;
                }
            }

            if (!force_ack) {
                /* Come back in exponential time */
                simpletimer_start(&socket->output_window_timer, socket->output_window->retry_time);

                /* Next time wait twice as long */
                socket->output_window->retry_time <<= 1;
            }
        }
    } else if (force_ack) {
        packet_t *packet = stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_NOP, 0);

        /* Add sequence info to packet */
        add_data_ack(socket->input_window, packet);

        /* Send it */
        linklayer_route_packet(packet);
    }

    os_release_recursive_mutex(socket->output_window->lock);
}

/*
 * Check for socket shutdown
 */
static bool socket_linger_check(ls_socket_t* socket)
{
    bool results = !socket->inuse && socket->socket_type == LS_STREAM && socket->state == LS_STATE_LINGER;
ESP_LOGI(TAG, "%s: returning %s", __func__, results ? "TRUE": "FALSE");
    return results;
}

/*
 * Put socket back to ground state
 */
static void socket_linger_finish(ls_socket_t* socket, ls_error_t error)
{
    if (socket->socket_type == LS_STREAM) {
ESP_LOGI(TAG, "%s: lingering socket released: %d", __func__, error);
        (void) release_socket(socket);
    }
}

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
 *
 * Returns:
 *      ls_error_t          If >0, number of bytes written (might be less than len if error)
 *                          Otherwise is error code (< 0)
 */
static ls_error_t ls_write_helper(ls_socket_t *socket, const char* buf, size_t len, bool eor)
{
    ls_error_t err;

    if (socket == NULL) {
        err = LSE_INVALID_SOCKET;

    } else {
         if (socket->busy) {
             err = LSE_SOCKET_BUSY;
         } else {
             socket->busy = true;

             switch (socket->socket_type) {
                 case LS_DATAGRAM: {
                     packet_t *packet = datagram_packet_create_from_socket(socket);
                     if (packet != NULL) {
                         int tomove = len;
                         if (tomove > DATAGRAM_MAX_DATA) {
                             tomove = DATAGRAM_MAX_DATA;
                         }
                         memcpy(packet->buffer + DATAGRAM_PAYLOAD, buf, tomove);
                         packet->length += tomove;
//linklayer_print_packet("DG OUT", packet);
                         /* Go back to "anything goes" for next inbound packet */
                         if (socket->rename_dest) {
                             socket->dest_addr = NULL_ADDRESS;
                             socket->dest_port = 0;
                         }

                         UNLOCK_SOCKET(socket);
                         linklayer_route_packet(packet);
                         LOCK_SOCKET(socket);

                         err = tomove;
                     } else {
                         err = LSE_NO_MEM;
                     }
                     break;
                 }
     
                 case LS_STREAM: {
                     if (socket->state == LS_STATE_CONNECTED) {
                         int written = 0;
     
                         if (len != 0 || eor) { 
                             while (len != 0) {
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
     
                                 /* Copy data to packet */
                                 memcpy(socket->current_write_packet->buffer + socket->current_write_packet->length, buf, towrite);
                                 socket->current_write_packet->length += towrite;
                                 len -= towrite;
                                 buf += towrite;
                                 written += towrite;
     
                                 /* If more data to send and we've run out of room, send it now. */
                                 if (len != 0 && socket->current_write_packet->length == MAX_PACKET_LEN) {
                                     UNLOCK_SOCKET(socket);
                                     socket_flush_current_packet(socket, /* wait */ true);
                                     LOCK_SOCKET(socket);
                                 }
                             }
     
                             /*
                              * Last packet generated.  If eor, change packet to DATA_EOR.
                              */
                             if (eor || socket->current_write_packet->length == MAX_PACKET_LEN) {
                                 if (eor) {
                                     set_uint_field(socket->current_write_packet, STREAM_FLAGS, FLAGS_LEN, STREAM_FLAGS_CMD_DATA_EOR);
                                 }
                                 UNLOCK_SOCKET(socket);
                                 socket_flush_current_packet(socket, /* wait */ true);
                                 LOCK_SOCKET(socket);
                             } else {
                                 /* Set a timer with CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY to force transmission if nothing else is stored in packet. */
                                 simpletimer_start(&socket->socket_flush_timer, CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY);
ESP_LOGI(TAG, "%s: setting socket_flush_timer for %d ms", __func__, CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY);
                             }
                         }
                         err = written;
                     } else {
                         /* Must be disconnecting - give error */
                         err = LSE_DISCONNECTING;
                     }
                     break;
                 }
     
                 default: {
                     err = LSE_NOT_WRITABLE;
                     break;
                }
            }
     
            socket->busy = false;
        }

        UNLOCK_SOCKET(socket);
    }

    return err;
}

ssize_t ls_write(int s, const void* buf, size_t len)
{
    return (ssize_t) ls_write_helper(validate_socket(s), buf, len, false);
}

/*
 * Same as ls_write, but delivers an 'end of record' mark at end of data.
 * End of record write does nothing special for datagram sockets.
 */
ssize_t ls_write_eor(int s, const void* buf, size_t len)
{
    return (ssize_t) ls_write_helper(validate_socket(s), buf, len, true);
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
ssize_t ls_read_with_address(int s, char* buf, size_t maxlen, int* address, int* port, int timeout)
{
    ls_error_t err = LSE_NO_ERROR;

    ls_socket_t *socket = validate_socket(s);

    if (socket == NULL) {
ESP_LOGI(TAG, "%s: bad socket", __func__);
        err = LSE_INVALID_SOCKET;
    } else {

        if (socket->busy) {
            err = LSE_SOCKET_BUSY;
        } else {
            socket->busy = true;

            if (socket->socket_type == LS_DATAGRAM) {

                /* Pend on a packet in the queue */
                packet_t *packet;

                UNLOCK_SOCKET(socket);

                bool success = os_get_queue_with_timeout(socket->datagram_packets, (os_queue_item_t*) &packet, timeout);

                LOCK_SOCKET(socket);

                if (success) {
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

                    /* Switch destination origin of packet so a write goes to correct location */
                    if (socket->rename_dest) {
                        socket->dest_addr = origin_address;
                        socket->dest_port = origin_port;
                    }

                    err = packet_data_length;
                    release_packet(packet);

                } else {
                    err = LSE_TIMEOUT;
                }

            } else if (socket->socket_type == LS_STREAM) {
ESP_LOGI(TAG, "%s: Reading stream... maxlen %d", __func__, maxlen);
                /* STREAM packets arrive on the socket stream_packet_queue after being sorted and acked as necessary */
                packet_t *packet;
                bool eor = false;
                int total_len = 0;

                /* Go until length satisfied or end of record */
                while (!eor && maxlen != 0) {
                    int offset = 0;

                    if (socket->current_read_packet != NULL) {
                        packet = socket->current_read_packet;
                        socket->current_read_packet = NULL;
                        offset = socket->current_read_offset;
                        socket->current_read_offset = 0;
                    } else {
ESP_LOGI(TAG, "%s: waiting for packet", __func__);
                        /* This blocks until packet is ready or socket is shut down remotely */
                        UNLOCK_SOCKET(socket);
                        packet = read_packet_from_window(socket->input_window);
                        LOCK_SOCKET(socket);
ESP_LOGI(TAG, "%s: read_packet_from_window returned %p", __func__, packet);
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
                            eor = (get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN) & STREAM_FLAGS_CMD) == STREAM_FLAGS_CMD_DATA_EOR;
                            /* Release packet */
                            release_packet(packet);
                        } else {
                            /* Otherwise save the packet and offset */
                            socket->current_read_packet = packet;
                            socket->current_read_offset = offset;
                        }
                    } else {
ESP_LOGI(TAG, "%s: received NULL packet - eor", __func__);
                        /* Socket remotely closed.  Return residue for last read. */
                        eor = true;
                        /* Shutdown socket */
                        //ls_close_immediate(s);
                    }
                }
                /* Return amount of data read */
                err = total_len;
            } else {
                err = LSE_INVALID_SOCKET;
            }
        
            socket->busy = false;
        }

        UNLOCK_SOCKET(socket);
    }

    if (err != LSE_INVALID_SOCKET) {
        socket->last_error = err;
    }

    return err;
}

ssize_t ls_read(int s, void* buf, size_t maxlen)
{
    return (int) ls_read_with_address(s, (char*) buf, maxlen, NULL, NULL, -1);
}

ssize_t ls_read_with_timeout(int s, void* buf, size_t maxlen, int timeout)
{
    return (int) ls_read_with_address(s, (char*) buf, maxlen, NULL, NULL, timeout);
}


/*
 * Close a socket.
 */
static ls_error_t ls_close_ptr(ls_socket_t *socket)
{
    ls_error_t err = LSE_NO_ERROR;

    if (socket != NULL) {
        if (socket->busy) {
            err = LSE_SOCKET_BUSY;
        } else {
            socket->busy = true;

            switch (socket->socket_type) {
                default: {
                    err = LSE_INVALID_SOCKET;
                    break;
                }

                case LS_DATAGRAM: {
                    err = release_socket(socket);
                    socket->inuse = false;
                    break;
                }

                case LS_STREAM: {
                    if (socket->listen) {
                        err = release_socket(socket);
                    } else {
                        /* Let others know this is the end... */
                        socket->output_window->closing = true;

//ESP_LOGI(TAG, "%s: start_state_machine ls_close DISCONNECTING_FLUSH stream_flush_output_window on socket %d", __func__, socket - socket_table);
                        start_state_machine(socket,
                                            LS_STATE_DISCONNECTING_FLUSH,
                                            stream_flush_output_window,
                                            NULL,
                                            STREAM_FLUSH_TIMEOUT,
                                            STREAM_FLUSH_RETRIES);

//ESP_LOGI(TAG, "%s: release socket lock...", __func__);
                        UNLOCK_SOCKET(socket);
//ESP_LOGI(TAG, "%s: read state machine response...", __func__);
                        err = get_state_machine_results(socket);
//ESP_LOGI(TAG, "%s: response was %d...", __func__, err);
                        LOCK_SOCKET(socket);
//ESP_LOGI(TAG, "%s: relock socket lock...", __func__);

//ESP_LOGI(TAG, "%s: start_state_machine ls_close DISCONNECTING stream_send_disconnect on socket %d", __func__, socket - socket_table);
                        start_state_machine(socket,
                                            LS_STATE_OUTBOUND_DISCONNECTING,
                                            stream_send_disconnect,
                                            NULL,
                                            STREAM_DISCONNECT_TIMEOUT,
                                            STREAM_DISCONNECT_RETRIES);
              
                        UNLOCK_SOCKET(socket);
                        err = get_state_machine_results(socket);
                        LOCK_SOCKET(socket);

                        /* Start a linger to leave socket intact for a short while */
                        start_state_machine(socket,
                                            LS_STATE_LINGER,
                                            socket_linger_check,
                                            socket_linger_finish,
                                            CONFIG_LASTLINK_SOCKET_LINGER_TIME,
                                            5);

                        socket->inuse = false;
                    }
                    break;
                }
            }

            socket->busy = false;
        }

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
static ls_socket_t *validate_socket(int s)
{
    ls_socket_t *ret = NULL;

    if (s >= FIRST_LASTLINK_FD && s <= LAST_LASTLINK_FD) {
 
        s -= FIRST_LASTLINK_FD;
    
        if (s >= 0 && s < ELEMENTS_OF(socket_table)) {

            os_acquire_recursive_mutex(socket_table[s].lock);

            if (socket_table[s].socket_type == LS_DATAGRAM || socket_table[s].socket_type == LS_STREAM) {
                ret = &socket_table[s];
            } else {
                os_release_recursive_mutex(socket_table[s].lock);
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
 * Put a packet into the queue at the expected sequence number entry.
 * Ignores action if outside of window.
 *
 * Entry:
 *      window              Window holding packets
 *      packet              Packet to add to window
 *
 * Returns true if packet accepted with success.
 *
 */
static bool receive_packet_to_window(packet_window_t *window, packet_t *packet)
{
    bool ok = false;

    if (window != NULL) {
//ESP_LOGI(TAG, "%s: waiting for lock", __func__);
        os_acquire_recursive_mutex(window->lock);
        int ready_count = 0;

        if (packet != NULL) {
            /* Compute window slot to receive the packet */
            int slot = get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN) - window->sequence;

//ESP_LOGI(TAG, "%s: slot %d from packet %d window %d", __func__, slot, get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN), window->sequence);

            /* Validate for fitting into table */
            if (slot >= 0 && slot < window->length) {
                /* Ignore packet if already installed */
                if (window->slots[slot] == NULL) {
                    window->slots[slot] = ref_packet(packet);
                    /* Remember high-water mark of packets received */
                    if (slot >= window->next_in) {
                         window->next_in = slot + 1;
                    }

ESP_LOGI(TAG, "%s: packet stored in slot %d", __func__, slot);

                    ok = true;

                    /* Find first hole.  If it is past our current entry, generate releases to indicate
                     * the new packets available in the window.
                     */
                    int hole;

                    for (hole = 0; hole < window->next_in && window->slots[hole] != NULL; ++hole) {
                        /* Finds hole or end */
                    }

                    if (hole > slot) {
                        ready_count += hole - slot;
                    }
                } 
            }
        } else {
ESP_LOGI(TAG, "%s: packet was NULL", __func__);

            /* A NULL packet is a special case to force clearing the packet window.  It generates a single release
             * and while be perceived as a close request when the packet is removed from the window.
             */
            /* One extra release so the end of data is seen */
            ready_count = 1;
        }

        /* Release as many slots as have become contiguous */
        if (ready_count != 0) {
//ESP_LOGI(TAG, "%s: %d packets ready", __func__, ready_count);
            bool rok = os_release_counting_semaphore(window->available, ready_count);
ESP_LOGI(TAG, "%s: release count %d returned %s", __func__, ready_count, rok ? "TRUE" : "FALSE");
        }

        os_release_recursive_mutex(window->lock);
    }

    return ok;
}

/*
 * Get a stream packet from window for delivery to user.
 *
 * When NULL is returned, the socket was closed remotely.
 */
static packet_t *read_packet_from_window(packet_window_t *window)
{
    packet_t *packet = NULL;

    if (! window->closing) {
        os_acquire_counting_semaphore(window->available);

ESP_LOGI(TAG, "%s: slot[0] is %p next_in is %d", __func__, window->slots[0], window->next_in);

        /* Remove first entry */
        os_acquire_recursive_mutex(window->lock);

        /* Take the top of the window */
        packet = window->slots[0];

        /* If nothing in window, we are closing */
        if (packet != NULL) {
            memcpy(window->slots + 0, window->slots + 1, sizeof(window->slots[0]) * window->next_in);
            window->next_in--;
            /* Indicate next sequence expected */
            window->sequence++;
ESP_LOGI(TAG, "%s: sequence is now %d", __func__, window->sequence);
        } else {
            window->closing = true;
        }

        os_release_recursive_mutex(window->lock);
    }

    return packet;
}

            

/*
 * Write a new sequential packet to the window.  Labels the packet with the correct sequence number
 * and places it into the appropriate slot.  Requires acquiring counting semaphore to approve
 * allocation of the slot.
 *
 * This is the user side of the write-packet-to-network functionality;
 *
 * Entry:
 *        window            Window to receive packets.
 *        packet            Packet to place in window.  Will be ref'd if accepted.
 *        wait              if true, waits for slot in packet otherwise bails if full.
 *
 * Returns true on success.
 */
static bool write_packet_to_window(packet_window_t *window, packet_t *packet, bool wait)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: waiting for available...", __func__);
    if (os_acquire_counting_semaphore_with_timeout(window->available, wait ? -1 : 0)) {

         /* Room to put the packet.  Lock the window */
ESP_LOGI(TAG, "%s: waiting for lock...", __func__);
         os_acquire_recursive_mutex(window->lock);

ESP_LOGI(TAG, "%s: got lock...", __func__);

         /* Sanity check */
         if (window->next_in < window->length) {
ESP_LOGI(TAG, "%s: putting packet %p in window at %d", __func__, packet, window->next_in);
             /* Insert sequence number into packet */
             set_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, window->sequence + window->next_in);
             window->slots[window->next_in++] = ref_packet(packet);
             ok = true;
         }

         os_release_recursive_mutex(window->lock);
    }

    return ok;
}


/* Return the sequence and ack_residue window */
static bool get_ack_window(packet_window_t *window, int *sequence, unsigned int *ack_residue)
{
    os_acquire_recursive_mutex(window->lock);

    *ack_residue = 0;

    int slot;

    /* Return sequence number of last packet in contiguous list + 1 */
    for (slot = 0; window->slots[slot] != NULL && slot < window->length; ++slot) {
        /* Look for last slot + 1 */

        /* And accumulate residue if past slot 2 */
        if ((slot >= 2) && window->slots[slot] != NULL) {
            *ack_residue |= 1 << (slot - 2);
        }
    }

    *sequence = (slot != 0)
              ? get_uint_field(window->slots[slot - 1], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN)  /* Sequence of last contiguous packet in list */
              : window->sequence ;                                                             /* Default is next expected in window */
 
    os_release_recursive_mutex(window->lock);

    return true;
}
            
/*
 * Called when a ACK sequence and ACK residue mask are received.
 *
 * The sequence number is the NEXT expected sequence to be received by the caller.
 * The window gives hints as to any out-of-sequence packets that have been received.
 * 
 * This function releases the packets, rolls up the window and release the appropriate
 * count to the semaphore describing the room available.
 *
 * Entry:
 *      window          - window containing the packets being processed.
 *      sequence        - sequence number being acked (i.e. next sequence number expected)
 *      residue_mask    - additional packets (bit field relative to sequence + 2) that are acked.
 *
 * Return true if more packets remain in window.
 *
 */
static bool release_packets_in_window(packet_window_t *window, int sequence, unsigned int residue_mask)
{
    os_acquire_recursive_mutex(window->lock);
ESP_LOGI(TAG, "%s: sequence %d residue_mask %04x", __func__, sequence, residue_mask);

    /* Ack the head of the queue, if any */
    while (window->next_in != 0 && (window->slots[0] == NULL ||  (sequence > get_uint_field(window->slots[0], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN)))) {
ESP_LOGI(TAG, "%s: release packet %p in window slot 0", __func__, window->slots[0]);

        if (window->slots[0] != NULL) {
            release_packet(window->slots[0]);

            /* Immediate release when window is squished */
            os_release_counting_semaphore(window->available, 1);
        }

        /* Remove from table */
        memcpy(window->slots + 0, window->slots + 1, sizeof(window->slots[0]) * window->next_in);

        /* Next expected sequence */
        window->sequence++;

        /* Move back one slot for next input position */
        window->next_in--;
    }

    /* Release the other packets defined by the window */
    int residue = 1;
    while (residue_mask != 0 && residue < window->next_in) {
        if (((residue_mask & 1) != 0) && (window->slots[residue] != NULL)) {
ESP_LOGI(TAG, "%s: release packet %p in window slot %d", __func__, window->slots[residue], residue);
            release_packet(window->slots[residue]);
            window->slots[residue] = NULL;
            window->released++;
        }
        residue_mask >>= 1;
        residue++;
    }

    /* When queue goes empty, release all pending releases */
    if (window->next_in == 0 && window->released != 0) {
        /* Window went from non-empty to empty so give space back to the caller */
ESP_LOGI(TAG, "%s: released %d in queue", __func__, window->released);
        os_release_counting_semaphore(window->available, window->released);
        window->released = 0;
    }

    os_release_recursive_mutex(window->lock);

    /* Return true if packets remain in window */
    return window->next_in != 0;
}


/*
 * Allocate and initialize the packet window.
 *
 * Entry:
 *        length           Total window size of queue
 *        available        Initial available slots (0 for read side; length for write side)
 */
static packet_window_t *allocate_packet_window(int length, int available)
{
     packet_window_t *window = (packet_window_t*) malloc(sizeof(packet_window_t) + sizeof(packet_t*) * (length - 1));

     if (window != NULL) {
         memset(window, 0, sizeof(packet_window_t) + sizeof(packet_t*) * (length - 1));
         window->available = os_create_counting_semaphore(length, available);
assert(window->available != NULL);
         window->lock = os_create_recursive_mutex();
         window->length = length;
     }

     return window;
}

static bool release_packet_window(packet_window_t *window)
{
    os_acquire_recursive_mutex(window->lock);

    os_delete_semaphore(window->available);

    /* Release packets in queue */
    for (int index = 0; index < window->length; ++index) {
        if (window->slots[index] != NULL) {
            release_packet(window->slots[index]);
            window->slots[index] = NULL;
        }
    }

    os_delete_mutex(window->lock);

    free((void*) window);

    return true;
}

static ls_error_t ls_dump_socket_ptr(const char* msg, const ls_socket_t *socket)
{
    if (socket != NULL) {
        printf("%s: %d (%p) state %s listen %s type %s local port %d dest port %d dest addr %d parent %d\n",
               msg, socket - socket_table + FIRST_LASTLINK_FD,
               socket,
               socket_state_of(socket),
               socket->listen ? "YES" : "NO",
               socket_type_of(socket),
               socket->local_port,
               socket->dest_port,
               socket->dest_addr,
               //socket->serial_number,
               socket->parent ? (socket->parent - socket_table) : -1);
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
    
/* Called by timer to work on the sockets.
 * Rechedules itself when new time interval is needed.
 */
static void socket_scanner_thread(void* param)
{
    static int count;

    while (true) {
        count = (count + 1) % 1000;

        uint32_t next_time = SOCKET_SCANNER_DEFAULT_INTERVAL;

        for (int socket_index = 0; socket_index < ELEMENTS_OF(socket_table); ++socket_index) {
            ls_socket_t *socket = &socket_table[socket_index];

            /* Attempt to acquire the socket - if not, move to the next one and get it next time */
            if (LOCK_SOCKET_TRY(socket)) {

                if (count == 0) {
                    ESP_LOGI(TAG, "%s: socket %d  state_machine_timer %d  socket_flush_timer %d  output_window_timer %d",
                             __func__,
                             socket - socket_table,
                             simpletimer_remaining(&socket->output_window_timer),
                             simpletimer_remaining(&socket->socket_flush_timer),
                             simpletimer_remaining(&socket->output_window_timer));
                }

//ESP_LOGI(TAG, "%s: scanning socket %d", __func__, socket_index);

                /* Check the socket timers and fire action if ready */
                if (simpletimer_is_expired_or_remaining(&socket->state_machine_timer, &next_time)) {
                    simpletimer_stop(&socket->state_machine_timer);
ESP_LOGI(TAG, "%s: **************************** state_machine_timer for socket %d fired", __func__, socket_index);
                    state_machine_timeout(socket);
ESP_LOGI(TAG, "%s: **************************** state_machine_timer finished", __func__);
//                } else if (simpletimer_is_running(&socket->state_machine_timer)) {
//ESP_LOGI(TAG, "%s: **************************** state_machine_timer remaining %d", __func__, simpletimer_remaining(&socket->state_machine_timer));
                }

                if (simpletimer_is_expired_or_remaining(&socket->socket_flush_timer, &next_time)) {
                    simpletimer_stop(&socket->socket_flush_timer);
ESP_LOGI(TAG, "%s: **************************** socket_flush_timer for socket %d fired", __func__, socket_index);
                    socket_flush_current_packet(socket, /* wait */ false);
ESP_LOGI(TAG, "%s: **************************** socket_flush_timer finished", __func__);
                }

                if (simpletimer_is_expired_or_remaining(&socket->output_window_timer, &next_time)) {
                    simpletimer_stop(&socket->output_window_timer);
ESP_LOGI(TAG, "%s: **************************** output_window_timer for socket %d fired", __func__, socket_index);
                    send_output_window(socket, /* singleton */ false);
ESP_LOGI(TAG, "%s: **************************** output_window_timer finished", __func__);
                }

                UNLOCK_SOCKET(socket);

            } else {
                ESP_LOGI(TAG, "%s: skipping socket %d", __func__, socket_index);
            }
        }

//ESP_LOGI(TAG, "%s: delay %d", __func__, next_time);
        os_delay(next_time);
    }
}

/*
 * User interface commands.
 */
#if CONFIG_LASTLINK_TABLE_COMMANDS
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
                pings[num_pings].thread = ping_table[num_pings].thread != NULL;
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
/* stlisten <port>                                                    */
/**********************************************************************/
static int stlisten_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<port>", "Listen for stream connection on <port>");
    } else if (argc > 1) {
        int port = strtol(argv[1], NULL, 10);
ESP_LOGI(TAG, "port %d", port);
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
                        printf("ls_listen returned %d\n", ret);
                        done = true;
                    } else {
                        done = hit_test('\x03');
                    } 
                } while (!done);
                ls_close(socket);
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
/* stconnect <address> <port>                                         */
/**********************************************************************/
static int stconnect_command(int argc, const char **argv)
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
                ls_dump_socket("sending", socket);
                if (ret >= 0) {
#if 1
                    for (int line = 1; line <= 1000; ++line) {
                        char buffer[80];
                        sprintf(buffer, "%s: line %d\n", message, line);
                        //ret = ls_write(socket, buffer, strlen(buffer));
                        //printf("ls_write returned %d\n", ret);
                        ret = write(socket, buffer, strlen(buffer));
                        printf("write returned %d\n", ret);
                    }
#else
                    static const char *buf = "this is a test\n";
                    ret = write(socket, buf, strlen(buf));
#endif
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

            os_delay(1000);

            count--;
        }
        printf("%d fail  %d good  %d total\n", fail, good, total);
    }

    return 0;
}

#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */

/*
 * Initialize ls_socket layer.
 */
ls_error_t ls_socket_init(void)
{
    ls_error_t err = 0;

    /* Add a mutex to all socket table entries */
    for (int socket = 0; socket < ELEMENTS_OF(socket_table); ++socket) {
        socket_table[socket].lock = os_create_recursive_mutex();
    }

    /* Register stream and datagram protocols */
    if (linklayer_register_protocol(PING_PROTOCOL,           ping_packet_process,          ping_packet_format)        &&
        linklayer_register_protocol(PINGREPLY_PROTOCOL,      pingreply_packet_process,     pingreply_packet_format)   &&
        linklayer_register_protocol(STREAM_PROTOCOL,         stream_packet_process,        stream_packet_format)      &&
        linklayer_register_protocol(DATAGRAM_PROTOCOL,       datagram_packet_process,      datagram_packet_format)) {

        global_socket_lock = os_create_recursive_mutex();

        /* Create the socket_scanner thread */
        socket_scanner_thread_id = os_create_thread(socket_scanner_thread, "socket_scanner", SOCKET_SCANNER_STACK_SIZE, 0, NULL);

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

#if CONFIG_LASTLINK_TABLE_COMMANDS
    add_command("dglisten",   dglisten_command);
    add_command("dgsend",     dgsend_command);
    add_command("stlisten",   stlisten_command);
    add_command("stconnect",  stconnect_command);
    add_command("stconcycle", stconcycle_command);
    add_command("ping",       ping_command);
    add_command("pt",         print_ping_table);
#endif

    return err;
}
ls_error_t ls_socket_deinit(void)
{
#if CONFIG_LASTLINK_TABLE_COMMANDS
    remove_command("dglisten");
    remove_command("dgsend");
    remove_command("stlisten");
    remove_command("stconnect");
    remove_command("ping");
    remove_command("pt");
#endif

    /* Kill socket scanner */
    os_delete_thread(socket_scanner_thread_id);
    socket_scanner_thread_id = NULL;

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
        linklayer_unregister_protocol(STREAM_PROTOCOL)      &&
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

    return err;
}

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_LAYER */
