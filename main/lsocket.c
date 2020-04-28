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

#include "os_freertos.h"
#include "lsocket_internal.h"
#include "packets.h"
#include "linklayer.h"

#define TAG "lsocket"

/* PING support */
static const char* ping_packet_format(const packet_t *p);
static const char* pingreply_packet_format(const packet_t *p);
static bool ping_packet_process(packet_t *p);
static bool pingreply_packet_process(packet_t *p);
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

static packet_t *datagram_packet_create_from_packet(const packet_t *packet);
static packet_t *datagram_packet_create_from_socket(const ls_socket_t *socket);

static packet_t *stream_packet_create_from_packet(const packet_t *packet, uint8_t flags);
static packet_t *stream_packet_create_from_socket(const ls_socket_t *socket, uint8_t flags);

static ls_socket_t *find_socket_from_packet(const packet_t *packet, ls_socket_type_t type);
static ls_socket_t *find_listening_socket_from_packet(const packet_t *packet);

static ssize_t release_socket(ls_socket_t* socket);

static os_timer_t socket_timer;
static void socket_timer_scanner(os_timer_t);
static void start_socket_timer(socket_timer_t *timer, int delay);
static void stop_socket_timer(socket_timer_t *timer);
static bool socket_timer_check(socket_timer_t *timer, uint64_t current_time, uint32_t *next_time);

static void start_state_machine(ls_socket_t *socket, ls_socket_state_t state, packet_t *packet, int timeout, int retries);
static void cancel_state_machine(ls_socket_t *socket);
static void state_machine_timeout(ls_socket_t *socket);
static void send_state_machine_response(ls_socket_t *socket, ls_error_t response);
static ls_error_t get_state_machine_response(ls_socket_t *socket);
static void socket_flush_stream(ls_socket_t *socket);
static void ack_output_window(ls_socket_t *socket, int ack_sequence, unsigned int ack_window);
static void send_output_window(ls_socket_t *socket);
static void socket_linger_check(ls_socket_t *socket);

static ls_error_t ls_close_ptr(ls_socket_t *socket);

/* Put a packet into the relative window of queue.  Returns true if success */
static bool put_packet_into_window(packet_window_t *queue, packet_t *packet);
static packet_t *get_packet_from_window(packet_window_t *queue);
static bool write_packet_to_window(packet_window_t *queue, packet_t *packet);
static void release_packets_in_window(packet_window_t *queue, int sequence, unsigned int window);
static packet_window_t *allocate_packet_window(int length, int available);
static bool release_packet_window(packet_window_t *queue);

typedef struct ping_table_entry {
    os_queue_t   queue;
    int          sequence;
} ping_table_entry_t;

ping_table_entry_t   ping_table[CONFIG_LASTLINK_MAX_OUTSTANDING_PINGS];

static ls_socket_t  socket_table[CONFIG_LASTLINK_NUMBER_OF_SOCKETS];

/*
 * Ping packet create
 */
packet_t *ping_packet_create(int dest)
{
    packet_t *p;

    p = linklayer_create_generic_packet(dest, PING_PROTOCOL, PING_LEN);

    if (p != NULL) {
        /* Generate sequence number */
        set_uint_field(p, PING_SEQUENCE, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());

        /* Put starting address in the table */
        set_uint_field(p, PING_ROUTE_TABLE, ADDRESS_LEN, linklayer_node_address);
    }

    return p;
}

static bool ping_packet_process(packet_t *p)
{
    bool processed = false;

    if (p != NULL) {
//linklayer_print_packet("PING RECEIVED", p);

        if (linklayer_lock()) {
            /* Sanity check - reuse last entry if full */
            if (p->length >= MAX_PACKET_LEN) {
                p->length = MAX_PACKET_LEN - ADDRESS_LEN;
            }

            /* Add our address to the chain of addresses */
            p->length += ADDRESS_LEN;
            set_uint_field(p, p->length - ADDRESS_LEN, ADDRESS_LEN, linklayer_node_address);

            /* If for us, turn packet into PING_REPLY */
            if (linklayer_packet_is_for_this_node(p)) {

                /* Alter the protocol */
                set_uint_field(p, HEADER_PROTOCOL, PROTOCOL_LEN, PINGREPLY_PROTOCOL);

                /* Turn the packet around */
                set_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN, get_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN));
                set_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
                set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);   /* Force packet to be routed */

                /* Rewind TTL */
                set_uint_field(p, HEADER_TTL, TTL_LEN, TTL_DEFAULT);

//linklayer_print_packet("SEND PING REPLY", p);

                linklayer_send_packet(ref_packet(p));

                processed = true;
            }

            linklayer_unlock();

        } else {
            /* Cannot get mutex */
        }
    }

    return processed;
}

static const char* ping_format_routes(const packet_t *p)
{
    int num_routes = (p->length - PING_ROUTE_TABLE) / ADDRESS_LEN;

    /* Allocate worst case length */
    char *routes = (char*) malloc(2 + (8 + 1) * num_routes + 2);
    char *routep = routes;

    *routep++ = '[';
    *routep++ = ' ';

    for (int route = 0; route < num_routes; ++route) {
        int len = sprintf(routep, "%X ", get_uint_field(p, PING_ROUTE_TABLE + ADDRESS_LEN * route, ADDRESS_LEN));
        routep += len;
    }
    *routep++ = ']';
    *routep++ = '\0';

    return routes;
}

static const char* ping_packet_format(const packet_t *p)
{
    char* info;

    int sequence = get_uint_field(p, PING_SEQUENCE, SEQUENCE_NUMBER_LEN);
    const char* routes = ping_format_routes(p);

    asprintf(&info, "Ping: Seq: %d Routes %s", sequence, routes);

    free((void*) routes);

    return info;
}

static const char* pingreply_packet_format(const packet_t *p)
{
    char* info;

    int sequence = get_uint_field(p, PING_SEQUENCE, SEQUENCE_NUMBER_LEN);
    const char* routes = ping_format_routes(p);

    asprintf(&info, "Ping Reply: Seq: %d Routes %s", sequence, routes);

    free((void*) routes);

    return info;
}

static bool pingreply_packet_process(packet_t *p)
{
    bool processed = false;

    if (p != NULL) {
        if (linklayer_lock()) {

//linklayer_print_packet("PING REPLY RECEIVED", p);

            /* If reply is for us, we do not add our address - but deliver it */
            if (linklayer_packet_is_for_this_node(p)) {
                /* Look for ping request and deliver packet to process queue */
                int slot = find_ping_table_entry(get_uint_field(p, PING_SEQUENCE, SEQUENCE_NUMBER_LEN));
                if (slot >= 0) {
                    os_put_queue(ping_table[slot].queue, ref_packet(p));
                }

            } else {
                /* Sanity check - reuse last entry if full */
                if (p->length >= MAX_PACKET_LEN) {
                    p->length = MAX_PACKET_LEN - ADDRESS_LEN;
                }

                p->length += ADDRESS_LEN;
                set_uint_field(p, p->length - ADDRESS_LEN, ADDRESS_LEN, linklayer_node_address);

                linklayer_send_packet_update_ttl(ref_packet(p));
            }

            processed = true;

            linklayer_unlock();

        } else {
            /* Cannot get mutex */
        }
    }

    return processed;
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

static int register_ping(const packet_t *packet)
{
    int slot = find_ping_table_entry(0);
    if (slot >= 0) {
        /* Remember we are waiting on this sequence return */
        ping_table[slot].queue = os_create_queue(1, sizeof(packet_t*));
        ping_table[slot].sequence = get_uint_field(packet, PING_SEQUENCE, SEQUENCE_NUMBER_LEN);
    }

    return slot;
}

static packet_t *wait_for_ping_reply(int slot, int timeout)
{
    packet_t *packet = NULL;

    if (slot >= 0 && slot < ELEMENTS_OF(ping_table)) {
        if (! os_get_queue_with_timeout(ping_table[slot].queue, (os_queue_item_t*) &packet, timeout)) {
            /* No reply - make sure empty return */
            packet = NULL;
        }
        ping_table[slot].sequence = 0;
        os_delete_queue(ping_table[slot].queue);
    }

    return packet;
}

/*
 * Ping an address and return it's pathlist
 */
ls_error_t ping(int address, int *pathlist, int pathlistlen, int timeout)
{
    ls_error_t err = 0;

    packet_t *packet = ping_packet_create(address);

    if (packet != NULL) {
        int slot = register_ping(packet);

        if (slot >= 0) {

            linklayer_send_packet(packet);

            packet_t *reply = wait_for_ping_reply(slot, timeout);

            if (reply != NULL) {
                /* Pass information back to caller */
                int num_routes = (reply->length - PING_ROUTE_TABLE) / ADDRESS_LEN;

                for (int route = 0; pathlistlen > 0 && route < num_routes; ++route) {
                    pathlist[route] = get_uint_field(reply, PING_ROUTE_TABLE + ADDRESS_LEN * route, ADDRESS_LEN);
                    pathlistlen--;
                }

                err = num_routes;

                release_packet(reply);
            } else {
                err = LSE_TIMEOUT;
            }

        } else {
            err = LSE_NO_MEM;
        }

    } else {
        err = LSE_NO_MEM;
    }

    return err;
}


static ls_socket_t *find_free_socket(void)
{
    ls_socket_t *socket = NULL;

    for (int index = 0; socket == NULL && index < ELEMENTS_OF(socket_table); ++index) {
        if (socket_table[index].socket_type == LS_UNUSED) {
            socket = &socket_table[index];
        }
    }

    return socket;
}

static int next_local_port = 5000;

static int find_free_local_port(void) {

    int found = -1;

    if (linklayer_lock()) {
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

        linklayer_unlock();
    }

    return found;
}


/* Look in socket table for socket matching the connection and type */
static ls_socket_t *find_socket_from_packet(const packet_t *packet, ls_socket_type_t type)
{
    ls_socket_t *socket = NULL;

    ls_port_t dest_port = get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN);
    ls_port_t src_port  = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN);
    int dest_addr       = get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN);

printf("find_socket_from_packet: addr %d src port %d dest port %d\n", dest_addr, src_port, dest_port);

    for (int index = 0; socket == NULL && index < CONFIG_LASTLINK_NUMBER_OF_SOCKETS; ++index) {
ls_dump_socket_ptr("testing", socket_table + index);
        if (socket_table[index].socket_type == type &&
            !socket_table[index].listen &&
            (socket_table[index].dest_addr == 0 || socket_table[index].dest_addr == dest_addr) &&
            socket_table[index].local_port == dest_port &&
            (socket_table[index].dest_port == 0 || socket_table[index].dest_port == src_port)) {

ls_dump_socket_ptr("found", socket_table + index);
            socket = &socket_table[index];
        }
    }

    /* Return socket if we found it */
    return socket;
}

static ls_socket_t *find_listening_socket_from_packet(const packet_t *packet)
{
    ls_socket_t *socket = NULL;

    ls_port_t dest_port = get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN);

    for (int index = 0; socket == NULL && index < ELEMENTS_OF(socket_table); ++index) {
        /* A socket listening on the specified port is all we need */
        if (socket_table[index].listen && socket_table[index].local_port == dest_port) {
            socket = &socket_table[index];
        }
    }

    /* Return socket if we found it */
    return socket;
}

static packet_t *datagram_packet_create_from_packet(const packet_t *packet)
{
    packet_t *new_packet = linklayer_create_generic_packet(get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                                           DATAGRAM_PROTOCOL, DATAGRAM_PAYLOAD);

    if (new_packet != NULL) {
        set_uint_field(new_packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN, get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN));
        set_uint_field(new_packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN, get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN));
    }

    return new_packet;
}

static packet_t *datagram_packet_create_from_socket(const ls_socket_t *socket)
{
    packet_t *packet = linklayer_create_generic_packet(socket->dest_addr, DATAGRAM_PROTOCOL, DATAGRAM_PAYLOAD);

    if (packet != NULL) {
        set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN, socket->dest_port);
        set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN, socket->local_port);
    }

    return packet;
}

static bool datagram_packet_process(packet_t *packet)
{
    bool processed = false;

    if (packet != NULL && linklayer_packet_is_for_this_node(packet)) {
//ESP_LOGD(TAG, "%s: finding socket for this packet", __func__);
        ls_socket_t *socket = find_socket_from_packet(packet, LS_DATAGRAM);

//ls_dump_socket_ptr("inbound packet for", socket);

        if (socket != NULL) {

            /* Send the packet to the datagram input queue */
            if (!os_put_queue(socket->datagram_packets, (os_queue_item_t) ref_packet(packet))) {
                release_packet(packet);
            }
 
            processed = true;
        }
    }

    return processed;
}

static const char* datagram_packet_format(const packet_t *packet)
{
    char* info;

    const char* data = linklayer_escape_raw_data(packet->buffer + DATAGRAM_PAYLOAD, packet->length - DATAGRAM_PAYLOAD);

    asprintf(&info, "Datagram: Src Port %d Dest Port %d \"%s\"",
            get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN),
            get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN),
            data);

     free((void*) data);

     return info;
}

static packet_t *stream_packet_create_from_packet(const packet_t *packet, uint8_t flags)
{
    packet_t *new_packet = linklayer_create_generic_packet(get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                                           STREAM_PROTOCOL, STREAM_PAYLOAD);

    if (new_packet != NULL) {
        set_uint_field(new_packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN, get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN));
        set_uint_field(new_packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN, get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN));
        set_uint_field(new_packet, STREAM_FLAGS, FLAGS_LEN, flags);
    }

    return new_packet;
}

static packet_t *stream_packet_create_from_socket(const ls_socket_t *socket, uint8_t flags)
{
    packet_t *packet = linklayer_create_generic_packet(socket->dest_addr, STREAM_PROTOCOL, STREAM_PAYLOAD);

    if (packet != NULL) {
        set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN, socket->dest_port);
        set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN, socket->local_port);
        /* The connection sequence number rides on the STREAM sequence number field until data flows */
        set_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, socket->serial_number);
        set_uint_field(packet, STREAM_FLAGS, FLAGS_LEN, flags);
    }

    return packet;
}

#if 0
static packet_t *assemble_socket_data(packet_t *packet_to_send, packet_t *packet)
{
    return packet_to_send;
}
#endif

static bool stream_packet_process(packet_t *packet)
{
    bool processed = false;

    bool reject = true;

linklayer_print_packet("stream packet", packet);

    if (packet != NULL && linklayer_packet_is_for_this_node(packet)) {
        /* Read the flags field for later use */
        uint8_t flags = get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN);

        /* Get the socket for this packet if there is one */
        ls_socket_t *socket = find_socket_from_packet(packet, LS_STREAM);

        switch (flags & STREAM_FLAGS_CMD) {
            default: {
                linklayer_print_packet("BAD STREAM", packet);
                /* Ignore noise */
                break;
            }

            case STREAM_FLAGS_CMD_DATA: {
                /*
                 * Pass the data through the assembly phase.  This might require modifying the current outbound packet
                 * to update ack seqeuence numbers, etc.  In some cases a brand new packet will be generated.
                 *
                 * If there is no data in this packet, it will be ignored if the first packet in the queue.
                 */
                if (put_packet_into_window(socket->input_window, packet)) {
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
                        linklayer_send_packet(stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_CONNECT_ACK));
                        reject = false;
                    }
                } else {
                    /* Didn't find actual socket, so see if this is a new listen */
                    ls_socket_t *listen_socket = find_listening_socket_from_packet(packet);

                    if (listen_socket != NULL) {
                        ls_socket_t *new_connection = find_free_socket();

                        if (new_connection != NULL) {
                            new_connection->inuse = true;
                            new_connection->dead = false;

                            /* Create a new connection that achievs a local endpoint for the new socket */
                            new_connection->socket_type = LS_STREAM;
                            new_connection->serial_number = get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN);
                            new_connection->state = LS_STATE_INBOUND_CONNECT;
                            new_connection->local_port = listen_socket->local_port;
                            new_connection->dest_port = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN);
                            new_connection->dest_addr = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
                            new_connection->parent = listen_socket;
                            new_connection->input_window = allocate_packet_window(CONFIG_LASTLINK_STREAM_WINDOW_SIZE, 0);
                            new_connection->output_window = allocate_packet_window(CONFIG_LASTLINK_STREAM_WINDOW_SIZE, CONFIG_LASTLINK_STREAM_WINDOW_SIZE);

ls_dump_socket_ptr("new connection", new_connection);

                            /* Send a CONNECT ACK and go to INBOUND CONNECT state waiting for full acks */
                            start_state_machine(new_connection, LS_STATE_INBOUND_CONNECT,
                                                stream_packet_create_from_socket(new_connection, STREAM_FLAGS_CMD_CONNECT_ACK),
                                                STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);

                            reject = false;
                        }
                    }
                }

                break;
            }

            case STREAM_FLAGS_CMD_CONNECT_ACK: {
                /* If are in connecting, respond with connect ack */
                if (socket != NULL) {
                    /* Cancel current state machine retry */
                    cancel_state_machine(socket);

                    if (socket->state == LS_STATE_OUTBOUND_CONNECT) {
                        /* Receiving a CONNECT ACK in the OUTBOUND connect, so send a CONNECT ACK and go the CONNECTED state */
                        start_state_machine(socket, LS_STATE_CONNECTED,
                                            stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_CONNECT_ACK),
                                            0, 0);

                        /* Finish building socket by adding queues */
                        socket->input_window = allocate_packet_window(CONFIG_LASTLINK_STREAM_WINDOW_SIZE, 0);
                        socket->output_window = allocate_packet_window(CONFIG_LASTLINK_STREAM_WINDOW_SIZE, CONFIG_LASTLINK_STREAM_WINDOW_SIZE);

                        /* Deliver success to caller of original connect */
                        send_state_machine_response(socket, LSE_NO_ERROR);
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
                            memset(socket, 0, sizeof(*socket));
                        }
                        /* Let ls_listen finish it's work */
                        send_state_machine_response(socket, LSE_NO_ERROR);
                    }
                }
                break;
            }

            case STREAM_FLAGS_CMD_DISCONNECT: {
                /* If still in connecting, respond with connect ack */
                if (socket != NULL) {
                    /* Cancel current state machine retry */
                    cancel_state_machine(socket);

                    /* In any state, start a disconnect */
                    start_state_machine(socket, LS_STATE_DISCONNECTING,
                                        stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_DISCONNECTED),
                                        STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);

                    /* This will shut down the reader when it reads to end of queue */
                    if (put_packet_into_window(socket->input_window, NULL)) {
                        reject = false;
                    }
                }
                break;
            }

            case STREAM_FLAGS_CMD_DISCONNECTED: {
                /* If are in connecting, respond with connect ack */
                if (socket != NULL) {
                    /* Cancel current state machine retry */
                    cancel_state_machine(socket);

                    socket->state = LS_STATE_DISCONNECTED;
                    send_state_machine_response(socket, LSE_NO_ERROR);
                    reject = false;
                }
                break;
            }

            case STREAM_FLAGS_CMD_REJECT: {
                /* If are in connecting, respond with connect ack */
                if (socket != NULL) {
                    /* Cancel current state machine retry */
                    cancel_state_machine(socket);

                    /* An error so tear down the connection */
                    start_state_machine(socket, LS_STATE_DISCONNECTED,
                                        stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_DISCONNECTED),
                                        STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);
                    reject = false;
                }
                break;
            }
        }

        if (reject) {
            linklayer_send_packet(stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_REJECT));
        } else if (socket != NULL) {
            /* Ack the output window */
            ack_output_window(socket,
                              get_uint_field(packet, STREAM_ACK_SEQUENCE, SEQUENCE_NUMBER_LEN),
                              get_uint_field(packet, STREAM_ACK_WINDOW, ACK_WINDOW_LEN));
        }

        processed = true;
    }

    return processed;
}

static const char* stream_packet_format(const packet_t *packet)
{
    char* info;

    const char* data = linklayer_escape_raw_data(packet->buffer + STREAM_PAYLOAD, packet->length - STREAM_PAYLOAD);

    asprintf(&info, "Stream: Src Port %d Dest Port %d Ack %d Seq %d Flags %02x \"%s\"",
            get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN),
            get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN),
            get_uint_field(packet, STREAM_ACK_SEQUENCE, SEQUENCE_NUMBER_LEN),
            get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN),
            get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN),
            data);

     free((void*) data);

     return info;
}



/*
 * Initialize ls_socket layer.
 */
ls_error_t ls_socket_init(void)
{
    ls_error_t err = 0;

    /* Register stream and datagram protocols */
    if (linklayer_register_protocol(PING_PROTOCOL,           ping_packet_process,          ping_packet_format)        &&
        linklayer_register_protocol(PINGREPLY_PROTOCOL,      pingreply_packet_process,     pingreply_packet_format)   &&
        linklayer_register_protocol(STREAM_PROTOCOL,         stream_packet_process,        stream_packet_format)      &&
        linklayer_register_protocol(DATAGRAM_PROTOCOL,       datagram_packet_process,      datagram_packet_format)) {

        /* Create the timer but don't start it */
        socket_timer = os_create_timer("socket_timer", 5, NULL, socket_timer_scanner);

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

    return err;
}

ls_error_t ls_socket_deinit(void)
{
    linklayer_lock();

    /* Kill socket timer */
    os_stop_timer(socket_timer);
    os_delete_timer(socket_timer);
    socket_timer = NULL;

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

    linklayer_unlock();

    return err;
}

static ssize_t release_socket(ls_socket_t* socket)
{
    ssize_t err = LSE_NO_ERROR;

    switch (socket->socket_type) {
        case LS_UNUSED: {
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
            socket->socket_type = LS_UNUSED;
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
                socket->socket_type = LS_UNUSED;
            } else {
                stop_socket_timer(&socket->state_machine_timer);
                stop_socket_timer(&socket->socket_flush_timer);
                stop_socket_timer(&socket->output_window_timer);

                /* Otherwise forcibly shut everything down */
                release_packet_window(socket->input_window);
                socket->input_window = NULL;
                release_packet_window(socket->output_window);
                socket->output_window = NULL;

                release_packet(socket->retry_packet);
                socket->retry_packet = NULL;

                if (socket->response_queue != NULL) {
                    os_delete_queue(socket->response_queue);
                    socket->response_queue = NULL;
                }

                release_packet(socket->current_read_packet);
                socket->current_read_packet = NULL;

                release_packet(socket->current_write_packet);
                socket->current_write_packet = NULL;

                socket->state = LS_STATE_IDLE;
                socket->socket_type = LS_UNUSED;
            }
            break;
        }
    }

    socket->inuse = false;

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

        if (linklayer_lock()) {
            ls_socket_t *socket = find_free_socket();

            if (socket != NULL) {
                socket->inuse = true;
                socket->dead = false;
                socket->socket_type = socket_type;
                socket->state = LS_STATE_IDLE;
                socket->local_port = -1;

                /* Return the socket index in the table plus the offset */
                ret = socket - socket_table + FIRST_LASTLINK_FD;
            } else {
                ret = LSE_NO_MEM;
            }

            linklayer_unlock();

        } else {
            ret = LSE_SYSTEM_ERROR;
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
    if (socket != NULL && socket->socket_type == LS_STREAM) {
        if (max_queue > 0 && max_queue < MAX_SOCKET_CONNECTIONS) {

            /* Only valid for STREAMS */
            /* Place socket in listen mode */

            if (linklayer_lock()) {
                if (socket->connections == NULL) {
                    /* Create queue for connections */
                    socket->connections = os_create_queue(max_queue, sizeof(ls_socket_t*));

                    socket->listen = true;
                }
                linklayer_unlock();

                /* Wait for something to arrive at connection queue */
                ls_socket_t *connection;
                if (os_get_queue_with_timeout(socket->connections, (os_queue_item_t*) &connection, timeout)) {
                    /* A new connection */
                    err = connection - socket_table;
                } else {
                    err = LSE_TIMEOUT;
                }
            } else {
                err = LSE_SYSTEM_ERROR;
            }
        } else {
            err = LSE_INVALID_MAXQUEUE;
        }
    } else {
        err = LSE_INVALID_SOCKET;
    }

    return err;
}

/* Called by repeating timer to keep machine running - retransmit data, etc. */
static void state_machine_timeout(ls_socket_t *socket)
{
    if (--(socket->retries) > 0) {

        /* Timed out so retry by sending connect packet again (if still present) */
        if (socket->retry_packet != NULL) {
            linklayer_send_packet(ref_packet(socket->retry_packet));
        }
    } else {
        /* Send error code to connect response queue */
        send_state_machine_response(socket, LSE_CONNECT_FAILED);
        /* This only stops the state machine mechanics.  response disappears when read. */
        cancel_state_machine(socket);
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
 *      packet       Initial packet to be sent (captured and retransmitted unless timeout, retry or answer)
 *      timeout      Timeout between retry cycles. (if 0, does not start timer)
 *      retries      Number of times to retry packet until state changes.
 *
 */
static void start_state_machine(ls_socket_t *socket, ls_socket_state_t state, packet_t *packet, int timeout, int retries)
{
    /* Cancel any running state machine */
    cancel_state_machine(socket);

    /* Validate socket */

    socket->state = state;
    if (timeout != 0) {
        socket->retries = retries;
        start_socket_timer(&socket->state_machine_timer, timeout);
        socket->retry_packet = ref_packet(packet);
    }

    if (socket->response_queue == NULL) {
        socket->response_queue = os_create_queue(1, sizeof(ls_error_t));
    }

ls_dump_socket_ptr("state changed", socket);
    if (packet != NULL) {
linklayer_print_packet("sending packet", packet);
        linklayer_send_packet(packet);
    }
}

void send_state_machine_response(ls_socket_t *socket, ls_error_t response)
{
    if (socket->response_queue != NULL) {
        os_put_queue(socket->response_queue, (os_queue_item_t) response);
    } else {
        ESP_LOGE(TAG, "%s: response queue is undefined", __func__);
    }
}

ls_error_t get_state_machine_response(ls_socket_t *socket)
{
    int response;
    if (! os_get_queue(socket->response_queue, (os_queue_item_t*) &response)) {
        response = LSE_SYSTEM_ERROR;
    }

    if (socket->response_queue != NULL) {
        os_delete_queue(socket->response_queue);
        socket->response_queue = NULL;
    } else {
        ESP_LOGE(TAG, "%s: response queue is undefined", __func__);
    }

    return (ls_error_t) response;
}

static void cancel_state_machine(ls_socket_t *socket)
{
    socket->retries = 0;

    stop_socket_timer(&socket->state_machine_timer);

    release_packet(socket->retry_packet);
    socket->retry_packet = NULL;

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
        socket->dest_port = port;
        socket->dest_addr = address;
        socket->serial_number = linklayer_allocate_sequence();  /* Unique serial number for this connection */

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
        } else {
            /* Start sending connect packet and wait for complete */
            start_state_machine(socket, LS_STATE_OUTBOUND_CONNECT,
                                stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_CONNECT),
                                STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);

            /* A response of some kind will be forthcoming.  It will be NO_ERROR or some other network error */
            err = get_state_machine_response(socket);
        }
    } else {
        err = LSE_INVALID_SOCKET;
    }

    if (err != LSE_INVALID_SOCKET) {
        socket->last_error = err;
    }

    return err;
}

static void socket_flush_stream(ls_socket_t *socket)
{
//linklayer_print_packet("socketflush", socket->current_write_packet);
/* Locks needed ? */
    stop_socket_timer(&socket->socket_flush_timer);
    assert(socket->output_window != NULL);

    if (socket->output_window != NULL) {
        write_packet_to_window(socket->output_window, ref_packet(socket->current_write_packet));
        release_packet(socket->current_write_packet);
        socket->current_write_packet = NULL;
ESP_LOGI(TAG, "%s: start output_window_timer", __func__);
        start_socket_timer(&socket->output_window_timer, CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY);
    }
}

/*
 * Send any packes in the output window (unacknowledged) + the ACK sequence and window flags.
 */
static void ack_output_window(ls_socket_t *socket, int ack_sequence, unsigned int ack_window)
{
    stop_socket_timer(&socket->output_window_timer);

ESP_LOGI(TAG, "%s: waiting for output window lock...", __func__);

    if (os_acquire_recursive_mutex(socket->output_window->lock)) {
ESP_LOGI(TAG, "%s: got output window lock...", __func__);

        /* Release these packets in window */
        release_packets_in_window(socket->output_window, socket->input_window->sequence, socket->input_window->window);

ESP_LOGI(TAG, "%s: released_packet_in_window done...", __func__);

        /* If packets remain in window, start the send_output_window timer again */
        if (socket->output_window->in != 0) {
            socket->output_window->retry_time = 0;
            start_socket_timer(&socket->output_window_timer, CONFIG_LASTLINK_STREAM_TRANSMIT_RETRY_TIME);
        }
          
        os_release_recursive_mutex(socket->output_window->lock);
    }
}

static void send_output_window(ls_socket_t *socket)
{
    stop_socket_timer(&socket->output_window_timer);

ESP_LOGI(TAG, "%s: waiting for window lock", __func__);

    if (os_acquire_recursive_mutex(socket->output_window->lock)) {
ESP_LOGI(TAG, "%s: input window in is %d", __func__, socket->output_window->in);
        /* If packets in window, then send them. */
        if (socket->output_window->in != 0) {
            /* If first time, set new timeout and retry count */
            if (socket->output_window->retries == 0) {
                socket->output_window->retries = CONFIG_LASTLINK_STREAM_TRANSMIT_RETRIES;
                socket->output_window->retry_time = CONFIG_LASTLINK_STREAM_TRANSMIT_RETRY_TIME;

            } else {
                socket->output_window->retries -= 1;
            }
 
            if (socket->output_window->retries != 0) {
ESP_LOGI(TAG, "%s: retries is %d", __func__, socket->output_window->retries);
                /* Packets in queue - send all that are here */
                for (int slot = 0; slot < socket->output_window->in; ++slot) {
                    if (socket->output_window->slots[slot] != NULL) {
ESP_LOGI(TAG, "%s: sending packet in slot %d", __func__, slot);
linklayer_print_packet("sending packet", socket->output_window->slots[slot]);
                        linklayer_send_packet(ref_packet(socket->output_window->slots[slot]));
                    }
                }

                /* Come back in exponential time */
                start_socket_timer(&socket->output_window_timer, socket->output_window->retry_time);

                /* Next time wait twice as long */
                socket->output_window->retry_time <<= 1;

            } else {
                /* Timed out - Mark it as dead */
                socket->dead = true;

            }
        }
        os_release_recursive_mutex(socket->output_window->lock);
    }
}

/*
 * Link and check every little while until socket has been disconnected.
 */
static void socket_linger_check(ls_socket_t* socket)
{
    if (!socket->inuse && socket->socket_type == LS_STREAM) {
        if (socket->state == LS_STATE_DISCONNECTED) {
            (void) release_socket(socket);
        } else {
            /* Look again later */
            start_socket_timer(&socket->linger_timer, CONFIG_LASTLINK_SOCKET_LINGER_TIME);
        }
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

    switch (socket->socket_type) {
        case LS_DATAGRAM: {
            packet_t *packet = datagram_packet_create_from_socket(socket);
            if (packet != NULL) {
                int tomove = len;
                if (tomove > DATAGRAM_LEN) {
                    tomove = DATAGRAM_LEN;
                }
                memcpy(packet->buffer + DATAGRAM_PAYLOAD, buf, tomove);
                packet->length += tomove;
                linklayer_send_packet(packet);
                err = tomove;
            } else {
                err = LSE_NO_MEM;
            }
            break;
        }

        case LS_STREAM: {
            int written = 0;

            if (len != 0 || eor) { 
                while (len != 0) {
                    if (socket->current_write_packet == NULL) {
                        socket->current_write_packet = stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_DATA);
                    }

                    /* room available in  the packet buffer */
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
                        write_packet_to_window(socket->output_window, socket->current_write_packet);
                        socket->current_write_packet = NULL;
                    }
                }

                /*
                 * Last packet generated.  If eor, set the EOR flag and send it, otherwise if packet is full, write it.
                 */
                if (eor || socket->current_write_packet->length == MAX_PACKET_LEN) {
                    if (eor) {
                        int flags = get_uint_field(socket->current_write_packet, STREAM_FLAGS, FLAGS_LEN);
                        flags |= STREAM_FLAGS_EOR;
                        set_uint_field(socket->current_write_packet, STREAM_FLAGS, FLAGS_LEN, flags);
                    }
                    write_packet_to_window(socket->output_window, socket->current_write_packet);
                    socket->current_write_packet = NULL;
                } else {
                    /* Set a timer with CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY to force transmission if nothing else is stored in packet. */
                    start_socket_timer(&socket->socket_flush_timer, CONFIG_LASTLINK_STREAM_TRANSMIT_DELAY);
                }
            }
            err = written;
            break;
        }

        default: {
            err = LSE_NOT_WRITABLE;
            break;
       }
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
        err = LSE_INVALID_SOCKET;
    } else if (socket->socket_type == LS_DATAGRAM) {
        /* Pend on a packet in the queue */
        packet_t *packet;
        if (os_get_queue_with_timeout(socket->datagram_packets, (os_queue_item_t*) &packet, timeout)) {
//linklayer_print_packet("ls_read DATAGRAM", packet);
            /* A packet with data */
            int packet_data_length = packet->length - DATAGRAM_PAYLOAD;
            if (packet_data_length > maxlen) {
                packet_data_length = maxlen;
            }
            memcpy(buf, packet->buffer + DATAGRAM_PAYLOAD, packet_data_length);
            if (address != NULL) {
                *address = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
            }
            if (port != NULL) {
                *port = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN);
            }
            err = packet_data_length;
            release_packet(packet);
        } else {
            err = LSE_TIMEOUT;
        }
    } else if (socket->socket_type == LS_STREAM) {
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
                /* This blocks until packet is ready or socket is shut down remotely */
                packet = get_packet_from_window(socket->input_window);
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

                eor = (get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN) & STREAM_FLAGS_EOR) != 0;

                /* If this was end of record or we consumed all the data in the packet, release it */
                if (eor ||  offset == packet->length - STREAM_PAYLOAD) {
                    release_packet(packet);
                } else {
                    /* Otherwise save the packet and offset */
                    socket->current_read_packet = packet;
                    socket->current_read_offset = offset;
                }
            } else {
                /* Socket remotely closed.  Return residue for last read. */
                eor = true;
                /* Shutdown socket */
                //ls_close_immediate(s);
            }
        }
    } else {
        err = LSE_INVALID_SOCKET;
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
        if (linklayer_lock()) {
            switch (socket->socket_type) {
                default: {
                    err = LSE_INVALID_SOCKET;
                    break;
                }

                case LS_DATAGRAM: {
                    err = release_socket(socket);
                    break;
                }

                case LS_STREAM: {
                    if (socket->listen) {
                        err = release_socket(socket);
                    } else {
                        /* Flush any remaining output */
                        socket_flush_stream(socket);

                        /*
                         * Start a close by sending disconnect to remote end.
                         * Socket will no longer be writable by can be read until end of data.
                         * When reaching end data, local socket will also close.
                         */
                        start_state_machine(socket, LS_STATE_DISCONNECTING,
                                            stream_packet_create_from_socket(socket, STREAM_FLAGS_CMD_DISCONNECT),
                                            STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);

                        err = get_state_machine_response(socket);

                        /* After purged queues finish or a suitable timeout, socket will be cleared */
                        start_socket_timer(&socket->linger_timer, CONFIG_LASTLINK_SOCKET_LINGER_TIME);
                    }
                    break;
                }
            }

            socket->inuse = false;

            linklayer_unlock();
        }
    } else {
        err = LSE_INVALID_SOCKET;
    }

    return err;
}

ls_error_t ls_close(int s)
{
    return ls_close_ptr(validate_socket(s));
}


static ls_socket_t *validate_socket(int s)
{
    ls_socket_t *ret = NULL;

    if (s >= FIRST_LASTLINK_FD && s <= LAST_LASTLINK_FD) {

        s -= FIRST_LASTLINK_FD;
    
        if (s >= 0 && s < ELEMENTS_OF(socket_table)) {
            if (socket_table[s].socket_type == LS_DATAGRAM || socket_table[s].socket_type == LS_STREAM) {
                ret = &socket_table[s];
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
    } else {
        err = LSE_INVALID_SOCKET;
    }

    return err;
}

/*
 * Put a packet into the queue at the expected sequence number entry.
 * Rejects action if outside of window.
 *
 * Entry:
 *      queue               Queue holding packets
 *      packet              Packet to add to queue
 *
 * Returns true if success.
 *
 */
static bool put_packet_into_window(packet_window_t *queue, packet_t *packet)
{
    bool ok = false;

    if (queue != NULL) {
ESP_LOGI(TAG, "%s: waiting for lock", __func__);
        if (os_acquire_recursive_mutex(queue->lock)) {
            int relative_sequence;
            int ready_count = 0;

            if (packet != NULL) {
                relative_sequence = get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN) - queue->sequence;
ESP_LOGI(TAG, "%s: relative sequence %d from packet %d queue %d", __func__, relative_sequence, get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN), queue->sequence);
            } else {
ESP_LOGI(TAG, "%s: packet was NULL", __func__);
                /* One extra release so the end of data is seen */
                ready_count = 1;

                /* Find first available entry and use it. */
                relative_sequence = 0;
                while (queue->slots[relative_sequence] != NULL && relative_sequence < queue->length) {
                    ++relative_sequence;
                }
                /* relative_sequence now points to first available entry or past the end if none found */
            }

ESP_LOGI(TAG, "%s: relative_sequence %d queue->length %d", __func__, relative_sequence, queue->length);
            if (relative_sequence >= 0 && relative_sequence < queue->length) {
                /* Ignore packet if already installed */
                if (queue->slots[relative_sequence] == NULL) {
                    queue->slots[relative_sequence] = ref_packet(packet);
                    queue->window |= 1 << relative_sequence;
ESP_LOGI(TAG, "%s: packet stored in slot %d; window %04x queue->in %d", __func__, relative_sequence, queue->window, queue->in);
                    ok = true;
                }
            }

            /* Make available all packets from here that are in the window */
            while ((queue->window & (1 << queue->in)) != 0) {
                ready_count++;
                queue->in++;
            }
      
ESP_LOGI(TAG, "%s: %d packets ready; in is now %d", __func__, ready_count, queue->in);

            /* Release as many slots as have become contiguous */
            if (ready_count != 0) {
                bool rok = os_release_counting_semaphore(queue->available, ready_count);
ESP_LOGI(TAG, "%s: release count %d returned %s", __func__, ready_count, rok ? "TRUE" : "FALSE");
            }

            os_release_recursive_mutex(queue->lock);
        }
    }

    return ok;
}

/*
 * Get a stream packet from window for delivery to user.
 *
 * When NULL is returned, the socket was closed remotely.
 */
static packet_t *get_packet_from_window(packet_window_t *queue)
{
    packet_t *packet = NULL;

    if (os_acquire_counting_semaphore(queue->available)) {
        /* Remove first entry */
        if (os_acquire_recursive_mutex(queue->lock)) {

            /* Take the top of the queue */
            packet = queue->slots[0];

            /* If nothing in queue, we are closing */
            if (packet != NULL) {
                if (queue->in > 1) {
                    memcpy(queue->slots + 0, queue->slots + 1, (queue->in - 1) * sizeof(queue->slots[0]));
                }
                queue->in--;
                queue->slots[queue->in] = NULL;
                queue->window >>= 1;
                queue->sequence++;

            }
            os_release_recursive_mutex(queue->lock);
        }
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
 *        queue             Queue to receive packets.
 *        packet            Packet to place in queue.  Will be ref'd if accepted.
 *
 * Returns true on success.
 */
static bool write_packet_to_window(packet_window_t *queue, packet_t *packet)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: waiting for available...", __func__);

    if (os_acquire_counting_semaphore(queue->available)) {
        /* Room to put the packet.  Lock the queue */
ESP_LOGI(TAG, "%s: waiting for lock...", __func__);
        if (os_acquire_recursive_mutex(queue->lock)) {
ESP_LOGI(TAG, "%s: got lock...", __func__);
            /* Sanity check */
            if (queue->in < queue->length) {
ESP_LOGI(TAG, "%s: putting packet %p in queue at %d", __func__, packet, queue->in);
                /* Insert sequence number into packet */
                set_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN, queue->sequence + queue->in);
                queue->slots[queue->in++] = ref_packet(packet);
                ok = true;
            }
            os_release_recursive_mutex(queue->lock);
        }
    }
    return ok;
}

            
/*
 * Called when a sequence ACK and window are received.
 *
 * The sequence number is the NEXT expected sequence to be received by the caller.
 * The window gives hints as to any out-of-sequence packets that have been received.
 * 
 * This function releases the packets, rolls up the queue and release the appropriate
 * count to the semaphore describing the room available.
 *
 * Entry:
 *      queue           - queue containing the packets being processed.
 *      sequence        - sequence number being acked.
 *      window          - additional packets (bit field relative to sequence) that are acked.
 *
 */
static void release_packets_in_window(packet_window_t *queue, int sequence, unsigned int window)
{
    if (os_acquire_recursive_mutex(queue->lock)) {
ESP_LOGI(TAG, "%s: sequence %d window %04x", __func__, sequence, window);
        /* Ack the head of the queue, if any */
        while (queue->in != 0 && queue->slots[0] != NULL &&  (sequence < get_uint_field(queue->slots[0], STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN))) {
ESP_LOGI(TAG, "%s: release packet %p in window slot 0 (in %d)", __func__, queue->slots[0], queue->in);
            release_packet(queue->slots[0]);

            /* Remove from table */
            if (queue->in > 1) {
                memcpy(queue->slots + 0, queue->slots + 1, (queue->in - 1) * sizeof(queue->slots[0]));
            }
            queue->in--;
            queue->slots[queue->in] = NULL;
             
            /* Immediate release when queue is squished */
            os_release_counting_semaphore(queue->available, 1);
        }

        /* Release the other packets defined by the window */
        int residue = 1;
        while (window != 0) {
            if (((window & 1) != 0) && (queue->slots[residue] != NULL)) {
ESP_LOGI(TAG, "%s: release packet %p in window slot %d", __func__, queue->slots[residue], residue);
                release_packet(queue->slots[residue]);
                queue->slots[residue] = NULL;
                queue->released++;
            }
            window >>= 1;
            residue++;
        }

        /* When queue goes empty, release all pending releases */
        if (queue->in == 0 && queue->released != 0) {
            /* Queue went from non-empty to empty so give space back to the caller */
ESP_LOGI(TAG, "%s: released %d in queue", __func__, queue->released);
            os_release_counting_semaphore(queue->available, queue->released);
            queue->released = 0;
        }

        os_release_recursive_mutex(queue->lock);
    }
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
     packet_window_t *queue = (packet_window_t*) malloc(sizeof(packet_window_t) + sizeof(packet_t*) * (length - 1));

     if (queue != NULL) {
         memset(queue, 0, sizeof(packet_window_t) + sizeof(packet_t*) * (length - 1));
         queue->available = os_create_counting_semaphore(length, available);
assert(queue->available != NULL);
         queue->lock = os_create_recursive_mutex();
         queue->length = length;
     }

     return queue;
}

static bool release_packet_window(packet_window_t *queue)
{
    if (os_acquire_recursive_mutex(queue->lock)) {
        os_delete_semaphore(queue->available);

        /* Release packets in queue */
        for (int index = 0; index < queue->length; ++index) {
            release_packet(queue->slots[index]);
            queue->slots[index] = NULL;
        }
        os_delete_mutex(queue->lock);
    }

    free((void*) queue);

    return true;
}

static const char* socket_type_of(const ls_socket_t *socket)
{
    const char* type = "UNDEFINED";
    if (socket != NULL) {
        switch (socket->socket_type) {
            case LS_UNUSED:   type = "Unused";    break;
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
        case LS_STATE_IDLE:             return "Idle";
        case LS_STATE_INBOUND_CONNECT:  return "Inbound";
        case LS_STATE_OUTBOUND_CONNECT: return "Outbound";
        case LS_STATE_CONNECTED:        return "Connected";
        case LS_STATE_DISCONNECTING:    return "Disconnecting";
        case LS_STATE_DISCONNECTED:     return "Disconnected";
        default:                        return "UNKNOWN";
    }
}

static ls_error_t ls_dump_socket_ptr(const char* msg, const ls_socket_t *socket)
{
    if (socket != NULL) {
        printf("%s: %d (%p) state %s listen %s type %s local port %d dest port %d dest addr %d sn %d parent %d\n",
               msg, socket - socket_table + FIRST_LASTLINK_FD,
               socket,
               socket_state_of(socket),
               socket->listen ? "YES" : "NO",
               socket_type_of(socket),
               socket->local_port,
               socket->dest_port,
               socket->dest_addr,
               socket->serial_number,
               socket->parent ? (socket->parent - socket_table) : -1);
    } else {
        printf("%s: invalid socket\n", msg);
    }

    return 0; 
}

ls_error_t ls_dump_socket(const char* msg, int s)
{
    return ls_dump_socket_ptr(msg, validate_socket(s));
}
    
/* Called by timer to work on the sockets.
 * Rechedules itself when new time interval is needed.
 */
static void socket_timer_scanner(os_timer_t timer)
{
    if (linklayer_lock()) {
        uint32_t next_time = 0;
        uint64_t current_time = get_milliseconds();

        for (int socket_index = 0; socket_index < ELEMENTS_OF(socket_table); ++socket_index) {
            /* Check the socket timer and fire action is ready */
            if (socket_timer_check(&socket_table[socket_index].state_machine_timer, current_time, &next_time)) {
ESP_LOGI(TAG, "%s: state_machine_timer for socket %d fired", __func__, socket_index);
               state_machine_timeout(&socket_table[socket_index]);
            }

            if (socket_timer_check(&socket_table[socket_index].socket_flush_timer, current_time, &next_time)) {
ESP_LOGI(TAG, "%s: socket_flush_timer for socket %d fired", __func__, socket_index);
                 socket_flush_stream(&socket_table[socket_index]);
            }

            if (socket_timer_check(&socket_table[socket_index].output_window_timer, current_time, &next_time)) {
ESP_LOGI(TAG, "%s: output_window_timer for socket %d fired", __func__, socket_index);
                 send_output_window(&socket_table[socket_index]);
            }

            if (socket_timer_check(&socket_table[socket_index].linger_timer, current_time, &next_time)) {
ESP_LOGI(TAG, "%s: linger_timer for socket %d fired", __func__, socket_index);
                 socket_linger_check(&socket_table[socket_index]);
            }
        }

        if (next_time != 0) {
            /* Set delay to next trigger */
            os_set_timer(socket_timer, next_time);
            /* And start it */
            os_start_timer(socket_timer);
        }

        linklayer_unlock();
    }
}

static bool socket_timer_check(socket_timer_t *timer, uint64_t current_time, uint32_t *next_time)
{
    bool fired = timer->expires != 0 && current_time >= timer->expires;

    /* If it didn't fire, calculate next time to fire */
    if (!fired) {
        uint32_t time_to_fire = timer->expires - current_time;
        if (*next_time == 0 || time_to_fire < *next_time) {
            *next_time = time_to_fire;
        }
    } 

    return fired;
}

static void start_socket_timer(socket_timer_t *timer, int delay)
{
    timer->expires = delay + get_milliseconds();

    /* Force a scan to reschedule */
    os_set_timer(socket_timer, 1);
    os_start_timer(socket_timer);
}

static void stop_socket_timer(socket_timer_t *timer)
{
    timer->expires = 0;
}

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_LAYER */
