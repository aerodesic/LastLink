/*
 * lsocket.c
 *
 * Lightweight Socket layer.
 *
 * Example DATAGRAM:
 *      // Create socket
 *      int socket = ls_socket(LS_DATAGRAM);
 *      // Bind port
 *      ls_bind(socket, 1234)
 *      ls_connect(socket, <destination address>, <destination port>)
 *      int len = ls_read(socket, buf, buflen);
 *          -or-
 *      int len = ls_write(socket, buf, buflen);
 *          ... more activity ...
 *      ls_close(socket);
 *
 *
 * Example OUTBOUND STREAM:
 *      // Create socket
 *      int socket = ls_socket(LS_STREAM);
 *      // Bind port
 *      ls_bind(socket, 1234)
 *      int rc = ls_connect(socket, <destination address>, <destination port>)
 *      if (rc == LS_NO_ERROR) {
 *          int len = ls_read(socket, buf, buflen);
 *              -or-
 *          int len = ls_write(socket, buf, buflen);
 *          ... more activity ...
 *          ls_close(socket);
 *      }
 *
 * Example OUTBOUND STREAM:
 *      // Create socket
 *      int listen_socket = ls_socket(LS_STREAM);
 *      // Bind port
 *      ls_bind(socket, 1234)
 *      int new_connection = ls_listen(socket, 5, 0);
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
#include "esp_system.h"
#include "esp_log.h"

#include "os_freertos.h"
#include "lsocket_internal.h"
#include "packets.h"
#include "linklayer.h"

#define TAG "lsocket"

/* PING support */
static const char* ping_packet_format(const packet_t* p);
static const char* pingreply_packet_format(const packet_t* p);
static bool ping_packet_process(packet_t* p);
static bool pingreply_packet_process(packet_t* p);
static int find_ping_table_entry(int sequence);

static ls_error_t ls_write_helper(ls_socket_t* socket, const char* buf, int len, bool eor);
static ls_socket_t*  validate_socket(int socket);

static bool stream_packet_process(packet_t* packet);
static const char* stream_packet_format(const packet_t* packet);
static packet_t* assemble_socket_data(packet_t* packet_to_send, packet_t* packet);

static bool datagram_packet_process(packet_t* packet);
static const char* datagram_packet_format(const packet_t* packet);

static packet_t* datagram_packet_create_from_packet(const packet_t* packet);
static packet_t* datagram_packet_create_from_socket(const ls_socket_t* socket);

static packet_t* stream_packet_create_from_packet(const packet_t* packet, uint8_t flags);
static packet_t* stream_packet_create_from_socket(const ls_socket_t* socket, uint8_t flags);

static ls_socket_t* find_socket_from_packet(const packet_t* packet, ls_socket_type_t type);

static void start_state_machine(ls_socket_t* socket, ls_socket_state_t state, packet_t* packet, int timeout, int retries);
static void cancel_state_machine(ls_socket_t* socket);
static void send_state_machine_response(ls_socket_t* socket, ls_error_t response);
static ls_error_t get_state_machine_response(ls_socket_t* socket);

static ls_error_t ls_close_helper(int socket, bool immediate);
static ls_error_t ls_close_immediate(int socket);

typedef struct ping_table_entry {
    os_queue_t   queue;
    int          sequence;
} ping_table_entry_t;

ping_table_entry_t   ping_table[CONFIG_LASTLINK_MAX_OUTSTANDING_PINGS];

static ls_socket_t  socket_table[CONFIG_LASTLINK_NUMBER_OF_SOCKETS];

/*
 * Ping packet create
 */
packet_t* ping_packet_create(int dest)
{
    packet_t* p;

    p = linklayer_create_generic_packet(dest, PING_PROTOCOL, PING_LEN);

    if (p != NULL) {
        /* Generate sequence number */
        set_uint_field(p, PING_SEQUENCE, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());

        /* Put starting address in the table */
        set_uint_field(p, PING_ROUTE_TABLE, ADDRESS_LEN, linklayer_node_address);
    }

    return p;
}

static bool ping_packet_process(packet_t* p)
{
    bool processed = false;

    if (p != NULL) {
linklayer_print_packet("PING RECEIVED", p);

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

linklayer_print_packet("SEND PING REPLY", p);

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

static const char* ping_format_routes(const packet_t* p)
{
    int num_routes = (p->length - PING_ROUTE_TABLE) / ADDRESS_LEN;

//ESP_LOGI(TAG, "%s: num_routes %d", __func__, num_routes);

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

static const char* ping_packet_format(const packet_t* p)
{
    char* info;

    int sequence = get_uint_field(p, PING_SEQUENCE, SEQUENCE_NUMBER_LEN);
    const char* routes = ping_format_routes(p);

    asprintf(&info, "Ping: Seq: %d Routes %s", sequence, routes);

    free((void*) routes);

    return info;
}

static const char* pingreply_packet_format(const packet_t* p)
{
    char* info;

    int sequence = get_uint_field(p, PING_SEQUENCE, SEQUENCE_NUMBER_LEN);
    const char* routes = ping_format_routes(p);

    asprintf(&info, "Ping Reply: Seq: %d Routes %s", sequence, routes);

    free((void*) routes);

    return info;
}

static bool pingreply_packet_process(packet_t* p)
{
    bool processed = false;

    if (p != NULL) {
        if (linklayer_lock()) {

linklayer_print_packet("PING REPLY RECEIVED", p);

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

static int register_ping(const packet_t* packet)
{
    int slot = find_ping_table_entry(0);
    if (slot >= 0) {
        /* Remember we are waiting on this sequence return */
        ping_table[slot].queue = os_create_queue(1, sizeof(packet_t*));
        ping_table[slot].sequence = get_uint_field(packet, PING_SEQUENCE, SEQUENCE_NUMBER_LEN);
    }

    return slot;
}

static packet_t* wait_for_ping_reply(int slot, int timeout)
{
    packet_t* packet = NULL;

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
    ls_error_t ret = 0;

    packet_t* packet = ping_packet_create(address);

    if (packet != NULL) {
        int slot = register_ping(packet);

        if (slot >= 0) {

            linklayer_send_packet(packet);

            packet = wait_for_ping_reply(slot, timeout);

            if (packet != NULL) {
                /* Pass information back to caller */
                int num_routes = (packet->length - PING_ROUTE_TABLE) / ADDRESS_LEN;

                for (int route = 0; pathlistlen > 0 && route < num_routes; ++route) {
                    pathlist[route] = get_uint_field(packet, PING_ROUTE_TABLE + ADDRESS_LEN * route, ADDRESS_LEN);
                    pathlistlen--;
                }

                ret = num_routes;
            } else {
                ret = LSE_TIMEOUT;
            }

        } else {
            ret = LSE_NO_MEM;
        }

        release_packet(packet);
    } else {
        ret = LSE_NO_MEM;
    }

    return ret;
}


static ls_socket_t* find_free_socket(void)
{
    ls_socket_t* socket = NULL;

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
            for (int socket = 0; !collision && socket < ELEMENTS_OF(socket_table); ++socket) {
                if (next_local_port == socket_table[socket].local_port) {
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
static ls_socket_t* find_socket_from_packet(const packet_t* packet, ls_socket_type_t type)
{
    ls_socket_t* socket = NULL;

    ls_port_t dest_port = get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN);
    ls_port_t src_port  = get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN);
    int dest_addr       = get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN);

    for (int index = 0; socket == NULL && index < CONFIG_LASTLINK_NUMBER_OF_SOCKETS; ++index) {
        if (socket_table[index].socket_type == type && socket_table[index].dest_addr == dest_addr &&
            socket_table[index].local_port == dest_port && socket_table[index].dest_port == src_port) {

            socket = &socket_table[index];
        }
    }

    /* Return socket if we found it */
    return socket;
}

static packet_t* datagram_packet_create_from_packet(const packet_t* packet)
{
    packet_t* new_packet = linklayer_create_generic_packet(get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                                           DATAGRAM_PROTOCOL, DATAGRAM_PAYLOAD);

    if (new_packet != NULL) {
        set_uint_field(new_packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN, get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN));
        set_uint_field(new_packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN, get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN));
    }

    return new_packet;
}

static packet_t* datagram_packet_create_from_socket(const ls_socket_t* socket)
{
    packet_t* packet = linklayer_create_generic_packet(socket->dest_addr, DATAGRAM_PROTOCOL, DATAGRAM_PAYLOAD);

    if (packet != NULL) {
        set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN, socket->dest_port);
        set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN, socket->local_port);
    }

    return packet;
}

static bool datagram_packet_process(packet_t* packet)
{
    bool processed = false;

    if (packet != NULL && linklayer_packet_is_for_this_node(packet)) {
        ls_socket_t* socket = find_socket_from_packet(packet, LS_DATAGRAM);

        if (socket != NULL) {
            /* Send the packet to the datagram input queue */
            if (!os_put_queue(socket->received_packets, (os_queue_item_t) ref_packet(packet))) {
                /* Not able to queue, so just log and drop it */
                linklayer_print_packet("Unable to queue", packet);
                release_packet(packet);  /* Release the ref above */
            }

            processed = true;
        }
    }

    return processed;
}

static const char* datagram_packet_format(const packet_t* packet)
{
    char* info;

    const char* data = linklayer_escape_raw_data(packet->buffer + DATAGRAM_PAYLOAD, packet->length - DATAGRAM_PAYLOAD);

    asprintf(&info, "Datagram: Src Port %d Dest Port %d \"%s\"",
            get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN),
            get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN),
            data);

     free((void*) data);

     return info;
}

static packet_t* stream_packet_create_from_packet(const packet_t* packet, uint8_t flags)
{
    packet_t* new_packet = linklayer_create_generic_packet(get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                                           STREAM_PROTOCOL, STREAM_PAYLOAD);

    if (new_packet != NULL) {
        set_uint_field(new_packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN, get_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN));
        set_uint_field(new_packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN, get_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN));
        set_uint_field(new_packet, STREAM_FLAGS, FLAGS_LEN, flags);
    }

    return new_packet;
}

static packet_t* stream_packet_create_from_socket(const ls_socket_t* socket, uint8_t flags)
{
    packet_t* packet = linklayer_create_generic_packet(socket->dest_addr, STREAM_PROTOCOL, STREAM_PAYLOAD);

    if (packet != NULL) {
        set_uint_field(packet, DATAGRAM_DEST_PORT, PORT_NUM_LEN, socket->dest_port);
        set_uint_field(packet, DATAGRAM_SRC_PORT, PORT_NUM_LEN, socket->local_port);
        set_uint_field(packet, STREAM_FLAGS, FLAGS_LEN, flags);
    }

    return packet;
}
static packet_t* assemble_socket_data(packet_t* packet_to_send, packet_t* packet)
{
    return packet_to_send;
}

static bool stream_packet_process(packet_t* packet)
{
    bool processed = false;

    if (packet != NULL && linklayer_packet_is_for_this_node(packet)) {
        /* Read the flags field for later use */
        uint8_t flags = get_uint_field(packet, STREAM_FLAGS, FLAGS_LEN);

        ls_socket_t* socket = find_socket_from_packet(packet, LS_STREAM);

        if (socket != NULL) {
            /* Cancel current state machine retry */
            cancel_state_machine(socket);

            switch (flags & STREAM_FLAGS_CMD) {
                default: {
                    linklayer_print_packet("BAD STREAM", packet);
                    /* Ignore noise */
                    break;
                }
                case STREAM_FLAGS_CMD_NOP: {
                    /* If we are connected, process the data packet */
                    break;
                }

                case STREAM_FLAGS_CMD_CONNECT: {
                    /* If we are listening, and connecting or connected, respond with connect ack */
                    if (socket->listen) {
                        if (socket->state == LS_STATE_IDLE || socket->state == LS_STATE_INBOUND_CONNECT) {
                            start_state_machine(socket, LS_STATE_INBOUND_CONNECT,
                                                stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_CONNECT_ACK),
                                                STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);
                        }
                    } else {
                        /* Send a reject - socket not in listen mode */
                        linklayer_send_packet(stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_REJECT));
                    }
                    break;
                }

                case STREAM_FLAGS_CMD_CONNECT_ACK: {
                    /* If are in connecting, respond with connect ack */
                    if (socket->state == LS_STATE_OUTBOUND_CONNECT) {
                        start_state_machine(socket, LS_STATE_CONNECTED,
                                            stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_CONNECT_ACK),
                                            STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);

                        /* Deliver success to caller */
                        send_state_machine_response(socket, LSE_NO_ERROR);
                    } else if (socket->state == LS_STATE_INBOUND_CONNECT) {
                        socket->state = LS_STATE_CONNECTED;
                        /* Launch a new socket for delivering to caller */
                    }
                    break;
                }

                case STREAM_FLAGS_CMD_DISCONNECT: {
                    /* In any state, start a disconnect */
                    start_state_machine(socket, LS_STATE_DISCONNECTING,
                                        stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_DISCONNECTED),
                                        STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);
                    break;
                }

                case STREAM_FLAGS_CMD_DISCONNECTED: {
                    socket->state = LS_STATE_DISCONNECTED;
                    send_state_machine_response(socket, LSE_NO_ERROR);
                    break;
                }

                case STREAM_FLAGS_CMD_REJECT: {
                    /* An error so tear down the connection */
                    start_state_machine(socket, LS_STATE_DISCONNECTED,
                                        stream_packet_create_from_packet(packet, STREAM_FLAGS_CMD_DISCONNECTED),
                                        STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);
                    break;
                }
            }

            /*
             * Pass the data through the assembly phase.  This might require modifying the current outbound packet
             * to update ack seqeuence numbers, etc.  In some cases a brand new packet will be generated.
             */
            if ((flags & STREAM_FLAGS_DATA) != 0) {
                /* Process any data if present */
            }

            processed = true;
        }
    }

    return processed;
}

static const char* stream_packet_format(const packet_t* packet)
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

        /* Initialize queues and such */

    } else {
        err = LSE_CANNOT_REGISTER;
    }

    return err;
}

ls_error_t ls_socket_deinit(void)
{
    linklayer_lock();

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
    for (int socket = 0; socket < ELEMENTS_OF(socket_table); ++socket) {
        ls_close_immediate(socket);
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

/*
 * Create a socket for sending or listening (determined by ls_connect or ls_listen)
 *
 * Entry:
 *      socket_type         LS_DATAGRAM or LS_STREAM
 *
 * Returns:
 *      ls_error_t          if < 0 an error, otherwise socket number.
 */
ls_error_t ls_socket(ls_socket_type_t socket_type)
{
    ls_error_t  ret;

    if (linklayer_lock()) {
        ls_socket_t* socket = find_free_socket();

        if (socket != NULL) {

            /* Validate parameters */
            if (socket_type == LS_DATAGRAM || socket_type == LS_STREAM) {
                socket->socket_type = socket_type;

                ret = socket - socket_table;
            } else {
                ret = LSE_BAD_TYPE;
            }
        } else {
            ret = LSE_NO_MEM;
        }

        linklayer_unlock();
    } else {
        ret = LSE_SYSTEM_ERROR;
    }

    return ret;
}

/*
 * ls_bind
 *
 * Bind an port to the local port.
 */
ls_error_t ls_bind(int socket, ls_port_t local_port)
{
    ls_error_t ret = LSE_INVALID_SOCKET;

    ls_socket_t* s = validate_socket(socket);

    if (s != NULL) {
        /* Assign local port from free pool if not specified by user */
        s->local_port = local_port ? local_port : find_free_local_port();

        ret = LSE_NO_ERROR;
    }

    return ret;
}

/*
 * Listen for connections on this socket.
 *
 * Entry:
 *      socket              The socket to listen on
 *      max_queue           Maximum number of simultaneous connections allowed
 *      timeout             How many mS to wait for a connection (0 is infinite)
 *
 * Returns:
 *      socket number if >= 0 else ls_error_t
 *
 */
ls_error_t ls_listen(int socket, int max_queue, int timeout)
{
    ls_error_t ret;

    ls_socket_t* s = validate_socket(socket);

    /* Validate paramters */
    if (s != NULL && s->socket_type == LS_STREAM) {
        if (max_queue > 0 && max_queue < MAX_SOCKET_CONNECTIONS) {

            /* Only valid for STREAMS */
            /* Place socket in listen mode */

            if (linklayer_lock()) {
                if (s->connections == NULL) {
                    /* Create queue for connections */
                    s->connections = os_create_queue(max_queue, sizeof(ls_socket_t*));

                    s->listen = true;
                }
                linklayer_unlock();

                /* Wait for something to arrive at connection queue */
                ls_socket_t *connection;
                if (os_get_queue_with_timeout(s->connections, (os_queue_item_t*) &connection, timeout)) {
                    /* A new connection */
                    ret = connection - socket_table;
                } else {
                    ret = LSE_TIMEOUT;
                }
            } else {
                ret = LSE_SYSTEM_ERROR;
            }
        } else {
            ret = LSE_INVALID_MAXQUEUE;
        }
    } else {
        ret = LSE_INVALID_SOCKET;
    }

    return ret;
}

/* Called by repeating timer to keep machine running - retransmit data, etc. */
static void state_machine_timeout(void* param)
{
    ls_socket_t* socket = (ls_socket_t*) param;

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
 *      timeout      Timeout between retry cycles.
 *      retries      Number of times to retry packet until state changes.
 *
 */
static void start_state_machine(ls_socket_t* socket, ls_socket_state_t state, packet_t* packet, int timeout, int retries)
{
    /* Cancel any running state machine */
    cancel_state_machine(socket);

    /* Validate socket */

    socket->state = state;
    socket->retries = retries;
    socket->retry_timer = os_create_timer("connecting", timeout, false, (void*) socket, state_machine_timeout);
    socket->retry_packet = ref_packet(packet);

    if (socket->response_queue == NULL) {
        socket->response_queue = os_create_queue(1, sizeof(ls_error_t));
    }

    if (packet != NULL) {
        linklayer_send_packet(packet);
    }
}

void send_state_machine_response(ls_socket_t* socket, ls_error_t response)
{
    if (socket->response_queue != NULL) {
        os_put_queue(socket->response_queue, (os_queue_item_t) response);
    } else {
        ESP_LOGE(TAG, "%s: response queue is undefined", __func__);
    }
}

ls_error_t get_state_machine_response(ls_socket_t* socket)
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

static void cancel_state_machine(ls_socket_t* socket)
{
    socket->retries = 0;

    if (socket->retry_timer != NULL) {
        os_delete_timer(socket->retry_timer);
        socket->retry_timer = NULL;
    }

    release_packet(socket->retry_packet);
    socket->retry_packet = NULL;

    /* Leave response queue in place so receiver gets the message */
}


/*
 * Connect to a remote port.
 *
 * Entry:
 *      socket              Socket from which to make the connection
 *      address             Target address of node
 *      port                Target port on node
 *
 * Returns:
 *      ls_error_t          LSE_NO_ERROR (0) if successful otherwise error code.
 */
ls_error_t ls_connect(int socket, ls_address_t address, ls_port_t port)
{
    ls_error_t ret = LSE_NO_ERROR;

    ls_socket_t* s = validate_socket(socket);

    if (s != NULL) {
        s->dest_port = port;
        s->dest_addr = address;
        s->serial_number = linklayer_allocate_sequence();  /* Unique serial number for this connection */

        if (s->socket_type == LS_DATAGRAM) {
            /* Datagram connection is easy - we just say it's connected */
            s->state = LS_STATE_CONNECTED;
        } else {
            /* Set a timer to check on results after no answer */
            start_state_machine(s, LS_STATE_OUTBOUND_CONNECT,
                                stream_packet_create_from_socket(s, STREAM_FLAGS_CMD_CONNECT),
                                STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);

            /* Wait for response indicating connection or failure */
            ret = get_state_machine_response(s);
        }
    } else {
        ret = LSE_INVALID_SOCKET;
    }

    if (ret != LSE_INVALID_SOCKET) {
        s->last_error = ret;
    }

    return ret;
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
static ls_error_t ls_write_helper(ls_socket_t* socket, const char* buf, int len, bool eor)
{
    return LSE_CLOSED;
}

ls_error_t ls_write(int socket, const char* buf, int len)
{
    return ls_write_helper(validate_socket(socket), buf, len, false);
}

/*
 * Same as ls_write, but delivers an 'end of record' mark at end of data.
 * End of record write does nothing special for datagram sockets.
 */
ls_error_t ls_write_eor(int socket, const char* buf, int len)
{
    return ls_write_helper(validate_socket(socket), buf, len, true);
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
 *      socket              Socket containing data
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
 */
ls_error_t ls_read_with_address(int socket, char* buf, int maxlen, int* address, int* port)
{
    ls_error_t ret = LSE_NO_ERROR;

    ls_socket_t* s = validate_socket(socket);

    if (s == NULL) {
        ret = LSE_INVALID_SOCKET;
    } else if (s->socket_type == LS_DATAGRAM) {
        /* Pend on a packet in the queue */
        packet_t* packet;
        if (os_get_queue(s->received_packets, (os_queue_item_t*) &packet)) {
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
            ret = packet_data_length;
            release_packet(packet);
        }
    } else if (s->socket_type == LS_STREAM) {
        /* STREAM packets arrive on the socket stream_packet_queue after being sorted and acked as necessary */
        packet_t* packet;
        bool eor = false;
        int total_len = 0;

        /* Go until length satisfied or end of record */
        while (!eor && maxlen != 0) {
            int offset = 0;

            if (s->residue_packet != NULL) {
                packet = s->residue_packet;
                s->residue_packet = NULL;
                offset = s->residue_offset;
                s->residue_offset = 0;
            } else if (os_get_queue(s->stream_packet_queue, (os_queue_item_t*) &packet)) {
                offset = 0;
            } else {
                packet = NULL;
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
                    s->residue_packet = packet;
                    s->residue_offset = offset;
                }
            } else {
                eor = true;
            }
        }

        /* If residue bytes in the last packet and it was NOT an eor, remember the packet and the offset */

    } else {
        ret = LSE_INVALID_SOCKET;
    }

    if (ret != LSE_INVALID_SOCKET) {
        s->last_error = ret;
    }

    return ret;
}

ls_error_t ls_read(int socket, char* buf, int maxlen)
{
    return ls_read_with_address(socket, buf, maxlen, NULL, NULL);
}


/*
 * Close a socket.  All internal information is deleted.
 * Returns status code.  0 is success.
 */
static ls_error_t ls_close_helper(int socket, bool immediate)
{
    ls_error_t ret = LSE_NO_ERROR;

    ls_socket_t* s = validate_socket(socket);
    if (s != NULL) {
        if (linklayer_lock()) {
            switch (s->socket_type) {
                case LS_UNUSED: {
                    ret = LSE_NOT_OPENED;
                    break;
                }

                case LS_STREAM: {
                    if (s->listen) {
                        /* Close listener queue */
                        ls_socket_t* c;
                        while (os_get_queue_with_timeout(s->connections, (os_queue_item_t*) &c, 0)) {
                            if (c != NULL) {
                                ls_close_helper(c - socket_table, immediate);
                            }
                        }
                        os_delete_queue(s->connections);
                        s->connections = NULL;
                        s->socket_type = LS_UNUSED;
                    } else {
                        /* Clear asssemnly buffer */
                        /* Clear outbound queue */
                        /* Fall through and clear received packets */
                        if (!immediate) {
                            /* Start a close  */
                            start_state_machine(s, LS_STATE_DISCONNECTING,
                                                stream_packet_create_from_socket(s, STREAM_FLAGS_CMD_DISCONNECTED),
                                                STREAM_CONNECT_TIMEOUT, STREAM_CONNECT_RETRIES);
                     
                            ret = get_state_machine_response(s);
                        }
                    }
                    break;
                }

                case LS_DATAGRAM: {
                    /* Close packet receive queue */
                    linklayer_release_packets_in_queue(s->received_packets);
                    os_delete_queue(s->received_packets);
                    s->received_packets = NULL;
                    s->socket_type = LS_UNUSED;
                    break;
                }

                default: {
                    ret = LSE_INVALID_SOCKET;
                    break;
                }
            }
            linklayer_unlock();
        }
    } else {
        ret = LSE_INVALID_SOCKET;
    }

    return ret;
}

ls_error_t ls_close(int socket)
{
    return ls_close_helper(socket, false);
}

static ls_error_t ls_close_immediate(int socket)
{
    return ls_close_helper(socket, true);
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
ls_error_t ls_get_last_error(ls_socket_t* socket)
{
    ls_error_t error = socket->last_error;
    socket->last_error = LSE_NO_ERROR;

    return error;
}

static ls_socket_t* validate_socket(int socket)
{
    ls_socket_t* ret = NULL;

    if (socket >= 0 && socket < ELEMENTS_OF(socket_table)) {
       if (socket_table[socket].socket_type == LS_DATAGRAM || socket_table[socket].socket_type == LS_STREAM) {
           ret = &socket_table[socket];
       }
    }

    return ret;
}
#endif /* CONFIG_LASTLINKE_ENABLE_SOCKET_LAYER */
