/*
 * lsocket.c
 *
 * Lightweight Socket layer.
 *
 * Example DATAGRAM:
 *      // Create socket
 *      ls_socket_t* socket = ls_socket(LS_DATAGRAM);
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
 *      ls_socket_t* socket = ls_socket(LS_STREAM);
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
 *      ls_socket_t* listen_socket = ls_socket(LS_STREAM);
 *      // Bind port
 *      ls_bind(socket, 1234)
 *      ls_socket_t* new_connection = ls_listen(socket, 5, 0);
 *         ( when connection arrives, ls_listen returns with allocated socket )
 *      if (new_connection != NULL) {
 *         int len = ls_read(socket, buf, buflen);
 *              -or-
 *          int len = ls_write(socket, buf, buflen);
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
#include "os_freertos.h"
#include "lsocket_internal.h"
#include "packets.h"
#include "linklayer.h"

/* PING support */
static const char* ping_packet_format(const packet_t* p);
static const char* pingreply_packet_format(const packet_t* p);
static bool ping_packet_process(packet_t* p);
static bool pingreply_packet_process(packet_t* p);
static int find_ping_table_entry(int sequence);

static ls_error_t ls_write_helper(ls_socket_t* socket, const char* buf, int len, bool eor);
static bool validate_socket(ls_socket_t* socket);
static bool stream_packet_process(packet_t* packet);
static const char* stream_packet_format(const packet_t* packet);
static bool datagram_packet_process(packet_t* packet);
static const char* datagram_packet_format(const packet_t* packet);


static ls_socket_t* find_socket_from_packet(const packet_t* packet, ls_socket_type_t type);

typedef struct ping_table_entry {
    os_queue_t   queue;
    int          sequence;
} ping_table_entry_t;

ping_table_entry_t   ping_table[CONFIG_LASTLINK_MAX_OUTSTANDING_PINGS];

static ls_socket_t  sockets[CONFIG_LASTLINK_NUMBER_OF_SOCKETS];

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


/* Look in socket table for socket matching the connection and type */
static ls_socket_t* find_socket_from_packet(const packet_t* packet, ls_socket_type_t type)
{
    ls_socket_t* socket = NULL;

    ls_port_t destport = get_uint_field(packet, DATAGRAM_DESTPORT, PORT_NUM_LEN);
    ls_port_t srcport  = get_uint_field(packet, DATAGRAM_SRCPORT, PORT_NUM_LEN);
    int destaddr       = get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN);

    for (int index = 0; socket == NULL && index < CONFIG_LASTLINK_NUMBER_OF_SOCKETS; ++index) {
        if (sockets[index].socket_type == type && sockets[index].destaddr == destaddr &&
            sockets[index].localport == destport && sockets[index].destport == srcport) {

            socket = &sockets[index];
        }
    }

    /* Return socket if we found it */
    return socket;
}


static bool datagram_packet_process(packet_t* packet)
{
    bool processed = false;

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

    return processed;
}

static const char* datagram_packet_format(const packet_t* packet)
{
    char* info;

    const char* data = linklayer_escape_raw_data(packet->buffer + DATAGRAM_PAYLOAD, packet->length - DATAGRAM_PAYLOAD);

    asprintf(&info, "Datagram: Src Port %d Dest Port %d \"%s\"",
            get_uint_field(packet, DATAGRAM_DESTPORT, PORT_NUM_LEN),
            get_uint_field(packet, DATAGRAM_SRCPORT, PORT_NUM_LEN),
            data);

     free((void*) data);

     return info;
}

static bool stream_packet_process(packet_t* packet)
{
    bool processed = false;

    ls_socket_t* socket = find_socket_from_packet(packet, LS_STREAM);

    if (socket != NULL) {
        /* Place packets into assembly buffer by sequence number.
         * Packets that are consecutive are placed into the receive buffer and
         * removed from the assembly buffer.
         *
         * The user 'read' process removes packets from the receive queue
         * and returns the data to the user.  record marks stop a read
         * with two record marks indicating end of data (packet closed)
         */

        processed = true;
    }

    return processed;
}

static const char* stream_packet_format(const packet_t* packet)
{
    char* info;

    const char* data = linklayer_escape_raw_data(packet->buffer + STREAM_PAYLOAD, packet->length - STREAM_PAYLOAD);

    asprintf(&info, "Stream: Src Port %d Dest Port %d Ack %d Seq %d \"%s\"",
            get_uint_field(packet, DATAGRAM_DESTPORT, PORT_NUM_LEN),
            get_uint_field(packet, DATAGRAM_SRCPORT, PORT_NUM_LEN),
            get_uint_field(packet, STREAM_ACK_SEQUENCE, SEQUENCE_NUMBER_LEN),
            get_uint_field(packet, STREAM_SEQUENCE, SEQUENCE_NUMBER_LEN),
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
    /* Purge all waiting pings */
    /* Purge all sockets */

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
      
    return err;
    return -1;
}

/*
 * Create a socket for sending or listening (determined by ls_connect or ls_listen)
 *
 * Entry:
 *      socket_type         LS_DATAGRAM or LS_STREAM
 *
 * Returns:
 *      ls_socket_t*        If successful, otherwise NULL (Out of memory probably)
 */
ls_socket_t* ls_socket(ls_socket_type_t socket_type)
{
    ls_socket_t* socket = NULL;

#ifdef NOTUSED
    /* Validate parameters */
    if (socket_type == LS_DATAGRAM || socket_type == LS_STREAM) {

        socket = (ls_socket_t*) malloc(sizeof(ls_socket_t));

        if (socket != NULL) {
             memset(socket, 0, sizeof(ls_socket_t));
             socket->input_queue = NULL;
        }
    }
#endif

    return socket;
}

/*
 * ls_bind
 *
 * Bind an port to the local port.
 */
ls_error_t ls_bind(ls_socket_t* socket, ls_port_t local_port)
{
    bool ok;

    if (validate_socket(socket)) {
        socket->localport = local_port;
        ok = true;
    }

    return ok;
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
 *      NULL                If timeout or error
 *      ls_socket_t*        If a connection arrives
 *
 */
ls_socket_t* ls_listen(ls_socket_t* socket, int max_queue, int timeout)
{
    ls_socket_t* new_connection = NULL;

    /* Validate paramters */
    if (validate_socket(socket) && socket->socket_type == LS_STREAM && max_queue > 0 && max_queue < MAX_SOCKET_CONNECTIONS) {

        /* Only valid for STREAMS */
        /* Place socket in listen mode */
        socket->listen = true;
        socket->max_queue = max_queue;

        /* Wait for something to arrive at connection queue */
        packet_t* connection;
        if (os_get_queue_with_timeout(socket->connections, (os_queue_item_t*) &connection, timeout)) {
            /* A new connection packet */
        }
    }

    return new_connection;
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
ls_error_t ls_connect(ls_socket_t* socket, ls_address_t address, ls_port_t port)
{
    return LSE_CLOSED;
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

ls_error_t ls_write(ls_socket_t* socket, const char* buf, int len)
{
    return ls_write_helper(socket, buf, len, false);
}

/*
 * Same as ls_write, but delivers an 'end of record' mark at end of data.
 * End of record write does nothing special for datagram sockets.
 */
ls_error_t ls_write_eor(ls_socket_t* socket, const char* buf, int len)
{
    return ls_write_helper(socket, buf, len, true);
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
 */
ls_error_t ls_read(ls_socket_t* socket, char* buf, int maxlen)
{
    ls_error_t ret;

    if (socket->socket_type == LS_DATAGRAM) {

    } else if (socket->socket_type == LS_STREAM) {
        /* Pend on a packet in the queue */
        packet_t* packet;
        if (os_get_queue(socket->received_packets, (os_queue_item_t*) &packet)) {
        }

    } else {
        ret = LSE_INVALID_SOCKET;
    }

    return ret;
}

/*
 * Close a socket.  All internal information is deleted.
 * Returns status code.  0 is success.
 */
ls_error_t ls_close(ls_socket_t* socket)
{
    return LSE_NO_ERROR;
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

static bool validate_socket(ls_socket_t* socket)
{
    bool ok = false;

    if (socket != NULL && (socket->socket_type == LS_DATAGRAM || socket->socket_type == LS_STREAM)) {
        ok = true;
    }

    return ok;
}
#endif /* CONFIG_LASTLINKE_ENABLE_SOCKET_LAYER */
