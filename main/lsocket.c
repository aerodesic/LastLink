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
 *      ls_connect(socket, <target address>, <target port>)
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
 *      int rc = ls_connect(socket, <target address>, <target port>)
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

#include "os_freertos.h"
#include "lsocket.h"
#include "packets.h"
#include "linklayer.h"

static ls_error_t ls_write_helper(ls_socket_t* socket, const char* buf, int len, bool eor);
static bool validate_socket(ls_socket_t* socket);

/*
 * Initialize ls_socket layer.
 */
ls_error_t ls_socket_init(void)
{
    return -1;
}

ls_error_t ls_socket_deinit(void)
{
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

    /* Validate parameters */
    if (socket_type == LS_DATAGRAM || socket_type == LS_STREAM) {

        socket = (ls_socket_t*) malloc(sizeof(ls_socket_t));

        if (socket != NULL) {
             memset(socket, 0, sizeof(ls_socket_t));
             socket->socket_type = socket_type;
             if (socket_type == LS_STREAM) {
                 /* Create queues for stream */
                 socket->transmit_results = os_create_queue(0,0);
             }
             socket->receive_queue = os_create_queue(0,0);
        }
    }

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
        socket->localaddress = linklayer_get_node_address();
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
        if (os_get_queue_with_timeout(socket->connection_queue, (os_queue_item_t*) &connection, timeout)) {
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
 *      ls_error_t          LS_NO_ERROR (0) if successful otherwise error code.
 */
ls_error_t ls_connect(ls_socket_t* socket, ls_address_t address, ls_port_t port)
{
    return LS_CLOSED;
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
    return LS_CLOSED;
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
 */
ls_error_t ls_read(ls_socket_t* socket, char* buf, int maxlen)
{
    return LS_CLOSED;
}

/*
 * Close a socket.  All internal information is deleted.
 * Returns status code.  0 is success.
 */
ls_error_t ls_close(ls_socket_t* socket)
{
    return LS_NO_ERROR;
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
    socket->last_error = LS_NO_ERROR;

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
