/*
 * lsocket.h
 *
 * Lightweight Socket layer.
 */
#ifndef __lsocket_h_included
#define __lsocket_h_included

#include <stdbool.h>

#include "sdkconfig.h"
#include "os_freertos.h"

#define MAX_PACKET_ASSEMBLY     CONFIG_LASTLINK_STREAM_MAX_PACKETS_IN_ASSEMBLY
#define MAX_SOCKET_CONNECTIONS  CONFIG_LASTLINK_STREAM_MAX_SIMULTANEOUS_CONNECTIONS

#include "packets.h"

typedef int ls_port_t;
typedef int ls_address_t;
typedef int ls_error_t;

typedef enum {
    LS_NO_ERROR         = 0,
    LS_CLOSED           = -1,
    LS_xxx              = -2,
} ls_errors_t;
    
typedef enum {
    LS_DATAGRAM = 0,
    LS_STREAM,
} ls_socket_type_t;


/* Forward ref */
typedef struct ls_socket ls_socket_t;

/*
 * A structure used to assemble packets arriving potentially out of order.
 *
 * Starts empty (first_sequence and num_in_buffer are 0)
 *
 * When a packet is received and matches next_sequence, it is sent to the user.
 * When a packet is received that is > next_sequence + MAX_ASSEMBLY_BUFFER, it is discarded.
 * Otherwise, the packet is placed buffer[<packet_sequence> - next_sequence].
 * When a packet is received at next_sequence, all packets from next_sequence up to the
 * next NULL or end of packets[] are send to the user and any packets following this
 * are moved up and next_sequence is set to the first empty slot in the buffer.
 * "Delivery" means putting the packet into the receive_queue.
 *
 * If a receive failure occurs, an error is placed in the socket structure and
 * NULL packet is delivered to the receive queue.  When the receivers reads the
 * NULL packet, it will substitute the error_code for the return value to the
 * read.
 */
typedef struct packet_assembly_buffer {
    int               next_sequence;
    int               num_in_buffer;
    ls_socket_t*      packets[MAX_PACKET_ASSEMBLY];
} PacketAssemblyBuffer_t;

typedef struct ls_socket {
    ls_port_t               localport;          /* Local port number of the connection */
    ls_port_t               destport;           /* Destination port of the connection */
    ls_address_t            localaddress;       /* Load address of the connection */
    ls_address_t            destaddress;        /* Destination address of the connection */
    ls_socket_type_t        socket_type;        /* Socket type (DATAGRAM or STREAM) */

    os_queue_t              transmit_results;   /* Receives result codes from transmissions */
    os_queue_t              receive_queue;      /* Receives packets from the destination socket */
    os_queue_t              connection_queue;   /* Receives notifications of new connections */
    PacketAssemblyBuffer_t  transmit_queue;     /* Assembly buffer for inbound packets */

    int                     max_queue;          /* Max in queue */
    bool                    listen;             /* True when listening */
    ls_error_t              last_error;         /* Error code if something goes wrong. */
} ls_socket_t;

/*
 * create a socket for sending or listening (determined by ls_connect or ls_listen)
 * return ls_socket_t*.
 */
ls_socket_t* ls_socket(ls_socket_type_t socket_type);

/*
 * Bind an port to the local port.
 */
ls_error_t ls_bind(ls_socket_t* socket, ls_port_t local_port);

/*
 * Listen for connections on this socket.  Returns a socket structure
 * when one is detected.
 */
ls_socket_t* ls_listen(ls_socket_t* socket, int max_queue, int timeout);

/*
 * Connect to a remote port.
 */
ls_error_t ls_connect(ls_socket_t* socket, ls_address_t address, ls_port_t port);

/*
 * Write to a datagram or stream socket.
 *
 * Datagrams are limited to one packet (MAX_DATAGRAM_SIZE) bytes.
 * Streams are limited by user and memory for buffers.
 *
 * Returns number of bytes written otherwise an error code (<0 values)
 */
ls_error_t ls_write(ls_socket_t* socket, const char* buf, int len);

/*
 * Same as ls_write, but delivers an 'end of record' mark at end of data.
 * End of record write does nothing special for datagram sockets.
 */
ls_error_t ls_write_eor(ls_socket_t* socket, const char* buf, int len);

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
 */
ls_error_t ls_read(ls_socket_t* socket, char* buf, int maxlen);

/*
 * Close a socket.  All internal information is deleted.
 * Returns status code.  0 is success.
 */
ls_error_t ls_close(ls_socket_t* socket);

/*
 * Get last error code.
 */
ls_error_t ls_get_last_error(ls_socket_t* socket);

#endif /* __lsocket_h_include */
