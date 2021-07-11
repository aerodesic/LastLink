/*
 * lsocket.h
 *
 * Lightweight Socket layer.
 */
#ifndef __lsocket_h_included
#define __lsocket_h_included

#include <stdbool.h>
#include <sys/stat.h>

#include "sdkconfig.h"
#include "os_specific.h"
#include "packets.h"

typedef int ls_port_t;
typedef int ls_address_t;
typedef int ls_error_t;

typedef enum {
    LSE_NO_ERROR         = 0,
    LSE_CLOSED           = -1000,
    LSE_NOT_OPENED       = -1001,
    LSE_CANNOT_REGISTER  = -1002,
    LSE_INVALID_SOCKET   = -1003,
    LSE_NO_MEM           = -1004,
    LSE_TIMEOUT          = -1005,
    LSE_FAILED           = -1006,
    LSE_NO_ROUTE         = -1007,
    LSE_BAD_TYPE         = -1008,
    LSE_SYSTEM_ERROR     = -1009,
    LSE_INVALID_MAXQUEUE = -1010,
    LSE_CONNECT_FAILED   = -1011,
    LSE_CONNECT_REJECTED = -1012,
    LSE_NOT_WRITABLE     = -1013,
    LSE_SOCKET_BUSY      = -1014,
    LSE_DISCONNECTING    = -1015,    /* Is disconneting */
    LSE_DISCONNECTED     = -1016,
    LSE_NOT_CONNECTED    = -1017,    /* No connect() issued */
    LSE_NOT_BOUND        = -1018,    /* No bind() issued */

    LSE_NOT_IMPLEMENTED  = -1999,
} ls_errors_t;

typedef enum {
    LS_UNUSED = 0,    /* Not currently in use */
    LS_INUSE,         /* In use but not assigned a type yet */
    LS_DATAGRAM,      /* Receiving/transmitting datagrams */
    LS_STREAM,        /* Receiving/transmitting streams */
} ls_socket_type_t;


ls_error_t ls_socket_init(void);
ls_error_t ls_socket_deinit(void);

/*
 * create a socket for sending or listening (determined by ls_connect or ls_listen)
 */
ls_error_t ls_socket(ls_socket_type_t socket_type);

/*
 * Bind an port to the local port.
 */
ls_error_t ls_bind(int socket, ls_port_t local_port);

/*
 * Listen for connections on this socket.  Returns a socket structure
 * when one is detected.
 */
ls_error_t ls_listen(int socket, int max_queue, int timeout);

/*
 * Connect to a remote port.
 */
ls_error_t ls_connect(int socket, ls_address_t address, ls_port_t port);

/*
 * Write to a datagram or stream socket.
 *
 * Datagrams are limited to one packet (MAX_DATAGRAM_SIZE) bytes.
 * Streams are limited by user and memory for buffers.
 *
 * Returns number of bytes written otherwise an error code (<0 values)
 */
ls_error_t ls_write(int s, const void* buf, size_t len);

/*
 * Write to a specific address (datagram only)
 */
ls_error_t ls_write_to(int s, const void* buf, size_t len, ls_address_t address, ls_port_t port);

/*
 * Same as ls_write, but delivers an 'end of record' mark at end of data.
 * End of record write does nothing special for datagram sockets.
 */
ls_error_t ls_write_eor(int s, const void* buf, size_t len);

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
ls_error_t ls_read(int s, void* buf, size_t maxlen);

ls_error_t ls_read_with_timeout(int s, void* buf, size_t maxlen, int timeout);

/* Add params to return address, port and provide timeout. */
ls_error_t ls_read_with_address(int s, char* buf, size_t maxlen, int* address, int* port, int timeout);

/*
 * Close a socket.  All internal information is deleted.
 * Returns status code.  0 is success.
 */
ls_error_t ls_close(int s);

/*
 * ls_ioctl
 */
int ls_ioctl(int s, int cmd, ...);

/*
 * ls_fcntl
 */
int ls_fcntl(int s, int cmd, int arg);

/*
 * ls_fstat
 */
int ls_fstat(int fd, struct stat *st);

/*
 * ls_select
 */
int ls_select(int maxdfp1, fd_set *readset, fd_set *writeset, fd_set *exceptset, struct timeval *timeout);

/*
 * Get local port of socket
 */

ls_error_t ls_get_local_port(int s);

/*
 * Get last error code.
 */
ls_error_t ls_get_last_error(int socket);

/*
 * Ping a node and return route list.
 */
ls_error_t ping(int address, uint32_t *elapsed, int* routelist, int routelistlen);

/*
 * debug
 */
ls_error_t ls_dump_socket(const char* msg, int socket);

#endif /* __lsocket_h_include */
