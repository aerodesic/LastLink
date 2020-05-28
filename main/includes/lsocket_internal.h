/*
 * lsocket.h
 *
 * Lightweight Socket layer.
 */
#ifndef __lsocket_internal_h_included
#define __lsocket_internal_h_included

#include <stdbool.h>

#include "sdkconfig.h"
#include "os_freertos.h"
#include "lsocket.h"
#include "simpletimer.h"


/*
 * Pings are always directed to a target node, but routed as all data packets.
 */
#define PING_SEQUENCE_NUMBER           (DATA_PAYLOAD)
#define PING_ROUTE_TABLE               (PING_SEQUENCE_NUMBER + SEQUENCE_NUMBER_LEN)
#define PING_LEN                       (PING_ROUTE_TABLE - HEADER_LEN)             /* Length with a no addresses in route table */

#define PING_PROTOCOL                  (FIRST_DATA_PROTOCOL+0)

#define PINGREPLY_PROTOCOL             (FIRST_DATA_PROTOCOL+1)

#define DATAGRAM_DEST_PORT             (DATA_PAYLOAD)
#define DATAGRAM_SRC_PORT              (DATAGRAM_DEST_PORT + PORT_NUMBER_LEN)
#define DATAGRAM_HEADER_END            (DATAGRAM_SRC_PORT + PORT_NUMBER_LEN)
#define DATAGRAM_PAYLOAD               (DATAGRAM_HEADER_END)
#define DATAGRAM_LEN                   (DATAGRAM_PAYLOAD - HEADER_LEN)
#define DATAGRAM_MAX_DATA              (MAX_PACKET_LEN - DATAGRAM_PAYLOAD)

#define DATAGRAM_PROTOCOL              (FIRST_DATA_PROTOCOL+2)

/*
 * STREAMING data connection is based on datagram with sequence numbers.
 */
#define STREAM_FLAGS                   (DATAGRAM_PAYLOAD)
#define   STREAM_FLAGS_CMD                0x0F
#define   STREAM_FLAGS_CMD_NOP            0x00   /* No operation */
#define   STREAM_FLAGS_CMD_DATA           0x01   /* Data for socket */
#define   STREAM_FLAGS_CMD_DATA_EOR       0x02   /* Data for socket with end of record */
#define   STREAM_FLAGS_CMD_CONNECT        0x03   /* Connect request */
#define   STREAM_FLAGS_CMD_CONNECT_ACK    0x04   /* Ack to connect request or ack to ACK */
#define   STREAM_FLAGS_CMD_DISCONNECT     0x05   /* Disconnect request */
#define   STREAM_FLAGS_CMD_DISCONNECTED   0x06   /* Disconect ack */
#define   STREAM_FLAGS_CMD_REJECT         0x07   /* Reject - something's wrong */
// #define  STREAM_FLAGS_UNUSED_8         0x08
// #define  STREAM_FLAGS_UNUSED_9         0x09
// #define  STREAM_FLAGS_UNUSED_A         0x0A
// #define  STREAM_FLAGS_UNUSED_B         0x0B
// #define  STREAM_FLAGS_UNUSED_C         0x0C
// #define  STREAM_FLAGS_UNUSED_D         0x0D
// #define  STREAM_FLAGS_UNUSED_E         0x0E
// #define  STREAM_FLAGS_UNUSED_F         0x0F
#define   STREAM_FLAGS_ACKNUM             0x10   /* ACK Sequence number is present */
// #define  STREAM_FLAGS_UNUSED           0xE0   /* Unused flags */
#define STREAM_SEQUENCE                (STREAM_FLAGS + FLAGS_LEN)
#define STREAM_ACK_SEQUENCE            (STREAM_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define STREAM_ACK_WINDOW              (STREAM_ACK_SEQUENCE + SEQUENCE_NUMBER_LEN)
// Allocate number of bytes for ACK window
#define ACK_WINDOW_LEN                 ((CONFIG_LASTLINK_STREAM_WINDOW_SIZE + 7)/8)
#define STREAM_HEADER_END              (STREAM_ACK_WINDOW + ACK_WINDOW_LEN)
#define STREAM_PAYLOAD                 (STREAM_HEADER_END)
#define STREAM_LEN                     (STREAM_PAYLOAD - HEADER_LEN)
#define STREAM_MAX_DATA                (MAX_PACKET_LEN - STREAM_PAYLOAD)
#define STREAM_PROTOCOL                (FIRST_DATA_PROTOCOL+3)

typedef struct packet_window {
    os_mutex_t       lock;                       /* For exclusive access */
    int              retry_time;                 /* Next time for retry delay */
    int              retries;                    /* Count of remaining tries */
    uint8_t          length;                     /* Number of slots */
    uint8_t          released;                   /* Number of packets freed that have not issued release_semaphore */
    int              next_in;                    /* Next input slot to use */
    int              sequence;                   /* Sequence number of next packet to be added to window */
    os_semaphore_t   available;                  /* Semaphore used to release access to packets */
    packet_t         *slots[1];                  /* 1..length slots */
} packet_window_t;

typedef enum {
    LS_STATE_IDLE = 0,
    LS_STATE_INBOUND_CONNECT,
    LS_STATE_OUTBOUND_CONNECT,
    LS_STATE_CONNECTED,
    LS_STATE_DISCONNECTING_FLUSH,
    LS_STATE_DISCONNECTING,
    LS_STATE_DISCONNECTED,
} ls_socket_state_t;

typedef struct ls_socket ls_socket_t;
typedef struct packet_window packet_window_t;

typedef struct ls_socket {
    os_mutex_t              lock;               /* MUTEX for user level access control - does not block I/O */
    bool                    inuse;              /* TRUE if socket is opened by user */
    bool                    busy;               /* Set true when inside user code in ls_xxx function */
    ls_socket_type_t        socket_type;        /* Socket type (DATAGRAM or STREAM) */

    ls_socket_state_t       state;              /* Current state */
    ls_port_t               local_port;         /* Local port number of the connection */
    ls_port_t               dest_port;          /* Destination port of the connection */
    bool                    rename_dest;        /* If datagram and true, dest_address gets value of last packet read */
                                                /* This allows us to ls_write() back to return data */
    ls_address_t            dest_addr;          /* Destination address of the connection */

    simpletimer_t           linger_timer;       /* Performs close after no more users */

    union {
        /* LISTEN SOCKET INFO */
        struct {
            os_queue_t   connections;           /* New connections arrive here */
        };

        /* DATAGRAM SOCKET INFO */
        struct {
            os_queue_t   datagram_packets;     /* Raw datagrams */
        };

        /* STREAM SOCKET INFO */
        struct {
           /* Filtered and ordered data packets show up here */
           ls_socket_t             *parent;      /* The listening socket number that begat us (if any) */

           /* state machine retry stuff */
           simpletimer_t           state_machine_timer;
           int                     state_machine_retries;
           bool                    (*state_machine_action)(ls_socket_t *socket);
           os_queue_t              state_machine_response;

           /* Deals with residue of left over data on packets between read calls */
           packet_t*               current_read_packet;
           int                     current_read_offset;

           /* Packet assembly buffers */
           packet_window_t         *input_window;
           simpletimer_t           output_window_timer;
           packet_window_t         *output_window;
           simpletimer_t           socket_flush_timer;
           packet_t                *current_write_packet;
        };
    };

    bool                    listen;             /* True when listening */
    bool                    record_mark_seen;   /* Detects two record marks in a row to signal socket closed */
    ls_error_t              last_error;         /* Error code if something goes wrong. */
} ls_socket_t;

#endif /* __lsocket_internal_h_included */

