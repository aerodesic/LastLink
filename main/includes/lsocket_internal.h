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


/*
 * Pings are always directed to a target node, but routed as all data packets.
 */
#define PING_SEQUENCE                  (HEADER_LEN + 0)
#define PING_ROUTE_TABLE               (PING_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define PING_LEN                       (PING_ROUTE_TABLE + ADDRESS_LEN - HEADER_LEN)             /* Length with a single ADDRESS in route table */

#define PING_PROTOCOL                  (FIRST_DATA_PROTOCOL+0)

/* When ping packet is returned, it is simply renamed with PING_REPLY and the return hops are recorded */
#define PINGREPLY_PROTOCOL             (FIRST_DATA_PROTOCOL+1)

#define DATAGRAM_DEST_PORT             (DATA_PAYLOAD + 0)
#define DATAGRAM_SRC_PORT              (DATAGRAM_DEST_PORT + PORT_NUM_LEN)
#define DATAGRAM_HEADER_END            (DATAGRAM_SRC_PORT + PORT_NUM_LEN)
#define DATAGRAM_PAYLOAD               (DATAGRAM_HEADER_END)
#define DATAGRAM_LEN                   (MAX_PACKET_LEN - DATAGRAM_PAYLOAD)

#define DATAGRAM_PROTOCOL              (FIRST_DATA_PROTOCOL+2)

/*
 * STREAMING data connection is based on datagram with sequence numbers.
 */
#define STREAM_FLAGS                   (DATAGRAM_PAYLOAD)
#define   STREAM_FLAGS_CMD                0x07
#define   STREAM_FLAGS_CMD_DATA           0x00   /* Data for socket */
#define   STREAM_FLAGS_CMD_CONNECT        0x01   /* Connect request */
#define   STREAM_FLAGS_CMD_CONNECT_ACK    0x02   /* Ack to connect request or ack to ACK */
#define   STREAM_FLAGS_CMD_DISCONNECT     0x03   /* Disconnect request */
#define   STREAM_FLAGS_CMD_DISCONNECTED   0x04   /* Disconect ack */
#define   STREAM_FLAGS_CMD_REJECT         0x05   /* Reject - something's wrong */
// #define  STREAM_FLAGS_UNUSED_6         0x06
// #define  STREAM_FLAGS_UNUSED_7         0x07
#define   STREAM_FLAGS_EOR                0x10   /* End of record at end of packet */
// #define  STREAM_FLAGS_UNUSED           0xE0   /* Unused flags */
#define STREAM_SEQUENCE                (STREAM_FLAGS + FLAGS_LEN)
#define STREAM_ACK_SEQUENCE            (STREAM_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define STREAM_ACK_WINDOW              (STREAM_ACK_SEQUENCE + SEQUENCE_NUMBER_LEN)
// Allocate number of bytes for ACK window
#define ACK_WINDOW_LEN                 ((CONFIG_LASTLINK_STREAM_WINDOW_SIZE + 1)/8)
#define STREAM_HEADER_END              (STREAM_ACK_WINDOW + ACK_WINDOW_LEN)
#define STREAM_PAYLOAD                 (STREAM_HEADER_END)
#define STREAM_LEN                     (MAX_PACKET_LEN - STREAM_PAYLOAD)

#define STREAM_PROTOCOL                (FIRST_DATA_PROTOCOL+3)

typedef struct packet_window {
    uint8_t          length;                     /* Number of slots */
    uint8_t          in;                         /* Where the next sequential packet is placed. */
    uint8_t          released;                   /* Number of packets freed that have not issued release_semaphore */
    int              sequence;                   /* Sequence number of first packet (or expected packet) */
    os_semaphore_t   available;                  /* Semaphore for access */
    os_mutex_t       lock;                       /* For exclusive access */
    unsigned int     window;                     /* Window of used slots */
    packet_t         *slots[1];
} packet_window_t;

typedef enum {
    LS_STATE_IDLE = 0,
    LS_STATE_INBOUND_CONNECT,
    LS_STATE_OUTBOUND_CONNECT,
    LS_STATE_CONNECTED,
    LS_STATE_DISCONNECTING,
    LS_STATE_DISCONNECTED,
} ls_socket_state_t;

typedef struct ls_socket ls_socket_t;
typedef struct packet_window packet_window_t;

typedef struct ls_socket {
    ls_socket_type_t        socket_type;        /* Socket type (DATAGRAM or STREAM) */

    ls_socket_state_t       state;              /* Current state */
    ls_port_t               local_port;         /* Local port number of the connection */
    ls_port_t               dest_port;          /* Destination port of the connection */
    ls_address_t            dest_addr;          /* Destination address of the connection */
    int                     serial_number;      /* Unique serial number */


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
           ls_socket_t             *parent;      /* The listening socket number that begat us */

           /* state machine retry stuff */
           struct {
               int                     retries;
               os_timer_t              retry_timer;
               packet_t*               retry_packet;
               os_queue_t              response_queue;
           };

           /* Deals with residue of left over data on packets between read calls */
           packet_t*               residue_packet;
           int                     residue_offset;

           /* Packet assembly buffers */
           packet_window_t         *input_window;
           packet_window_t         *output_window;
           packet_t                *current_write_packet;
        };
    };

    bool                    listen;             /* True when listening */
    bool                    record_mark_seen;   /* Detects two record marks in a row to signal socket closed */
    ls_error_t              last_error;         /* Error code if something goes wrong. */
} ls_socket_t;

#endif /* __lsocket_internal_h_included */

