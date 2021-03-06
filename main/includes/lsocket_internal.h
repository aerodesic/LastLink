/*
 * lsocket_internal.h
 *
 * Lightweight Socket layer.
 */
#ifndef __lsocket_internal_h_included
#define __lsocket_internal_h_included

#include <stdbool.h>

#include "sdkconfig.h"
#include "os_specific.h"
#include "lsocket.h"
#include "simpletimer.h"


#define MAX_PACKET_ASSEMBLY     CONFIG_LASTLINK_STREAM_MAX_PACKETS_IN_ASSEMBLY
#define MAX_SOCKET_CONNECTIONS  CONFIG_LASTLINK_STREAM_MAX_SIMULTANEOUS_CONNECTIONS

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS

#define STREAM_CONNECT_MAX_TIME   10000   /* 10 seconds */
#define STREAM_CONNECT_RETRIES    10

/*
 * How often we check the send_output_window results when closing.
 * The send_output_window is running on its own timer so this only
 * affects how often we *check* the results.
 */
#define STREAM_FLUSH_TIMEOUT     500    /* .5 seconds */
#define STREAM_FLUSH_RETRIES     120    /* 60 seconds worth */

#define STREAM_DISCONNECT_MAX_TIME 10000  /* 10 seconds */
#define STREAM_DISCONNECT_RETRIES  5

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

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

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS

/*
 * STREAMING data connection is based on datagram with sequence numbers.
 */
#define STREAM_FLAGS                   (DATAGRAM_PAYLOAD)
#define   STREAM_FLAGS_CMD                0x07
#define   STREAM_FLAGS_CMD_NOP            0x00   /* No operation (only look at FLAGS) */
#define   STREAM_FLAGS_CMD_DATA           0x01   /* Data for stream connection */
/*
 * Connect is:
 *      -> CONNECT                               // One side starts with connect
 *         CONNECT_ACK <-                        // Receiver responds with CONNECT ACK
 *      -> CONNECT_ACK                           // Originator responds back with CONNECT ACK
 *
 * The STREAM_SEQUENCE number on connect ack is used to convey the maximum number of in-queue
 * packets allowed by the sender.  When the packet is received, if it is non-zero, it will be
 * used to minimize the number (length) of the receive * window (hence setting the transmit
 * window length) of the sender.
 */
#define   STREAM_FLAGS_CMD_CONNECT        0x02   /* Connect request */
#define   STREAM_FLAGS_CMD_CONNECT_ACK    0x03   /* Ack to connect request or ack to ACK */
/*
 * Disconnect is:
 *     -> DISCONNECT                   // One side send DISCONNECT
 *        DISCONNECTED <-              // Other side responds with DISCONNECTED
 *     -> DISCONNECTED                 // Originator responds back with DISCONNECTED
 */
#define   STREAM_FLAGS_CMD_DISCONNECT     0x04   /* Disconnect request */
#define   STREAM_FLAGS_CMD_DISCONNECTED   0x05   /* Disconect ack */
/*
 * REJECT is sent when something went wrong.
 */
#define   STREAM_FLAGS_CMD_REJECT         0x06   /* Reject - something's wrong */
// #define  STREAM_FLAGS_UNUSED_7         0x07
#define STREAM_FLAGS_CMD_COUNT            (STREAM_FLAGS_CMD+1)
#define   STREAM_FLAGS_BITS               0xF8
#define   STREAM_FLAGS_ACKNUM             0x80   /* ACK Sequence number is present */
#define   STREAM_FLAGS_EOR                0x40   /* End of record flag */
// #define  STREAM_FLAGS_UNUSED           0x38   /* Unused flags */
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

/* Set to add debugging logic to socket and global socket locking */

#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */

typedef enum {
    LS_STATE_IDLE = 0,                           /* Idle - not connected */
    LS_STATE_INUSE,                              /* In use but not assigned */
    LS_STATE_SOCKET,                             /* SOCKET has been instantiated */
    LS_STATE_BOUND,                              /* SOCKET has been bound to an address / port */
    LS_STATE_CONNECTED,                          /* Connected - bidirectional streams ready */
#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
    LS_STATE_LISTENING,                          /* SOCKET is in listen mode */
    LS_STATE_INBOUND_CONNECT,                    /* Inbound connect on listen socket received */
    LS_STATE_OUTBOUND_CONNECT,                   /* Outbound connect sent */
    LS_STATE_DISCONNECTING_FLUSH_START,          /* Disconnecting but flush output first */
    LS_STATE_DISCONNECTING_FLUSHING,             /* Disconnecting but flush output first - in progress */
    LS_STATE_INBOUND_DISCONNECTING,              /* Tearing down connection from external request */
    LS_STATE_OUTBOUND_DISCONNECTING,             /* Tearing down connection from internal request */
    LS_STATE_LINGER,                             /* Lingering at end of connection */
    LS_STATE_DISCONNECTED,                       /* Disconnected */
#endif  /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
    LS_STATE_CLOSING,                            /* When complete idle but still in use */
    LS_STATE_CLOSED,                             /* When complete idle but still in use */
} ls_socket_state_t;


typedef struct ls_socket ls_socket_t;
typedef struct packet_window packet_window_t;    /* Forward declaration for external structure */

typedef struct ls_socket {
    os_mutex_t              lock;                /* MUTEX for user level access control - does not block I/O */
#ifdef CONFIG_LASTLINK_SOCKET_LOCKING_DEBUG
    const char              *last_lock_file;
    int                     last_lock_line;
    int                     lock_count;
#endif /* CONFIG_LASTLINK_SOCKET_LOCKING_DEBUG */
    int                     busy;                /* Busy when non-zero */
    ls_socket_type_t        socket_type;         /* Socket type (DATAGRAM or STREAM) */

    ls_socket_state_t       state;               /* Current state */

    /* src_addr is implied as this node's address */
    ls_port_t               src_port;            /* Local port number of the connection */
    ls_address_t            dest_addr;           /* Destination address of the connection */
    ls_port_t               dest_port;           /* Destination port of the connection */

    union {
        /* LISTEN SOCKET INFO */
        struct {
            os_queue_t   connections;           /* New connections arrive here */
        };

        /* DATAGRAM SOCKET INFO */
        struct {
            os_queue_t   datagram_packets;     /* Raw datagrams */
        };

#if CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS
        /* STREAM SOCKET INFO */
        struct {
           /* Filtered and ordered data packets show up here */
           ls_socket_t             *parent;      /* The listening socket number that begat us (if any) */

           /* state machine retry stuff */
           simpletimer_t           state_machine_timer;
           int                     state_machine_retries;
           int                     (*state_machine_action_callback)(ls_socket_t *socket);
#define STATE_MACHINE_ACTION_NONE      (-1)          /* No action required on return */
#define STATE_MACHINE_ACTION_PAUSE     (-2)          /* Pause state machine for delayed action */
#define STATE_MACHINE_ACTION_SUCCESS   (-3)          /* Finished with success */
#define STATE_MACHINE_ACTION_RETRY     (-4)          /* Retry with another timeout */
/* Negative numbers out of this range are return error codes */
           void                    (*state_machine_results_callback)(ls_socket_t *socket, ls_error_t error);
#define STATE_MACHINE_RESULTS_TO_QUEUE  NULL
           os_queue_t              state_machine_results_queue;
           bool                    state_machine_running;  /* Mostly for debug */
           const char              *state_machine_ident;

           /* Deals with residue of left over data on packets between read calls */
           packet_t*               current_read_packet;
           int                     current_read_offset;

           /* Packet assembly buffers */
           packet_window_t         *input_window;
           simpletimer_t           output_window_timer;
           simpletimer_t           input_window_timer;
           simpletimer_t           ack_delay_timer;
           packet_window_t         *output_window;
           int                     output_retries;
           int                     output_retry_time;
           int                     output_last_sequence;       /* Last sequence number block transmitted */
           bool                    output_disconnect_on_error;
#if CONFIG_LASTLINK_STREAM_KEEP_ALIVE_ENABLE
           simpletimer_t           keepalive_timer;
#endif

           simpletimer_t           socket_flush_timer;
           packet_t                *current_write_packet;
           bool                    end_of_data;

           /* Outbound packet cache to avoid duplication.  Packets are released from here after transmission,
            * otherwise they are reused inplace when updates are needed.
            */
           packet_t                *packet_cache[STREAM_FLAGS_CMD_COUNT];
        };
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_STREAMS */
    };

    ls_error_t              last_error;         /* Error code if something goes wrong. */
} ls_socket_t;

/* Convert socket number to socket structure */
ls_socket_t *validate_socket(int s);

bool lock_socket(ls_socket_t* socket);
void unlock_socket(ls_socket_t* socket);

#endif /* __lsocket_internal_h_included */

