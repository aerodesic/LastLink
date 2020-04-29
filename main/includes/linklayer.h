/*
 * linklayer.h
 *
 * This module contains the top-level LastLink packetizer and distributor.
 *
 * It draws upon a lower-level packet serializer that can be one of several
 * devices (currently SX127x) interfaced via SPI.
 */
#ifndef __linklayer_h_included
#define __linklayer_h_included

#include "packets.h"
#include "radio.h"
#include "esp_vfs.h"

#ifdef CONFIG_LWIP_MAX_SOCKETS
#define LAST_LASTLINK_FD               ((MAX_FDS)-(CONFIG_LWIP_MAX_SOCKETS)-1)
#else
#define LAST_LASTLINK_FD               ((MAX_FDS)-1)
#endif

#define FIRST_LASTLINK_FD              (LAST_LASTLINK_FD - CONFIG_LASTLINK_NUMBER_OF_SOCKETS)

/* Default values for some entries */
#define ROUTE_LIFETIME                 CONFIG_LASTLINK_ROUTE_LIFETIME
#define MAX_ROUTES                     CONFIG_LASTLINK_MAX_ROUTES
#define NUM_PACKETS                    CONFIG_LASTLINK_NUM_PACKETS
#define ANNOUNCE_INTERVAL_DEFAULT      CONFIG_LASTLINK_ANNOUNCE_INTERVAL
#define ROUTEREQUEST_TIMEOUT           CONFIG_LASTLINK_ROUTEREQUEST_TIMEOUT
#define ROUTEREQUEST_RETRIES           CONFIG_LASTLINK_ROUTEREQUEST_RETRIES

#define UNDEFINED_SEQUENCE_NUMBER      0
#define NO_HOST_FLAGS                  0

#define PROTOCOL_LEN                   CONFIG_LASTLINK_PROTOCOL_LENGTH
#define FLAGS_LEN                      CONFIG_LASTLINK_FLAGS_LENGTH
#define ADDRESS_LEN                    CONFIG_LASTLINK_ADDRESS_LENGTH
#define BROADCAST_ADDRESS              CONFIG_LASTLINK_BROADCAST_ADDRESS
#define NULL_ADDRESS                   CONFIG_LASTLINK_NULL_ADDRESS
#define SEQUENCE_NUMBER_LEN            CONFIG_LASTLINK_SEQUENCE_NUMBER_LENGTH
#define INTERVAL_LEN                   CONFIG_LASTLINK_INTERVAL_LENGTH
#define MAX_METRIC                     (TTL_DEFAULT+1)
#define TTL_LEN                        CONFIG_LASTLINK_TTL_LENGTH
#define TTL_DEFAULT                    CONFIG_LASTLINK_TTL_DEFAULT
#define METRIC_LEN                     CONFIG_LASTLINK_METRIC_LENGTH
#define BEACON_NAME_LEN                CONFIG_LASTLINK_BEACON_NAME_LENGTH
#define REASON_LEN                     CONFIG_LASTLINK_REASON_LENGTH
#define PORT_NUM_LEN                   2

/*******************************************************************************************
 *                                                                                         *
 *  A Header common to all packet types.                                                   *
 *                                                                                         *
 *******************************************************************************************/
#define HEADER_ROUTETO_ADDRESS         0
#define HEADER_FLAGS                   (HEADER_ROUTETO_ADDRESS + ADDRESS_LEN)
#define HEADER_ORIGIN_ADDRESS          (HEADER_FLAGS + FLAGS_LEN)
#define HEADER_DEST_ADDRESS            (HEADER_ORIGIN_ADDRESS + ADDRESS_LEN)
#define HEADER_SENDER_ADDRESS          (HEADER_DEST_ADDRESS + ADDRESS_LEN)
#define HEADER_PROTOCOL                (HEADER_SENDER_ADDRESS + ADDRESS_LEN)
#define HEADER_TTL                     (HEADER_PROTOCOL + PROTOCOL_LEN)
#define HEADER_LEN                     (HEADER_TTL + TTL_LEN)

/* The generic payload part of the packet */
#define DATA_PAYLOAD                   (HEADER_LEN + 0)
#define DATA_LEN                       (MAX_PACKET_LEN - HEADER_LEN)

packet_t* generic_packet_create(int address, int protocol, int length);

/*******************************************************************************************
 *                                                                                         *
 *  A local beacon message.                                                                *
 *                                                                                         *
 *******************************************************************************************/
#define BEACON_NAME                    (DATA_PAYLOAD)
#define BEACON_LEN                     (BEACON_NAME + BEACON_NAME_LEN - HEADER_LEN)
#define BEACON_PROTOCOL                0

packet_t* beacon_packet_create(const char*name);


/*******************************************************************************************
 *                                                                                         *
 *  Announce a route either by a ROUTE REQUEST or to publish a well-known route.           *
 *                                                                                         *
 *******************************************************************************************/
#define ROUTEANNOUNCE_FLAGS            (DATA_PAYLOAD)
#define    ROUTEANNOUNCE_FLAGS_GATEWAY    = 0x01
#define ROUTEANNOUNCE_SEQUENCE         (ROUTEANNOUNCE_FLAGS + FLAGS_LEN)
#define ROUTEANNOUNCE_METRIC           (ROUTEANNOUNCE_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define ROUTEANNOUNCE_LEN              (ROUTEANNOUNCE_METRIC + METRIC_LEN - HEADER_LEN)
#define ROUTEANNOUNCE_PROTOCOL         1

/*******************************************************************************************
 *                                                                                         *
 *  Request a route                                                                        *
 *                                                                                         *
 *******************************************************************************************/
#define ROUTEREQUEST_FLAGS             (DATA_PAYLOAD)
#define    ROUTEREQUEST_FLAGS_GATEWAY     = 0x01
#define ROUTEREQUEST_SEQUENCE          (ROUTEREQUEST_FLAGS + FLAGS_LEN)
#define ROUTEREQUEST_METRIC            (ROUTEREQUEST_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define ROUTEREQUEST_LEN               (ROUTEREQUEST_METRIC + METRIC_LEN - HEADER_LEN)
#define ROUTEREQUEST_PROTOCOL          2

#define NAMELOOKUP_NAME_LEN            16

/*******************************************************************************************
 *                                                                                         *
 *  Announce node address lookup by name.  Always a directed reply to a broadcast          *
 *  NAME REQUEST                                                                           *
 *                                                                                         *
 *******************************************************************************************/
#define NAMEANNOUNCE_SEQUENCE          (DATA_PAYLOAD)
#define NAMEANNOUNCE_NAME              (NAMEANNOUNCE_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define NAMEANNOUNCE_LEN               (NAMEANNOUNCE_NAME + NAMEANNOUNCE_NAME_LEN - HEADER_LEN)

#define NAMEANNOUNCE_PROTOCOL       3


/*******************************************************************************************
 *                                                                                         *
 *  Request port of service by type and name.                                              *
 *                                                                                         *
 *  Can be sent to a speecific node (and will always receive a reply even if none) or to   *
 *  BROADCAST, all nodes having the service will respond with a message.  If none, no      *
 *  responses will be received.                                                            *
 *                                                                                         *
 *******************************************************************************************/
#define NAMEREQUEST_SEQUENCE           (DATA_PAYLOAD)
#define NAMEREQUEST_NAME               (NAMEREQUEST_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define NAMEREQUEST_LEN                (NAMEREQUEST_NAME + NAMELOOKUP_NAME_LEN - HEADER_LEN)

#define NAMEREQUEST_PROTOCOL           4

#define ROUTEERROR_ADDRESS             (DATA_PAYLOAD)
#define ROUTEERROR_SEQUENCE            (ROUTEERROR_ADDRESS + ADDRESS_LEN)
#define ROUTEERROR_REASON              (ROUTEERROR_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define ROUTEERROR_LEN                 (ROUTEERROR_REASON + REASON_LEN - HEADER_LEN)

#define ROUTEERROR_PROTOCOL            5

#define FIRST_DATA_PROTOCOL            6


/*************************************************************************
 * This level maintains handles the routing protocol
 * and will deliver non-routing messages to the inheriter.
 *************************************************************************/

/*
 * Init process:
 *   radio_t* radio = create_sx127x();
 *   linklayer_init(radio);
 *
 * Stop:
 *    linklayer_deinit();
 */

#define MAX_DIOS     3

typedef enum {
    RADIO_IS_SX126x_DEVICE,
    RADIO_IS_SX127x_DEVICE,
} radio_type_t;

typedef struct radio_config {
    const char* type;
    radio_type_t radio_type;
    int crystal;
    int channel;
    int delay;
    int dios[MAX_DIOS];
    int reset;
    union {
       struct {
         int spi_host;
         int spi_sck;
         int spi_mosi;
         int spi_miso;
         int spi_cs;
         int spi_clock;
         void (*spi_pre_xfer_callback)(spi_transaction_t*);
         int dma_chan;
       };
       struct {
         int i2c_blah;
         int i2c_blah2;
       };
       struct {
         const char* dev;
       };
    };
} radio_config_t;

bool linklayer_io_init(radio_t* radio, radio_config_t* config);
bool linklayer_init(int address, int flags, int announce_interval);
bool linklayer_add_radio(int radio_num, const radio_config_t* radio);
bool linklayer_remove_radio(int radio_num);

extern int linklayer_node_address;

packet_t* linklayer_create_generic_packet(int dest, int protocol, int length);

bool linklayer_lock();
bool linklayer_unlock();

bool linklayer_deinit(void);
void linklayer_reset(void);

const char* linklayer_escape_raw_data(const uint8_t* data, size_t length);

int linklayer_allocate_sequence(void);
bool linklayer_register_protocol(int number, bool (*protocol_processor)(packet_t* packet), const char* (*protocol_format)(const packet_t* packet));
bool linklayer_unregister_protocol(int number);

os_queue_t linklayer_set_promiscuous_mode(bool mode);
bool linklayer_set_debug(bool enable);

void linklayer_send_packet(packet_t* packet);
void linklayer_send_packet_update_ttl(packet_t* packet);

int linklayer_get_node_address(void);
void linklayer_print_packet(const char* reason, packet_t* packet);

bool linklayer_put_received_packet(packet_t* packet);
bool linklayer_packet_is_for_this_node(const packet_t* p);

void linklayer_release_packets_in_queue(os_queue_t queue);

radio_t* linklayer_get_radio_from_number(int radio_num);

#ifdef CONFIG_LASTLINK_RADIO_SX126x_ENABLED
bool sx126x_radio(radio_t* radio);
#endif

#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED
bool sx127x_radio(radio_t* radio);
#endif

#endif /* __linklayer_h_included */



