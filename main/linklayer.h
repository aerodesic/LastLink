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

/* Default values for some entries */
#define ROUTE_LIFETIME                 CONFIG_LASTLINK_ROUTE_LIFETIME
#define MAX_ROUTES                     CONFIG_LASTLINK_MAX_ROUTES
#define NUM_PACKETS                    CONFIG_LASTLINK_NUM_PACKETS
#define ANNOUNCE_INTERVAL_DEFAULT      CONFIG_LASTLINK_ANNOUNCE_INTERVAL
#define ROUTEREQUEST_TIMEOUT           CONFIG_LASTLINK_ROUTEREQUEST_TIMEOUT
#define ROUTEREQUEST_RETRIES           CONFIG_LASTLINK_ROUTEREQUEST_RETRIES

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

#define HEADER_TARGET_ADDRESS          0
#define HEADER_SOURCE_ADDRESS          (HEADER_TARGET_ADDRESS + ADDRESS_LEN)
#define HEADER_NEXTHOP_ADDRESS         (HEADER_SOURCE_ADDRESS + ADDRESS_LEN)
#define HEADER_PREVIOUS_ADDRESS        (HEADER_NEXTHOP_ADDRESS + ADDRESS_LEN)
#define HEADER_PROTOCOL                (HEADER_PREVIOUS_ADDRESS + ADDRESS_LEN)
#define HEADER_TTL                     (HEADER_PROTOCOL + PROTOCOL_LEN)
#define HEADER_LEN                     (HEADER_TTL + TTL_LEN)

packet_t* generic_packet_create(int address, int protocol, int length);

#define BEACON_NAME                    (HEADER_LEN+0)
#define BEACON_LEN                     (BEACON_NAME + BEACON_NAME_LEN - HEADER_LEN)
#define BEACON_PROTOCOL                0

packet_t* beacon_packet_create(const char*name);

#define RANN_FLAGS                     (HEADER_LEN + 0)
#define    RANN_FLAGS_GATEWAY                = 0x01
#define RANN_SEQUENCE                  (RANN_FLAGS + FLAGS_LEN)
#define RANN_METRIC                    (RANN_SEQUENCE + SEQUENCE_NUMBER_LEN)
#define RANN_LEN                       (RANN_METRIC + METRIC_LEN - HEADER_LEN)
#define RANN_PROTOCOL                  1

#define RREQ_FLAGS                     (HEADER_LEN + 0)
#define    RREQ_FLAGS_GATEWAY                = 0x01
#define RREQ_SEQUENCE                  (RREQ_FLAGS + FLAGS_LEN)
#define RREQ_METRIC                    (RREQ_SEQUENCE + SEQUENCE_NUMBER_LEN - HEADER_LEN)
#define RREQ_LEN                       (RREQ_METRIC + METRIC_LEN)
#define RREQ_PROTOCOL                  2

#define RERR_ADDRESS                   (HEADER_LEN + 0)
#define RERR_SEQUENCE                  (RERR_ADDRESS + ADDRESS_LEN)
#define RERR_REASON                    (RERR_SEQUENCE + SEQUENCE_NUMBER_LEN - HEADER_LEN)
#define RERR_LEN                       (RERR_REASON + REASON_LEN)
#define RERR_PROTOCOL                  3

#define DATA_PAYLOAD                   (HEADER_LEN + 0)
#define DATA_LEN                       (MAX_PACKET_LEN - HEADER_LEN)
#define DATA_PROTOCOL                  4


packet_t* create_data_packet(int target, int protocol, uint8_t* data, int length);

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

bool linklayer_init(int address, int flags, int announce_interval);
bool linklayer_deinit(void);
void linklayer_reset(void);
bool linklayer_add_radio(radio_t* radio);
bool linklayer_remove_radio(radio_t* radio);

void linklayer_send_packet(packet_t* packet);
void linklayer_send_packet_update_ttl(packet_t* packet);

int linklayer_get_node_address(void);

#endif /* __linklayer_h_included */


