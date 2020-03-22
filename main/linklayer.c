/*
 * linklayer.c
 *
 * This module contains the top-level LastLink packetizer and distributor.
 *
 * It draws upon a lower-level packet serializer that can be one of several
 * devices (currently SX127x) interfaced via SPI.
 *
 * The link layer
 */

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "os_freertos.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "esp_system.h"
#include "esp_log.h"

//#include "lwip/err.h"
//#include "lwip/sys.h"

#include "packets.h"
#include "routes.h"
#include "radio.h"
#include "linklayer.h"
#include "linklayer_io.h"

#ifdef CONFIG_LASTLINK_RADIO_SX126x_ENABLED
#include "sx126x_driver.h"
#endif
#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED
#include "sx127x_driver.h"
#endif

#define TAG  "linklayer"

static void beacon_packet_process(packet_t* p);
static const char* beacon_packet_format(const packet_t* p);

static packet_t* routeannounce_packet_create(int dest, int sequence, int metric);
static void routeannounce_packet_process(packet_t* p);

static const char* routeannounce_packet_format(const packet_t* p);
static packet_t* routerequest_packet_create(int address);

static void routerequest_packet_process(packet_t* p);
static const char* routerequest_packet_format(const packet_t* p);

static packet_t* routeerror_packet_create(int dest, int address, int sequence, const char* reason);
static void routeerror_packet_process(packet_t* p);

static const char* ping_packet_format(const packet_t* p);
static const char* pingreply_packet_format(const packet_t* p);
static void ping_packet_process(packet_t* p);
static void pingreply_packet_process(packet_t* p);

static void data_packet_process(packet_t* p);
static char tohex(int v);
static const char* escape_raw_data(const uint8_t* data, size_t length);
static const char* default_packet_format(const packet_t* p);
static bool put_received_packet(packet_t* p);

static bool linklayer_init_radio(radio_t* radio);
static bool linklayer_deinit_radio(radio_t*);
static void linklayer_transmit_packet(radio_t* radio, packet_t* packet);
static const  char* linklayer_packet_format(const packet_t* packet, int protocol);

/* Calls from radio driver to linklayer */
static bool linklayer_attach_interrupt(radio_t* radio, int dio, GPIO_INT_TYPE edge, void (*handler)(void* p));
static void linklayer_on_receive(radio_t* radio, packet_t* packet);
static packet_t* linklayer_on_transmit(radio_t* radio);
static void reset_device(radio_t* radio);

static os_mutex_t                 linklayer_lock;
static int                        node_address;
static int                        node_flags;
static os_queue_t                 receive_queue;
static os_queue_t                 promiscuous_queue;
static int                        sequence_number;
static bool                       debug_flag;
static os_thread_t                announce_thread;
static int                        announce_interval;
static route_table_t              route_table;
static int                        packet_errors_crc;   /* Packets with crc errors */
static int                        packet_received;     /* Total packets received */
static int                        packet_processed;    /* Packets accepted for processing */
static int                        packet_transmitted;  /* Total packets transmitted */
static int                        packet_ignored;      /* Packets not processed */
static radio_t**                  radio_table;

typedef struct protocol_entry {
    void        (*process)(packet_t*);                 /* Process function */
    const char* (*format)(const  packet_t*);           /* Format for printing */
} protocol_entry_t;

static protocol_entry_t           protocol_table[CONFIG_LASTLINK_MAX_PROTOCOL_NUMBER + 1];
                            

#ifdef NOTUSED
static void test_button_handler(void*);
#endif

/*
 * Configuration
 */
#define SPI_GPIO_DIO0      0
#define SPI_GPIO_DIO1      1
#define SPI_GPIO_DIO2      2
#define SPI_GPIO_SCK       3
#define SPI_GPIO_MOSI      4
#define SPI_GPIO_MISO      5
#define SPI_GPIO_SS        6
#define SPI_GPIO_RESET     7
#define SPI_NUM_GPIO       8

#define MAX_RADIO_GPIO     SPI_NUM_GPIO

#define RADIO_CONFIG_EXPAND(name_begin, radio, name_end) name_begin##_##radio##_##name_end

#define RADIO_CONFIG_SPI(radio, module) \
    { \
    .type       = "spi",                                                         \
    .radio_type = RADIO_CONFIG_EXPAND(RADIO_IS, module, DEVICE),                 \
    .crystal    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, CRYSTAL),    \
    .channel    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, CHANNEL),    \
    .delay      = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, DELAY),      \
    .dios[0]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_DIO0),  \
    .dios[1]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_DIO1),  \
    .dios[2]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_DIO2),  \
    .reset      = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_RESET), \
    .spi_sck    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_SCK),   \
    .spi_mosi   = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_MOSI),  \
    .spi_miso   = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_MISO),  \
    .spi_cs     = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_SS),    \
    .spi_host   = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, SPI_HOST),   \
    .spi_clock  = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, SPI_CLOCK_HZ), \
    .spi_pre_xfer_callback = NULL,                                               \
    .dma_chan   = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, DMA_CHAN),   \
   },

#define RADIO_CONFIG_I2C(radio, module) \
    { \
    .type       = "i2c",                                                         \
    .radio_type = RADIO_CONFIG_EXPAND(RADIO_IS, module, DEVICE),                 \
    .crystal    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, CRYSTAL),    \
    .channel    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, CHANNEL),    \
    .delay      = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, DELAY),      \
    .reset      = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, I2C_RESET),  \
    .dios[0]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, I2C_DIO0),   \
    .dios[1]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, I2C_DIO1),   \
    .dios[2]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, I2C_DIO2),   \
    .i2c_blah1  = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, I2C_blah1),  \
    .i2c_blah2  = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, I2C_blah2),  \
   },

#define RADIO_CONFIG_SERIAL(radio, module) \
    { \
    .type       = "serial",                                                      \
    .radio_type = RADIO_CONFIG_EXPAND(RADIO_IS, module, DEVICE),                 \
    .crystal    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, CRYSTAL),    \
    .channel    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, CHANNEL),    \
    .delay      = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, DELAY),      \
    .dios[0]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, SER_DIO0),   \
    .dios[1]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, SER_DIO1),   \
    .dios[2]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, SER_DIO2),   \
    .reset      = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, SER_RESET),  \
    .dev        = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, SER_DEV),    \
   },

static const radio_config_t radio_config[] = {

#if defined(CONFIG_LASTLINK_RADIO_1_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_1_SX126x_SPI)
    RADIO_CONFIG_SPI(1, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_1_SX127x_SPI)
    RADIO_CONFIG_SPI(1, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_2_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_2_SX126x_SPI)
    RADIO_CONFIG_SPI(2, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_2_SX127x_SPI)
    RADIO_CONFIG_SPI(2, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_3_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_3_SX126x_SPI)
    RADIO_CONFIG_SPI(3, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_3_SX127x_SPI)
    RADIO_CONFIG_SPI(3, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_4_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_4_SX126x_SPI)
    RADIO_CONFIG_SPI(4, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_4_SX127x_SPI)
    RADIO_CONFIG_SPI(4, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_5_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_5_SX126x_SPI)
    RADIO_CONFIG_SPI(5, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_5_SX127x_SPI)
    RADIO_CONFIG_SPI(5, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_6_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_6_SX126x_SPI)
    RADIO_CONFIG_SPI(6, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_6_SX127x_SPI)
    RADIO_CONFIG_SPI(6, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_7_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_7_SX126x_SPI)
    RADIO_CONFIG_SPI(7, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_7_SX127x_SPI)
    RADIO_CONFIG_SPI(7, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_8_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_8_SX126x_SPI)
    RADIO_CONFIG_SPI(8, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_8_SX127x_SPI)
    RADIO_CONFIG_SPI(8, SX127x)
  #endif
#endif
};

#define NUM_RADIOS ELEMENTS_OF(radio_config)

/*
 * Create a generic packet
 *
 * Entry:
 *      dest          Target address
 *      protocol        Protocol number
 *      length          Payload length
 */
packet_t* create_generic_packet(int dest, int protocol, int length)
{
    packet_t* p = allocate_packet();
    if (p != NULL) {
         /* Set to header length plus payload required */
         p->length = HEADER_LEN + length;
         set_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN, dest);
         set_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
         set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
         set_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
         set_uint_field(p, HEADER_PROTOCOL, PROTOCOL_LEN, protocol);
         set_uint_field(p, HEADER_TTL, TTL_LEN, TTL_DEFAULT);
    }
    return p;
}

/*
 * Create a Beacon packet
 */
packet_t* beacon_packet_create(const char* name)
{
    int length = strlen(name);
    if (length > BEACON_NAME_LEN) {
        length = BEACON_NAME_LEN;
    }

    packet_t* p = create_generic_packet(BROADCAST_ADDRESS, BEACON_PROTOCOL, length);
    if (p != NULL) {
        set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        set_int_field(p, HEADER_TTL, TTL_LEN, 1);
        int moved = set_str_field(p, BEACON_NAME, length, name);
        /* Update packet length */
        p->length = HEADER_LEN + moved;
    }
    return p;
}

static void beacon_packet_process(packet_t* p)
{
    if (p != NULL) {
        const char* name = get_str_field(p, BEACON_NAME, BEACON_NAME_LEN);

        ESP_LOGD(TAG, "Beacon: dest %d origin %d routeto %d sender %d ttl %d name '%s'",
                 get_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN),
                 get_int_field(p, HEADER_TTL, TTL_LEN),
                 name);

        free((void*) name);

        release_packet(p);
    }
}

static const char* beacon_packet_format(const packet_t* p)
{
    char* info;

    const char* name = get_str_field(p, BEACON_NAME, BEACON_NAME_LEN);

    asprintf(&info, "Beacon: %s", name);
    free((void*) name);

    return info;
}

 
/*
 * RouteAnnounce is used to advertise a route to a node, usually a gateway node.
 *
 * When sent to a specific node, it announces an established route.
 * When sent as broadcast, it helps establish routes (and gateway status)
 * and is rebroadcast so adjacent nodes will see it.
 */
static packet_t* routeannounce_packet_create(int dest, int sequence, int metric)
{
    packet_t* p = create_generic_packet(dest, RANN_PROTOCOL, RANN_LEN);
    if (p != NULL) {
        set_uint_field(p, RANN_FLAGS, FLAGS_LEN, node_flags);
        set_uint_field(p, RANN_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
        set_uint_field(p, RANN_METRIC, METRIC_LEN, metric);
        set_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, node_address);
        set_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN, node_address);
    }

    return p;
}

static void routeannounce_packet_process(packet_t* p)
{
    if (p != NULL) {
        linklayer_print_packet("Route Announce", p);
        if (route_table_lock(&route_table)) {
            route_t* route = route_update(&route_table,
                                          p->radio_num,
                                          get_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                          get_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN),
                                          get_uint_field(p, RANN_SEQUENCE, SEQUENCE_NUMBER_LEN),
                                          get_uint_field(p, RANN_METRIC, METRIC_LEN),
                                          get_uint_field(p, RANN_FLAGS, FLAGS_LEN));

            if (route != NULL) {
                /* If this packet was for us, then release any pending route packets to queue now */
                if (get_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN) == node_address) {
                    route_release_packets(route);
                } else {
                    /* Not for us, so clear routeto so a re-route will be done */
                    set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
                    /* Update the metric */
                    set_uint_field(p, RANN_METRIC, METRIC_LEN, get_uint_field(p, RANN_METRIC, METRIC_LEN));
                    linklayer_send_packet_update_ttl(p);
                }
            }

            route_table_unlock(&route_table);

        } else {
            /* Unable to get mutex */
        }

        release_packet(p);
    }
}

static const char* routeannounce_packet_format(const packet_t* p)
{
    char* info;

    int sequence = get_uint_field(p, RANN_SEQUENCE, SEQUENCE_NUMBER_LEN);
    int metric = get_uint_field(p, RANN_METRIC, METRIC_LEN);
    int flags = get_uint_field(p, RANN_FLAGS, FLAGS_LEN);

    asprintf(&info, "Route Announce: seq %d mestric %d flags %02x", sequence, metric, flags);

    return info;
}

/*
 * A route request is sent by a node when it needs to know a route to a specific dest
 * node.  This packet is always braodcast rather than single address.
 *
 * This packet will be rebroadcast until it reaches all nodes.  This will
 * partially build routing tables through the N-1 nodes handling the packets
 * but will result in a RouteAnnounce from the destination node indicating
 * to the caller the node that returned it plus the metric to that node.
 */
static packet_t* routerequest_packet_create(int address)
{
    packet_t* p = create_generic_packet(address, RREQ_PROTOCOL, RREQ_LEN);

    if (p != NULL) {
        set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        set_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, node_address);
        set_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN, node_address);
        set_uint_field(p, RREQ_FLAGS, FLAGS_LEN, node_flags);
        set_uint_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());
        set_uint_field(p, RREQ_METRIC, METRIC_LEN, 0);
    }

    return p;
}

static void routerequest_packet_process(packet_t* p)
{
    if (p != NULL) {
        linklayer_print_packet("Route Request", p);

        if (os_acquire_recursive_mutex(linklayer_lock)) {

            /* TODO: Need brakes to avoid transmitting too many at once !! (Maybe ok for testing) */

            /* Update the route table and see if we have a route */
            route_t* route = route_update(
                                 &route_table,
                                 p->radio_num,
                                 get_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                 get_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN),
                                 get_uint_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN),
                                 get_uint_field(p, RREQ_METRIC, METRIC_LEN),
                                 get_uint_field(p, RREQ_FLAGS, FLAGS_LEN));

            /* If packet is dest for our node, process it */
            if (get_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN) == node_address) {
                packet_t* ra = routeannounce_packet_create(
                                          get_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                          get_int_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN),
                                          get_uint_field(p, RREQ_METRIC, METRIC_LEN));

                ra->radio_num = p->radio_num;
                linklayer_send_packet(ref_packet(ra));
                release_packet(ra);

            /* Else if it's a broadcast and we haven't seen it before, rebroadcast it to next hop */
            } else if (get_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN) == BROADCAST_ADDRESS && route != NULL) {
                set_uint_field(p, RREQ_METRIC, METRIC_LEN, get_uint_field(p, RREQ_METRIC, METRIC_LEN));
                linklayer_send_packet_update_ttl(p);
            }
            os_release_recursive_mutex(linklayer_lock);

        } else {
            /* Cannot get mutex */
        }

        release_packet(p);
    }
}

static const char* routerequest_packet_format(const packet_t* p)
{
    char* info;

    int sequence = get_uint_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN);
    int metric = get_uint_field(p, RREQ_METRIC, METRIC_LEN);
    int flags = get_uint_field(p, RREQ_FLAGS, FLAGS_LEN);

    asprintf(&info, "Route Request: seq %d mestric %d flags %02x", sequence, metric, flags);

    return info;
}

/*
 * A RouteError is returned to the origin for any packet that cannot be delivered.
 */
static packet_t* routeerror_packet_create(int dest, int address, int sequence, const char* reason)
{
    packet_t* p = create_generic_packet(dest, RERR_PROTOCOL, RERR_LEN);
    if (p != NULL) {
        set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        set_uint_field(p, RERR_ADDRESS, ADDRESS_LEN, address);
        set_uint_field(p, RERR_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
        set_str_field(p, RERR_REASON, REASON_LEN, reason);
    }

    return p;
}

static void routeerror_packet_process(packet_t* p)
{
    if (p != NULL) {

        if (os_acquire_recursive_mutex(linklayer_lock)) {
            const char* reason = get_str_field(p, RERR_REASON, REASON_LEN);

            ESP_LOGD(TAG, "RouteError: dest %d origin %d routeto %d sender %d ttl %d address %d sequence %d reason '%s'",
                     get_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN),
                     get_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                     get_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN),
                     get_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN),
                     get_int_field(p, HEADER_TTL, TTL_LEN),
                     get_uint_field(p, RERR_ADDRESS, ADDRESS_LEN),
                     get_uint_field(p, RERR_SEQUENCE, SEQUENCE_NUMBER_LEN), reason);

            free((void*) reason);

            os_release_recursive_mutex(linklayer_lock);

        } else {
            /* Cannot get mutex */
        }

        release_packet(p);
    }
}

static const char* routeerror_packet_format(const packet_t* p)
{
    char* info;

    int address = get_uint_field(p, RERR_ADDRESS, ADDRESS_LEN);
    int sequence = get_uint_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN);

    asprintf(&info, "Route Error: seq %d address %x", sequence, address);

    return info;
}


/*
 * Ping packet create
 */
packet_t* ping_packet_create(int dest)
{
    packet_t* p;

    p = create_generic_packet(dest, PING_PROTOCOL, PING_LEN);

    if (p != NULL) {
        /* Generate sequence number */
        set_uint_field(p, PING_SEQUENCE, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());

        /* Put starting address in the table */
        set_uint_field(p, PING_ROUTE_TABLE, ADDRESS_LEN, node_address);
    }

    return p;
}

static void ping_packet_process(packet_t* p)
{
    if (p != NULL) {
linklayer_print_packet("PING RECEIVE", p);

        if (os_acquire_recursive_mutex(linklayer_lock)) {
            /* Sanity check - reuse last entry if full */
            if (p->length >= MAX_PACKET_LEN) {
                p->length = MAX_PACKET_LEN - ADDRESS_LEN;
            }

            p->length += ADDRESS_LEN;
            set_uint_field(p, p->length - ADDRESS_LEN, ADDRESS_LEN, node_address);


            /* If for us, turn packet into PING_REPLY */
            if (get_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN) == node_address) {
                set_uint_field(p, HEADER_PROTOCOL, PROTOCOL_LEN, PINGREPLY_PROTOCOL);
                /* And reroute back to sender */
                set_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN, get_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN));
                set_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, node_address);
                /* Rewind TTL */
                set_uint_field(p, HEADER_TTL, TTL_LEN, TTL_DEFAULT);

linklayer_print_packet("PING REPLY", p);

                linklayer_send_packet(ref_packet(p));
            } else {
                /* Relable routeto so it forwards it on */
                set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
                linklayer_send_packet_update_ttl(ref_packet(p));
            }

            os_release_recursive_mutex(linklayer_lock);

        } else {
            /* Cannot get mutex */
        }

        release_packet(p);
    }
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

static void pingreply_packet_process(packet_t* p)
{
    if (p != NULL) {
        if (os_acquire_recursive_mutex(linklayer_lock)) {

            /* If reply is for us, we do not add our address - but deliver it */
            if (get_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN) == node_address) {
                /* Deliver packet to process queue */
                put_received_packet(ref_packet(p));
            } else {
                /* Sanity check - reuse last entry if full */
                if (p->length >= MAX_PACKET_LEN) {
                    p->length = MAX_PACKET_LEN - ADDRESS_LEN;
                }

                p->length += ADDRESS_LEN;
                set_uint_field(p, p->length - ADDRESS_LEN, ADDRESS_LEN, node_address);
            }

            linklayer_send_packet_update_ttl(ref_packet(p));

            os_release_recursive_mutex(linklayer_lock);

        } else {
            /* Cannot get mutex */
        }

        release_packet(p);
    }
}

/*
 * A Data packet is used to convey any data between nodes.  This packet
 * will be overloaded for any purpose that is needed.
 */
packet_t* data_packet_create(int dest, int protocol, uint8_t* data, int length)
{
    packet_t* p;

    if (DATA_PAYLOAD + length <= DATA_LEN) {
        p = create_generic_packet(dest, protocol, length);

        if (p != NULL) {
            /* Move the data and set the final packet length */
            int moved = set_bytes_field(p, DATA_PAYLOAD, length, data);
            p->length = DATA_PAYLOAD + moved;
        }

    } else {
        /* Invalid length */
        p = NULL;
    }

    return p;
}

static void data_packet_process(packet_t* p)
{
    if (p != NULL) {
        if (os_acquire_recursive_mutex(linklayer_lock)) {
            if (get_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN) == node_address) {
                put_received_packet(ref_packet(p));
            } else {
                set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
                linklayer_send_packet_update_ttl(ref_packet(p));
            }

            os_release_recursive_mutex(linklayer_lock);
        } else {
            /* Cannot get mutex */
        }

        release_packet(p);
    }
}

static char tohex(int v)
{
    v &= 0x0F;
    if (v <= 9) {
        return '0' + v;
    } else {
        return 'A' + v - 10;
    }
}

static const char* escape_raw_data(const uint8_t* data, size_t length)
{
    /* Determine length of output data field */
    int outlen = 0;

//ESP_LOGI(TAG, "%s: data at %p length %u", __func__, data, length);

    for (int index = 0; index < length; ++index) {
        if (isprint(data[index])) {
            outlen += 1;
        } else {
            /* Allow 4 for each non-printable, even though some will be only two (e.g. '\n') */
            outlen += 4;  // "\xNN";
        }
    }

    char *outdata = (char*) malloc(outlen + 1);
    if (outdata != NULL) {
        char* outp = outdata;
        for (const uint8_t* p = data; *p != '\0'; ++p) {
            if (isprint(*p)) {
                *outp++ = *p;
            } else if (*p == '\n') {
                *outp++ = '\\';
                *outp++ = 'n';
            } else if (*p == '\r') {
                *outp++ = '\\';
                *outp++ = 'r';
            } else if (*p == '\a') {
                *outp++ = '\\';
                *outp++ = 'a';
            } else if (*p == '\t') {
                *outp++ = '\\';
                *outp++ = 't';
            } else if (*p == '\f') {
                *outp++ = '\\';
                *outp++ = 'f';
            } else {
                *outp++ = '\\';
                *outp++ = 'x';
                *outp++ = tohex(*p >> 4);
                *outp++ = tohex(*p);
            }
        }
        *outp = '\0';
    }
    return outdata;
}

static const char* default_packet_format(const packet_t* p)
{
    char* info;

//ESP_LOGI(TAG, "%s: packet %p length %u", __func__, p, p->length);

    /* Extract from end of header to end of packet */
    const char* data = escape_raw_data(p->buffer + HEADER_LEN, p->length - HEADER_LEN);
 
    asprintf(&info, "Data: \"%s\"", data);

    free((void*) data);

    return info;
}

/* Creates the linklayer */
bool linklayer_init(int address, int flags, int announce)
{
    bool ok = false;

    ESP_LOGI(TAG, "%s: address %d flags 0x%02x announce %d", __func__, address, flags, announce);

    linklayer_lock = os_create_recursive_mutex();
    node_address = address;
    node_flags = flags; 

    /* Allocate packets */
    ok = init_packets(NUM_PACKETS);

    if (ok) {
#if DEBUG
        /* Get a free packet */
        packet_t* packet = allocate_packet();
        ESP_LOGD(TAG, "allocate_packet returned %p (ref %d, length %d)", packet, packet->ref, packet->length);

        /* Show packets available */
        ESP_LOGD(TAG, "number of packets available %d", available_packets());

        ok = release_packet(packet);
        ESP_LOGD(TAG, "release_packet returned %s", ok ? "OK" : "FAIL");

        /* Show packets available */
        ESP_LOGD(TAG, "number of packets available %d", available_packets());
#endif

        /* Receive queue goes to packet data layer */
        receive_queue = os_create_queue(NUM_PACKETS, sizeof(packet_t*));

        promiscuous_queue = NULL;
        sequence_number = 0;
        debug_flag = false;
        announce_interval = announce;

        if (announce_interval != 0) {
            /* Start the route announce thread */
        }

        route_table_init(&route_table);
        packet_errors_crc = 0;
        packet_processed = 0;
        packet_received = 0;
        packet_transmitted = 0;
        packet_ignored = 0;

        ok = true;

        /* Reserve first packet protocol types */
        if (linklayer_register_protocol(BEACON_PROTOCOL,    beacon_packet_process,        beacon_packet_format)        &&
            linklayer_register_protocol(RREQ_PROTOCOL,      routerequest_packet_process,  routerequest_packet_format)  &&
            linklayer_register_protocol(RANN_PROTOCOL,      routeannounce_packet_process, routeannounce_packet_format) &&
            linklayer_register_protocol(RERR_PROTOCOL,      routeerror_packet_process,    routeerror_packet_format)    &&
            linklayer_register_protocol(PING_PROTOCOL,      ping_packet_process,          ping_packet_format)          &&
            linklayer_register_protocol(PINGREPLY_PROTOCOL, pingreply_packet_process,     pingreply_packet_format)     &&
            linklayer_register_protocol(DATA_PROTOCOL,      data_packet_process,          default_packet_format)) {

            for (int radio_num = 0; radio_num < NUM_RADIOS; ++radio_num) {
                ok = ok && linklayer_add_radio(radio_num, &radio_config[radio_num]);
            }
        } else {
           ok = false;
        }
    }

#ifdef NOTUSED
    if (ok) {
        ok = os_attach_gpio_interrupt(0, GPIO_INT_PULLDOWN, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, test_button_handler, (void*) 0);
        if (ok) {
            ESP_LOGI(TAG, "Button interrupt attached");
        } else {
            ESP_LOGI(TAG, "Button interrupt attach failed");
        }
    }
/*END DEBUG*/
#endif

    return ok;
}

#ifdef NOTUSED
int button_interrupts;
static void test_button_handler(void* param)
{
    ++button_interrupts;
}
#endif

bool linklayer_set_promiscuous_mode(bool mode) {
    bool ok = true;

    if (mode) {
        if (promiscuous_queue == NULL) {
            promiscuous_queue = os_create_queue(NUM_PACKETS, sizeof(packet_t*));
            ok = promiscuous_queue != NULL;
        }
    } else {
        if (promiscuous_queue != NULL) {
            ok = os_delete_queue(promiscuous_queue);
            promiscuous_queue = NULL;
        }
    }

    return ok;
}

bool linklayer_set_debug(bool enable)
{
    debug_flag = enable;
    return true;
}

/*
 * This adds a radio definition from a radio_config_t entry.
 */
bool linklayer_add_radio(int radio_num, const radio_config_t* config)
{
    bool ok = false;

    if (radio_table != NULL && radio_table[radio_num] != NULL) {
        linklayer_remove_radio(radio_num);
    }

    /* Allocate radio structure */
    radio_t* radio = (radio_t*) malloc(sizeof(radio_t));
ESP_LOGI(TAG, "%s: radio_num %d allocated radio_t at %p", __func__, radio_num, radio);

    if (radio != NULL) {
        memset(radio, 0, sizeof(radio_t));

        radio->radio_num = radio_num;

        /* Add our information */
        linklayer_init_radio(radio);

        /*
         * Create the io device wrapper for radio.
         * This fills in the device handle field (e.g. spi channel), read and write functions, `k
         */
        if (io_init(radio, config)) {
            /*
             * Initialize the radio.  Fills in the the 'set by driver' section.
             * Check only the radios types that are configured.
             */
#ifdef CONFIG_LASTLINK_RADIO_SX126x_ENABLED
            if (config->radio_type == RADIO_IS_SX126x_DEVICE) {
                ok = sx126x_create(radio);
            }
#endif
#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED
            if (config->radio_type == RADIO_IS_SX127x_DEVICE) {
                ok = sx127x_create(radio);
            }
#endif
            if (ok) {
//                if (radio->set_channel(radio, config->channel)) {

                    /* Put radio in active table. If no radio, create one. */
                    if (radio_table == NULL) {
                        /* Create radio table */
                        radio_table = (radio_t**) malloc(sizeof(radio_t*) * NUM_RADIOS);

ESP_LOGI(TAG, "%s: allocated radio_table at %p", __func__, radio_table);
                        if (radio_table != NULL) {
                            memset(radio_table, 0, sizeof(radio_t*) * NUM_RADIOS);
                        }
                    }
//                }
            }

            if (radio_table != NULL) {
                radio_table[radio_num] = radio;
            } else {
                ok = false;
            }
        }

        if (!ok) {
            linklayer_deinit_radio(radio);

            /* init or radio failure.  deinit  */
            io_deinit(radio);

            /* Release storage */
            free((void*) radio);
        }
    }
    return ok;
}

bool linklayer_remove_radio(int radio_num)
{
    bool ok = false;

    if (radio_table != NULL && radio_num >= 0 && radio_num < NUM_RADIOS && radio_table[radio_num] != NULL) {
        radio_t* radio = radio_table[radio_num];

        /* Shut down the radio itself */
        if ((radio->stop == NULL) || radio->stop(radio)) {

            /* Deinit the bus connection */
            if ((radio->bus_deinit == NULL) || radio->bus_deinit(radio)) {

                /* Deinit the linklayer parts */
                ok = linklayer_deinit_radio(radio);

                if (ok) {
                    free((void*) radio);
                    /* Clear radio table entry */
                    radio_table[radio_num] = NULL;
                }
            }
        }
    }

    return ok;
}

static bool linklayer_init_radio(radio_t* radio)
{
    radio->attach_interrupt = linklayer_attach_interrupt;
    radio->on_receive = linklayer_on_receive;
    radio->on_transmit = linklayer_on_transmit;
    radio->transmit_queue = os_create_queue(NUM_PACKETS, sizeof(packet_t*));
    radio->reset_device = reset_device;

    return true;
}

radio_t* linklayer_get_radio_from_number(int radio_num)
{
    if (radio_table != NULL && radio_num >= 0 && radio_num < NUM_RADIOS) {
        return radio_table[radio_num];
    } else {
        return NULL;
    }
}

static void reset_device(radio_t* radio)
{
    int gpio = radio_config[radio->radio_num].reset;

    gpio_config_t io;
    io.intr_type = GPIO_PIN_INTR_DISABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = 1ULL << gpio;
    io.pull_down_en = 0;
    io.pull_up_en = 0;
    gpio_config(&io);

    if (gpio_set_level(gpio, 0) != ESP_OK) {
        ESP_LOGE(TAG, "%s: ******************************* error setting reset low", __func__);
    }

    os_delay(100);

    if (gpio_set_level(gpio, 1) != ESP_OK) {
       ESP_LOGE(TAG, "%s: ******************************* error setting reset high", __func__);
    }

    /* Leave configured as output with pulled up */
}

static bool linklayer_deinit_radio(radio_t* radio)
{
    bool ok = true;

    if (radio != NULL) {
        /* Deinitialize the radio module if active */
        if (radio->stop == NULL || (ok = radio->stop(radio))) {
           radio->attach_interrupt = NULL;
           radio->on_receive = NULL;
           radio->on_transmit = NULL;
           os_delete_queue(radio->transmit_queue);
           radio->transmit_queue = NULL;
           radio->reset_device = NULL;
        }
    }

    return ok;
}

bool linklayer_deinit(void)
{
    if (os_acquire_recursive_mutex(linklayer_lock)) {

        if (announce_thread != NULL) {
            os_delete_thread(announce_thread);
            announce_thread = NULL;
        }

        if (receive_queue != NULL) {
            os_delete_queue(receive_queue);
            receive_queue = NULL;
        }

        route_table_deinit(&route_table);

        /* Deinitialize and remove all radios */
        for (int radio_num = 0; radio_num < NUM_RADIOS; ++radio_num) {
            linklayer_remove_radio(radio_num);
        }

        os_release_recursive_mutex(linklayer_lock);
        os_delete_mutex(linklayer_lock);

        linklayer_lock = NULL;
    }

    return linklayer_lock == NULL;
}

int linklayer_allocate_sequence(void)
{
    int sequence = 0;

    if (os_acquire_recursive_mutex(linklayer_lock)) {
        sequence = ++sequence_number;
        os_release_recursive_mutex(linklayer_lock);
    }

    return sequence;
}

/*
 * Send a packet
 */
void linklayer_send_packet(packet_t* packet)
{
    if (packet == NULL) {
        ESP_LOGE(TAG, "%s: Packet is NULL", __func__);
    } else {
        /*
         * A packet with a origin and destination is ready to transmit.
         * Label the from address and if no to address, attempt to route
         */

        /* Indicate coming from us */
        set_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN, node_address);
    
        /* If origin is NULL address, set it to us */
        if (get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN) == NULL_ADDRESS) {
            set_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, node_address);
        }
    
        /*
         * If the routeto is NULL, then we compute next hop based on route table.
         * If no route table, create pending NULL route and cache packet for later retransmission.
         */
        unsigned int routeto = get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN);
        
        radio_t* radio = NULL;  /* Gets the target radio device address */
    
        if (routeto == NULL_ADDRESS) {
        
            unsigned int dest = get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN);
        
            route_table_lock(&route_table);
        
            route_t* route = route_find(&route_table, dest);
        
            /* If no route, create a dummy and make a request */
            if (route == NULL) {
                route = route_update(&route_table, packet->radio_num, dest, NULL_ADDRESS, linklayer_allocate_sequence(), 1, 0);
        
                /* Queue with it's pending ownership */
                route_put_pending_packet(route, packet);
        
                if (debug_flag) {
                    linklayer_print_packet("Routing", packet);
                }
        
                /* Create a route request to be sent instead */
                packet = routerequest_packet_create(dest);
                route_set_pending_request(route, packet, ROUTE_REQUEST_RETRIES, ROUTE_REQUEST_TIMEOUT);
        
            } else if (route->routeto == NULL_ADDRESS) {
                /* Still have pending route, add packet to queue */
                route_put_pending_packet(route, packet);
                /* And drop it */
                packet = NULL;

            } else {
                /* We have a route so select routeto and radio */
                set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, route->routeto);
                if (route->radio_num != UNKNOWN_RADIO) {
        ESP_LOGI(TAG, "%s: routing to radio %d", __func__, route->radio_num);
                    radio = radio_table[route->radio_num];
                } else {
        ESP_LOGE(TAG, "%s: no radio for route", __func__);
                    release_packet(packet);
                    packet = NULL;
                }
            }

            route_table_unlock(&route_table);
        }
    
        /*
         * TODO: we may need another method to restart transmit queue other than looking
         * and transmit queue length.  A 'transmitting' flag (protected by meshlock) that
         * is True if transmitting of packet is in progress.  Cleared on onTransmit when
         * queue has become empty.
         *
         * This may need to be implemented to allow stalling transmission for windows of
         * reception.  A timer will then restart the queue if items remain within it.
         */
    
        /* If radio is NULL, packet will be sent through all radios */
        if (packet != NULL) {
            linklayer_transmit_packet(radio, ref_packet(packet));
            release_packet(packet);
        }
    }

    release_packet(packet);
}

void linklayer_send_packet_update_ttl(packet_t* packet)
{
    int ttl = get_uint_field(packet, HEADER_TTL, TTL_LEN);
    if (--ttl > 0) {
        set_uint_field(packet, HEADER_TTL, TTL_LEN, ttl);
        linklayer_send_packet(packet);
    } else {
linklayer_print_packet("TTL EXPIRED", packet);
        release_packet(packet);
    }
}

/*
 * linklayer_transmit_packet

 * Put the packet onto the radio send queue.  If the queue now contains one item,
 * activate the send by issuing a send_packet to the radio.
 *
 * When the on_transmit() callback fires, we discard the top packet of the queue
 * and peek to see if the queue has another item.  If so, we return that item
 * for the next transmit action.
 *
 * Entry:
 *      radio           The radio to transmit the packet
 *      packet          The packet to transmit
 *
 * Past this point, the 'radio_num' in the packet is meaningless.
 */
static void linklayer_transmit_packet(radio_t* radio, packet_t* packet)
{
    if (radio == NULL) {
        for (int radio_num = 0; radio_num < NUM_RADIOS; ++radio_num) {
            linklayer_transmit_packet(radio_table[radio_num], ref_packet(packet));
        }
    } else {
        char *info;
        asprintf(&info, "To Radio %d", radio->radio_num);
        linklayer_print_packet(info, packet);
        free((void*) info);

    // ESP_LOGI(TAG, "%s: sending packet to radio %d", __func__, radio->radio_num);

        if (os_put_queue(radio->transmit_queue, packet)) {
            /* See if the queue was empty */
            if (os_items_in_queue(radio->transmit_queue) == 1) {
                /* Start the transmission */
                radio->transmit_packet(radio, packet);
            }
        }
    }

    release_packet(packet);
}

bool linklayer_register_protocol(int protocol, void (*protocol_processor)(packet_t*), const char* (*protocol_format)(const packet_t*))
{
    bool ok = true;

    if (protocol >= 0 && protocol < ELEMENTS_OF(protocol_table)) {
        if (protocol_table[protocol].process == NULL) {
            protocol_table[protocol].process = protocol_processor;
            protocol_table[protocol].format = protocol_format;
            ok = true;
        }
    }

    return ok;
}

bool linklayer_unregister_protocol(int protocol)
{
    bool ok = true;

    if (protocol >= DATA_PROTOCOL && protocol < ELEMENTS_OF(protocol_table)) {
        if (protocol_table[protocol].process != NULL) {
            protocol_table[protocol].process = NULL;
            protocol_table[protocol].format = NULL;
            ok = true;
        }
    }

    return ok;
}

int linklayer_get_node_address(void)
{
    return node_address;
}

static bool put_received_packet(packet_t* packet)
{
    return os_put_queue(receive_queue, packet);
}


static bool linklayer_attach_interrupt(radio_t* radio, int dio, GPIO_INT_TYPE edge, void (*handler)(void* p))
{
    bool ok = false;

    if (dio >= 0 && dio < ELEMENTS_OF(radio_config[radio->radio_num].dios)) {
        int gpio = radio_config[radio->radio_num].dios[dio];

        ok = os_attach_gpio_interrupt(gpio, edge, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, handler, (void*) radio);
    } else {
        ESP_LOGE(TAG, "%s: dio out of range: %d on radio %d", __func__, dio, radio->radio_num);
    }

    return ok;
}

/*
 * linklayer_on_receive
 *
 * Called by thread in driver.
 *
 * A packet arrives here.  It will have been wrapped apporpriately
 * and will contain the RSSI, the crc_ok flag and the radio number
 * that received it.  The packet will have a single ref so releasing
 * it will free it.  The radio forwarding the packet will be also
 * be provided.
 *
 * Entry:
 *      radio               Pointer to radio delivering the packet
 *      packet              The data packet
 */
static void linklayer_on_receive(radio_t* radio, packet_t* packet)
{
    linklayer_print_packet("on_receive", packet);

    if (packet != NULL) {
        packet_received += 1;

        if (packet->crc_ok) {

            int routeto = get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN);

            if (routeto == BROADCAST_ADDRESS || routeto == node_address) {
                if (packet->crc_ok) {
                    ++packet_processed;

                    if (debug_flag) {
                        linklayer_print_packet("Received", packet);
                    }

                    /* In promiscuous mode, deliver to receiver so it can handle it (but not process it) */
                    if (promiscuous_queue != NULL) {
                        os_put_queue(promiscuous_queue, duplicate_packet(packet));
                    }

#ifdef CONFIG_LASTLINK_ENABLE_OPPORTUNISTIC_ROUTES
                    /* We have received a packet from someone.  We can add a provisional
                     * route to the ORIGIN via the SENDER and give it a high metric
                     * and unused sequence number.  (note: SEQ 0 is never used for real
                     * messages.)
                     */
                    route_update(&route_table,
                                 packet->radio_num,
                                 get_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                 get_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN),
                                 NON_EXISTANT_SEQUENCE_NUMBER,
                                 IMPOSSIBLY_HIGH_METRIC,
                                 NO_HOST_FLAGS);
#endif
                    int protocol = get_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN);

                    if (protocol >= 0 && protocol < ELEMENTS_OF(protocol_table)) {
                        if (protocol_table[protocol].process != NULL) {
                            protocol_table[protocol].process(ref_packet(packet));
                        } else {
                            ESP_LOGE(TAG, "%s: protocol not registered: %d", __func__, protocol);
                        }
                    } else {
                        ESP_LOGE(TAG, "%s: bad protocol: %d", __func__, protocol);
                   }
                }
            } else {
                /* It is not processed */
                packet_ignored++;
            }

        } else {
            packet_errors_crc++;
        }

        release_packet(packet);
    }
}

/*
 * linklayer_on_transmit.
 *
 * Called from the radio driver when it is ready to transmit another packet.
 *
 * Returns next packet to transmit or NULL if nothing available.
 */
static packet_t* linklayer_on_transmit(radio_t* radio)
{
    /* Pull packet from transmit queue and discard */
    packet_t* packet = NULL;

ESP_LOGI(TAG, "%s: getting last packet", __func__);

    if (os_get_queue_with_timeout(radio->transmit_queue, (os_queue_item_t*) &packet, 0)) {
ESP_LOGI(TAG, "%s: releasing packet %p", __func__, packet);
        release_packet(packet);

        /* Peek next packet to send */
        if (!os_peek_queue(radio->transmit_queue, (os_queue_item_t*) &packet)) {
            packet = NULL;
        }
    }

ESP_LOGI(TAG, "%s: returning next packet %p", __func__, packet);
    /* Return next packet to send or NULL if no more. */
    return packet;
}

void linklayer_print_packet(const char* reason, packet_t* packet)
{
    if (packet != NULL) {
        int dest   = get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN);
        int origin   = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
        int routeto  = get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN);
        int sender = get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN);
        int protocol = get_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN);
        int ttl      = get_uint_field(packet, HEADER_TTL, TTL_LEN);

        //ESP_LOGI(TAG, "%s: D=%04x O=%04x R=%04x S=%04x TTL=%d Proto=%d Len=%d Ref=%d Radio=%d", reason, dest, origin, routeto, sender, ttl, protocol, packet->length, packet->ref, packet->radio_num);

        const char* info = linklayer_packet_format(packet, protocol);

        ESP_LOGI(TAG, "%s: D=%04x O=%04x R=%04x S=%04x TTL=%d Proto=%d Ref=%d Radio=%d: %s", reason, dest, origin, routeto, sender, ttl, protocol, packet->ref, packet->radio_num, info ? info : "");

        free((void*) info);
    } else {
        ESP_LOGI(TAG, "%s: NULL packet", __func__);
    } 
}

static const  char* linklayer_packet_format(const packet_t* packet, int protocol)
{
    if (protocol >= 0 && protocol < ELEMENTS_OF(protocol_table) && protocol_table[protocol].format != NULL) {
        return protocol_table[protocol].format(packet);
    } else {
        return default_packet_format(packet);
    }
}

