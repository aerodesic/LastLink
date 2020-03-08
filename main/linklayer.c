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
#include "os_freertos.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_log.h"

//#include "lwip/err.h"
//#include "lwip/sys.h"

#include "packets.h"
#include "routes.h"
#include "radio.h"
#include "linklayer.h"
#include "linklayer_io.h"

#define TAG  "linklayer"

static void beacon_packet_process(packet_t* p);
static packet_t* routeannounce_packet_create(int target, int sequence, int metric);
static void routeannounce_packet_process(packet_t* p);
static packet_t* routerequest_packet_create(int address);
static void routerequest_packet_process(packet_t* p);
static packet_t* routeerror_packet_create(int target, int address, int sequence, const char* reason);
static void routeerror_packet_process(packet_t* p);
static void data_packet_process(packet_t* p);
static void put_received_packet(packet_t* p);

static bool linklayer_init_radio(radio_t* radio);
static bool linklayer_deinit_radio(radio_t*);

/* Calls from radio driver to linklayer */
static int linklayer_read_register(radio_t* radio, int reg);
static void linklayer_write_register(radio_t* radio, int reg, int value);
static bool linklayer_attach_interrupt(radio_t* radio, int dio, dio_edge_t edge, void (*handler)(void* arg));
static void linklayer_on_receive(radio_t* radio, packet_t* packet);
static packet_t* linklayer_on_transmit(radio_t* radio, packet_t* packet);
static void linklayer_write_buffer(radio_t* radio, int reg, uint8_t* buffer, int length);
static int linklayer_read_buffer(radio_t* radio, int reg, uint8_t* buffer, int bufsize);

static os_mutex_t                 linklayer_lock;
static int                        node_address;
static int                        node_flags;
static os_queue_t                 transmit_queue;
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

typedef void (protocol_process_t)(packet_t*);
static protocol_process_t*        protocol_table[CONFIG_LASTLINK_MAX_PROTOCOL_NUMBER + 1];
                            

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
    .spi_pre_xfer_callback = NULL,                                               \
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
    .ser_name   = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, SER_SCK),    \
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

/*
 * Create a generic packet
 *
 * Entry:
 *      target          Target address
 *      protocol        Protocol number
 *      length          Payload length
 */
packet_t* create_generic_packet(int target, int protocol, int length)
{
    packet_t* p = allocate_packet();
    if (p != NULL) {
         /* Set to header length plus payload required */
         p->length = HEADER_LEN + length;
         set_uint_field(p, HEADER_TARGET_ADDRESS, ADDRESS_LEN, target);
         set_uint_field(p, HEADER_SOURCE_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
         set_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
         set_uint_field(p, HEADER_PREVIOUS_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
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

        ESP_LOGD(TAG, "Beacon: target %d source %d nexthop %d previous %d ttl %d name '%s'",
                 get_uint_field(p, HEADER_TARGET_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_SOURCE_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_PREVIOUS_ADDRESS, ADDRESS_LEN),
                 get_int_field(p, HEADER_TTL, TTL_LEN),
                 name);

        free((void*) name);
        release_packet(p);
    }
}


 
/*
 * RouteAnnounce is used to advertise a route to a node, usually a gateway node.
 *
 * When sent to a specific node, it announces an established route.
 * When sent as broadcast, it helps establish routes (and gateway status)
 * and is rebroadcast so adjacent nodes will see it.
 */
static packet_t* routeannounce_packet_create(int target, int sequence, int metric)
{
    packet_t* p = create_generic_packet(target, RANN_PROTOCOL, RANN_LEN);
    if (p != NULL) {
        set_uint_field(p, RANN_FLAGS, FLAGS_LEN, node_flags);
        set_uint_field(p, RANN_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
        set_uint_field(p, RANN_METRIC, METRIC_LEN, metric);
        p->length = RANN_LEN;
    }

    return p;
}

static void routeannounce_packet_process(packet_t* p)
{
    if (p != NULL) {
        if (os_acquire_recursive_mutex(route_table.lock)) {
            route_t* route = route_update(&route_table,
                                          get_uint_field(p, HEADER_SOURCE_ADDRESS, ADDRESS_LEN),
                                          get_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN),
                                          get_uint_field(p, RANN_SEQUENCE, SEQUENCE_NUMBER_LEN),
                                          get_uint_field(p, RANN_METRIC, METRIC_LEN),
                                          get_uint_field(p, RANN_FLAGS, FLAGS_LEN));

            if (route != NULL) {
                /* If this packet was for us, then release any pending route packets to queue now */
                if (get_uint_field(p, HEADER_TARGET_ADDRESS, ADDRESS_LEN) == node_address) {
                    route_release_packets(route);
                } else {
                    /* Not for us, so clear nexthop so a re-route will be done */
                    set_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
                    /* Update the metric */
                    set_uint_field(p, RANN_METRIC, METRIC_LEN, get_uint_field(p, RANN_METRIC, METRIC_LEN));
                    linklayer_send_packet_update_ttl(p);
               }
            }
            os_release_recursive_mutex(route_table.lock);
            release_packet(p);
        }
    }
}

/*
 * A route request is sent by a node when it needs to know a route to a specific target
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
        set_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        set_uint_field(p, RREQ_FLAGS, FLAGS_LEN, node_flags);
        set_uint_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());
        set_uint_field(p, RREQ_METRIC, METRIC_LEN, 0);
        p->length = RREQ_LEN;
    }

    return p;
}

static void routerequest_packet_process(packet_t* p)
{
    if (p != NULL) {
        if (xSemaphoreTakeRecursive(linklayer_lock, 0) == pdTRUE) {;

            /* TODO: Need brakes to avoid transmitting too many at once !! (Maybe ok for testing) */

            /* Update the route table and see if we have a route */
            route_t* route = route_update(
                                 &route_table,
                                 get_uint_field(p, HEADER_SOURCE_ADDRESS, ADDRESS_LEN),
                                 get_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN),
                                 get_uint_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN),
                                 get_uint_field(p, RREQ_METRIC, METRIC_LEN),
                                 get_uint_field(p, RREQ_FLAGS, FLAGS_LEN));

            /* If packet is targeted for our node, process it */
            if (get_uint_field(p, HEADER_TARGET_ADDRESS, ADDRESS_LEN) == node_address) {
                linklayer_send_packet(routeannounce_packet_create(
                                 get_uint_field(p, HEADER_SOURCE_ADDRESS, ADDRESS_LEN),
                                 get_int_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN),
                                 get_uint_field(p, RREQ_METRIC, METRIC_LEN)));

          /* Else if it's a broadcast and we haven't seen it before, rebroadcast it to next hop */
            } else if (get_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN) == BROADCAST_ADDRESS && route != NULL) {
                set_uint_field(p, RREQ_METRIC, METRIC_LEN, get_uint_field(p, RREQ_METRIC, METRIC_LEN));
                linklayer_send_packet_update_ttl(p);
            }
        }

        xSemaphoreGiveRecursive(linklayer_lock);
        release_packet(p);
    }
}

/*
 * A RouteError is returned to the source for any packet that cannot be delivered.
 */
static packet_t* routeerror_packet_create(int target, int address, int sequence, const char* reason)
{
    packet_t* p = create_generic_packet(target, RERR_PROTOCOL, RERR_LEN);
    if (p != NULL) {
        set_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        set_uint_field(p, RERR_ADDRESS, ADDRESS_LEN, address);
        set_uint_field(p, RERR_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
        set_str_field(p, RERR_REASON, REASON_LEN, reason);
    }

    return p;
}

static void routeerror_packet_process(packet_t* p)
{
    if (p != NULL) {
        const char* reason = get_str_field(p, RERR_REASON, REASON_LEN);

        ESP_LOGD(TAG, "RouteError: target %d source %d nexthop %d previous %d ttl %d address %d sequence %d reason '%s'",
                 get_uint_field(p, HEADER_TARGET_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_SOURCE_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, HEADER_PREVIOUS_ADDRESS, ADDRESS_LEN),
                 get_int_field(p, HEADER_TTL, TTL_LEN),
                 get_uint_field(p, RERR_ADDRESS, ADDRESS_LEN),
                 get_uint_field(p, RERR_SEQUENCE, SEQUENCE_NUMBER_LEN), reason);

        free((void*) reason);
        release_packet(p);
    }
}

/*
 * A Data packet is used to convey any data between nodes.  This packet
 * will be overloaded for any purpose that is needed.
 */
packet_t* data_packet_create(int target, int protocol, uint8_t* data, int length)
{
    packet_t* p;

    if (DATA_PAYLOAD + length <= DATA_LEN) {
        p = create_generic_packet(target, protocol, length);

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
    if (os_acquire_recursive_mutex(linklayer_lock)) {

        if (get_uint_field(p, HEADER_TARGET_ADDRESS, ADDRESS_LEN) == node_address) {
            put_received_packet(p);
        } else {
            set_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
            linklayer_send_packet_update_ttl(p);
        }
        release_packet(p);
        os_release_recursive_mutex(linklayer_lock);
    }
}

/* Creates the linklayer */
bool linklayer_init(int address, int flags, int announce_interval)
{
    linklayer_lock = os_create_recursive_mutex();
    node_address = address;
    node_flags = flags; 
    transmit_queue = os_create_queue(NUM_PACKETS, sizeof(packet_t));
    receive_queue = os_create_queue(NUM_PACKETS, sizeof(packet_t));
    promiscuous_queue = NULL;
    sequence_number = 0;
    debug_flag = false;
    announce_interval = announce_interval;

    if (announce_interval != 0) {
        /* Start the route announce thread */
    }

    route_table_init(&route_table);
    packet_errors_crc = 0;
    packet_processed = 0;
    packet_received = 0;
    packet_transmitted = 0;
    packet_ignored = 0;

    bool ok = true;

    /* Reserve first packet types */
    if (linklayer_reserve_protocol(BEACON_PROTOCOL, beacon_packet_process)      &&
        linklayer_reserve_protocol(RREQ_PROTOCOL, routerequest_packet_process)  &&
        linklayer_reserve_protocol(RANN_PROTOCOL, routeannounce_packet_process) &&
        linklayer_reserve_protocol(RERR_PROTOCOL, routeerror_packet_process)    &&
        linklayer_reserve_protocol(DATA_PROTOCOL, data_packet_process)) {

        for (int radio_num = 0; radio_num < ELEMENTS_OF(radio_config); ++radio_num) {
            ok = ok && linklayer_add_radio(radio_num, &radio_config[radio_num]);
        }
    } else {
       ok = false;
    }

    return ok;
}

bool linklayer_set_promiscuous_mode(bool mode) {
    bool ok = true;

    if (mode) {
        if (promiscuous_queue == NULL) {
            promiscuous_queue = os_create_queue(NUM_PACKETS, sizeof(packet_t));
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

    if (radio != NULL) {
        memset(radio, 0, sizeof(radio_t));

        radio->radio_num = radio_num;

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
                ok = sx126x_radio(radio);
            }
#endif
#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED
            if (config->radio_type == RADIO_IS_SX127x_DEVICE) {
                ok = sx127x_radio(radio);
            }
#endif
            if (radio->start == NULL || radio->start(radio, config->channel)) {

                /* Put radio in active table. If no radio, create one. */
                if (radio_table == NULL) {

                    /* Create radio table */
                    radio_table = (radio_t**) malloc(sizeof(radio_t*) * ELEMENTS_OF(radio_config));
                }

                if (radio_table != NULL) {
                    memset(radio_table, 0, sizeof(radio_t) * ELEMENTS_OF(radio_config));
                }
            }

            if (radio_table != NULL) {
                radio_table[radio_num] = radio;
            } else {
                ok = false;
            }
        }

        if (!ok) {
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

    if (radio_table != NULL && radio_num >= 0 && radio_num < ELEMENTS_OF(radio_config) && radio_table[radio_num] != NULL) {
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

    return true;
}

static bool linklayer_deinit_radio(radio_t* radio)
{
    radio->attach_interrupt = NULL;
    radio->on_receive = NULL;
    radio->on_transmit = NULL;

    return true;
}

bool linklayer_deinit(void)
{
    if (xSemaphoreTakeRecursive(linklayer_lock, 0) == pdTRUE) {

        if (announce_thread != NULL) {
            /* kill it */
            announce_thread = NULL;
        }


        if (transmit_queue != NULL) {
            os_delete_queue(transmit_queue);
            transmit_queue = NULL;
        }
        if (receive_queue != NULL) {
            os_delete_queue(receive_queue);
            receive_queue = NULL;
        }

        route_table_deinit(&route_table);

        /* Deinitialize and remove all radios */
        for (int radio_num = 0; radio_num < ELEMENTS_OF(radio_config); ++radio_num) {
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

    if (xSemaphoreTakeRecursive(linklayer_lock, 0) == pdTRUE) { 
        sequence = ++sequence_number;
        xSemaphoreGiveRecursive(linklayer_lock);
    }

    return sequence;
}

void linklayer_send_packet(packet_t* packet)
{
    /* work */
}

void linklayer_send_packet_update_ttl(packet_t* packet)
{
    int ttl = get_uint_field(packet, HEADER_TTL, TTL_LEN);
    if (--ttl > 0) {
        set_uint_field(packet, HEADER_TTL, TTL_LEN, ttl);
        linklayer_send_packet(packet);
    } else {
        release_packet(packet);
    }
}

void linklayer_process_packet(packet_t* packet)
{
    int protocol = get_int_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN);

    if (protocol >= 0 && protocol <= CONFIG_LASTLINK_MAX_PROTOCOL_NUMBER) {
        if (protocol_table[protocol] != NULL) {
            protocol_table[protocol](packet);
        }
    }

    release_packet(packet);
}

bool linklayer_reserve_protocol(int protocol, protocol_process_t* protocol_processor )
{
    bool ok = true;

    if (protocol >= DATA_PROTOCOL && protocol <= CONFIG_LASTLINK_MAX_PROTOCOL_NUMBER) {
        if (protocol_table[protocol] == NULL) {
            protocol_table[protocol] = protocol_processor;
            ok = true;
        }
    }

    return ok;
}

bool linklayer_release_protocol(int protocol)
{
    bool ok = true;

    if (protocol >= DATA_PROTOCOL && protocol <= CONFIG_LASTLINK_MAX_PROTOCOL_NUMBER) {
        if (protocol_table[protocol] != NULL) {
            protocol_table[protocol] = NULL;
            ok = true;
        }
    }

    return ok;
}

int linklayer_get_node_address(void)
{
    return node_address;
}

static void put_received_packet(packet_t* packet)
{
    /*stub*/
}

static bool linklayer_attach_interrupt(radio_t* radio, int dio, dio_edge_t edge, void (*handler)(void* arg))
{
    return false;
}

/*
 * A packet arrives here.  Wrap it with a packet and procecss it.
 */
static void linklayer_on_receive(radio_t* radio, packet_t* packet)
{
    if (packet != NULL) {
        packet_received += 1;

        if (packet->crc_ok) {

            int nexthop = get_uint_field(packet, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN);

            if (debug_flag) {
                linklayer_print_packet(packet);
            }

            /* In promiscuous, deliver to receiver so it can handle it (but not process it) */
            if (promiscuous_queue != NULL) {
                os_put_queue(promiscuous_queue, ref_packet(packet));
            }

            if (nexthop == BROADCAST_ADDRESS || nexthop == node_address) {
                linklayer_process_packet(ref_packet(packet));
                packet_processed++;
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
 * linklayer_on_transmit
 *
 * Called from the radio driver when it receives an end-of-transmit interrupt.
 *
 * Returns next packet to transmit.
 */
static packet_t* linklayer_on_transmit(radio_t* radio, packet_t* packet)
{
    return NULL;
}

void linklayer_print_packet(packet_t* packet)
{
    /* STUB */
}
