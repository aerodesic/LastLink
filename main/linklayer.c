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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_log.h"

//#include "lwip/err.h"
//#include "lwip/sys.h"

#include "packets.h"
#include "routes.h"

#include "radio.h"
#include "linklayer.h"

#define TAG  "linklayer"

static void beacon_packet_process(packet_t* p);
static packet_t* routeannounce_packet_create(int target, int sequence, int metric);
static void routeannounce_packet_process(packet_t* p);
static packet_t* routerequest_packet_create(int address);
static void routerequest_packet_process(packet_t* p);
static packet_t* routeerror_packet_create(int target, int address, int sequence, const char* reason);
static void routeerror_packet_process(packet_t* p);
static void data_packet_process(packet_t* p);
static int allocate_sequence(void);
static void put_received_packet(packet_t* p);

/* Calls from radio driver to linklayer */
static int linklayer_read_register(radio_t* radio, int reg);
static void linklayer_write_register(radio_t* radio, int reg, int value);
static bool linklayer_attach_interrupt(radio_t* radio, int dio, dio_edge_t edge, void (*handler)(void* arg));
static packet_t* linklayer_on_receive(radio_t* radio, packet_t* packet, bool crc_ok, int rssi);
static packet_t* linklayer_on_transmit(radio_t* radio, packet_t* packet);
static void linklayer_write_buffer(radio_t* radio, int reg, uint8_t* buffer, int length);
static int linklayer_read_buffer(radio_t* radio, int reg, uint8_t* buffer, int bufsize);

static SemaphoreHandle_t    linklayer_lock;
static int                  node_address;
static int                  node_flags;
static QueueHandle_t        transmit_queue;
static QueueHandle_t        receive_queue;
static int                  sequence_number;
static bool                 promiscuous_flag;
static bool                 debug_flag;
static TaskHandle_t         announce_thread;
static int                  announce_interval;
static route_table_t        route_table;
static int                  packet_errors_crc;
static int                  packet_received;
static int                  packet_transmitted;
static int                  packet_ignored;
static radio_t*             radio;
                            

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

typedef struct radio_config {
    const char* type;
    int gpios[MAX_RADIO_GPIO];
} radio_config_t;

#define RADIO_CONFIG_EXPAND(name_begin, radio, name_end) name_begin##_##radio##_##name_end

#define RADIO_CONFIG_SPI(radio, module) \
    { \
    .type = "spi", \
    .gpios[SPI_GPIO_DIO0]  = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_DIO0),  \
    .gpios[SPI_GPIO_DIO1]  = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_DIO1),  \
    .gpios[SPI_GPIO_DIO2]  = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_DIO2),  \
    .gpios[SPI_GPIO_SCK]   = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_SCK),   \
    .gpios[SPI_GPIO_MOSI]  = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_MOSI),  \
    .gpios[SPI_GPIO_MISO]  = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_MISO),  \
    .gpios[SPI_GPIO_SS]    = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_SS),    \
    .gpios[SPI_GPIO_RESET] = RADIO_CONFIG_EXPAND(CONFIG_LASTLINK_RADIO, radio, GPIO_RESET), \
   },

static radio_config_t radio_config[] = {

#if defined(CONFIG_LASTLINK_RADIO_1_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_1_SX126x_SPI)
    RADIO_CONFIG_SPI(1, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_1_SX127x_SPI)
    RADIO_CONFIG_SPI(1, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_2_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_2_SX127x_SPI)
    RADIO_CONFIG_SPI(2, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_2_SX127x_SPI)
    RADIO_CONFIG_SPI(2, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_3_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_3_SX127x_SPI)
    RADIO_CONFIG_SPI(3, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_3_SX127x_SPI)
    RADIO_CONFIG_SPI(3, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_4_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_4_SX127x_SPI)
    RADIO_CONFIG_SPI(4, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_4_SX127x_SPI)
    RADIO_CONFIG_SPI(4, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_2_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_5_SX127x_SPI)
    RADIO_CONFIG_SPI(5, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_5_SX127x_SPI)
    RADIO_CONFIG_SPI(5, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_6_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_6_SX127x_SPI)
    RADIO_CONFIG_SPI(6, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_6_SX127x_SPI)
    RADIO_CONFIG_SPI(6, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_7_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_7_SX127x_SPI)
    RADIO_CONFIG_SPI(7, SX126x)
  #elif defined(CONFIG_LASTLINK_RADIO_7_SX127x_SPI)
    RADIO_CONFIG_SPI(7, SX127x)
  #endif
#endif
#if defined(CONFIG_LASTLINK_RADIO_8_ENABLED)
  #if defined(CONFIG_LASTLINK_RADIO_8_SX127x_SPI)
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
packet_t* generic_packet_create(int target, int protocol, int length)
{
    packet_t* p = packet_allocate();
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

    packet_t* p = generic_packet_create(BROADCAST_ADDRESS, BEACON_PROTOCOL, length);
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
        packet_release(p);
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
    packet_t* p = generic_packet_create(target, RANN_PROTOCOL, RANN_LEN);
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
        xSemaphoreTakeRecursive(&route_table.lock, 0);
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
        xSemaphoreGiveRecursive(route_table.lock);
        packet_release(p);
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
    packet_t* p = generic_packet_create(address, RREQ_PROTOCOL, RREQ_LEN);
    if (p != NULL) {
        set_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        set_uint_field(p, RREQ_FLAGS, FLAGS_LEN, node_flags);
        set_uint_field(p, RREQ_SEQUENCE, SEQUENCE_NUMBER_LEN, allocate_sequence());
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
        packet_release(p);
    }
}

/*
 * A RouteError is returned to the source for any packet that cannot be delivered.
 */
static packet_t* routeerror_packet_create(int target, int address, int sequence, const char* reason)
{
    packet_t* p = generic_packet_create(target, RERR_PROTOCOL, RERR_LEN);
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
        packet_release(p);
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
        p = generic_packet_create(target, protocol, length);

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
    if (xSemaphoreGiveRecursive(linklayer_lock) == pdTRUE) {

        if (get_uint_field(p, HEADER_TARGET_ADDRESS, ADDRESS_LEN) == node_address) {
            put_received_packet(p);
        } else {
            set_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
            linklayer_send_packet_update_ttl(p);
        }
        packet_release(p);
        xSemaphoreGiveRecursive(linklayer_lock);
    }
}

static linklayer_t linklayer = {
    .read_register    = linklayer_read_register,
    .write_register   = linklayer_write_register,
    .attach_interrupt = linklayer_attach_interrupt,
    .on_receive       = linklayer_on_receive,
    .on_transmit      = linklayer_on_transmit,
    .write_buffer     = linklayer_write_buffer,
    .read_buffer      = linklayer_read_buffer,
};

/* Creates the linklayer */
bool linklayer_init(int address, int flags, int announce_interval)
{
    linklayer_lock = xSemaphoreCreateRecursiveMutex(); 
    node_address = address;
    node_flags = flags; 
    transmit_queue = xQueueCreate((UBaseType_t) NUM_PACKETS, (UBaseType_t) sizeof(packet_t));
    receive_queue = xQueueCreate((UBaseType_t) NUM_PACKETS, (UBaseType_t) sizeof(packet_t));
    sequence_number = 0;
    promiscuous_flag = false;
    debug_flag = false;
    announce_interval = announce_interval;

    if (announce_interval != 0) {
    }

    route_table_init(&route_table);
    packet_errors_crc = 0;
    packet_received = 0;
    packet_transmitted = 0;
    packet_ignored = 0;

    radio = NULL;

#if 0
    self._spi = SPI(baudrate=10000000, polarity=0, phase=0, bits=8, firstbit = SPI.MSB,
                    sck = Pin(_SX127x_SCK, Pin.OUT, Pin.PULL_DOWN),
                    mosi = Pin(_SX127x_MOSI, Pin.OUT, Pin.PULL_UP),
                    miso = Pin(_SX127x_MISO, Pin.IN, Pin.PULL_UP))

        self._ss = Pin(_SX127x_SS, Pin.OUT)
        self._reset = Pin(_SX127x_RESET, Pin.OUT)
        self._dio_table = [ Pin(_SX127x_DIO0, Pin.IN), Pin(_SX127x_DIO1, Pin.IN), Pin(_SX127x_DIO2, Pin.IN) ]
        self._ping_count = 0
        self._power = None # not True nor False
#endif
    return true;
}

/*
 * a radio_t is returned by the sx127x_create() or sx126x_create)()
 * Only allows one radio at the moment.
 */
bool linklayer_add_radio(radio_t* radio)
{
    bool ok;

    if (radio == NULL) {
        ok = radio->start(radio, &linklayer);
    } else {
        ok = false;
    }

    return ok;
}

bool linklayer_deinit(void)
{
    if (xSemaphoreTakeRecursive(linklayer_lock, 0) == pdTRUE) {

        if (announce_thread != NULL) {
            /* kill it */
            announce_thread = NULL;
        }


        if (transmit_queue != NULL) {
            vQueueDelete(transmit_queue);
            transmit_queue = NULL;
        }
        if (receive_queue != NULL) {
            vQueueDelete(receive_queue);
            receive_queue = NULL;
        }

        route_table_deinit(&route_table);

        radio->stop(radio);
        radio = NULL;
        xSemaphoreGiveRecursive(linklayer_lock);
        vSemaphoreDelete(linklayer_lock);
        linklayer_lock = NULL;
    }

    return linklayer_lock == NULL;
}

static int allocate_sequence(void)
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
        packet_release(packet);
    }
}

int linklayer_get_node_address(void)
{
    return node_address;
}

static void put_received_packet(packet_t* packet)
{
    /*stub*/
}

static int linklayer_read_register(radio_t* radio, int reg)
{
    return 0;
}

static void linklayer_write_register(radio_t* radio, int reg, int value)
{
}

static bool linklayer_attach_interrupt(radio_t* radio, int dio, dio_edge_t edge, void (*handler)(void* arg))
{
    return false;
}

static packet_t* linklayer_on_receive(radio_t* radio, packet_t* packet, bool crc_ok, int rssi)
{
    return NULL;
}

static packet_t* linklayer_on_transmit(radio_t* radio, packet_t* packet)
{
    return NULL;
}

static void linklayer_write_buffer(radio_t* radio, int reg, uint8_t* buffer, int length)
{
}

static int linklayer_read_buffer(radio_t* radio, int reg, uint8_t* buffer, int bufsize)
{
    return 0;
}
