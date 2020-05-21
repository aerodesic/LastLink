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
#include "duplicate_packets.h"

#if CONFIG_LASTLINK_TABLE_COMMANDS
#include "commands.h"
#endif

#if CONFIG_LASTLINK_ENABLE_SOCKET_LAYER
#include "lsocket.h"
#endif

#ifdef CONFIG_LASTLINK_RADIO_SX126x_ENABLED
#include "sx126x_driver.h"
#endif
#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED
#include "sx127x_driver.h"
#endif

#define TAG  "linklayer"

static bool beacon_packet_process(packet_t* packet);
static const char* beacon_packet_format(const packet_t* packet);

static bool routeannounce_packet_process(packet_t* packet);
static const char* routeannounce_packet_format(const packet_t* packet);

static bool routerequest_packet_process(packet_t* packet);
static const char* routerequest_packet_format(const packet_t* packet);

static bool routeerror_packet_process(packet_t* packet);
static const char* routeerror_packet_format(const packet_t* packet);

static char tohex(int v);
static const char* default_packet_format(const packet_t* packet);

static bool linklayer_init_radio(radio_t* radio);
static bool linklayer_deinit_radio(radio_t*);
static void linklayer_transmit_packet(radio_t* radio, packet_t* packet);

#ifdef NOTUSED
static void linklayer_transmit_packet_delayed(os_timer_t timer);
/* item sent to delay start through timer to actually launch the packet */
typedef struct {
    radio_t *radio;
    packet_t *packet;
} start_delay_param_t;
#endif

static const  char* linklayer_packet_format(const packet_t* packet, int protocol);

/* Calls from radio driver to linklayer */
static bool linklayer_attach_interrupt(radio_t* radio, int dio, GPIO_INT_TYPE edge, void (*handler)(void* p));
static void linklayer_on_receive(radio_t* radio, packet_t* packet);
static void linklayer_activity_indicator(radio_t* radio, bool active);

static void reset_device(radio_t* radio);

int                               linklayer_node_address;
static os_mutex_t                 linklayer_mutex;
static int                        node_flags;
static os_queue_t                 receive_queue;
static os_queue_t                 promiscuous_queue;
static uint16_t                   sequence_number;
static bool                       debug_flag;
static bool                       listen_only;
static os_thread_t                announce_thread;
static int                        announce_interval;
static int                        packet_errors_crc;   /* Packets with crc errors */
static int                        packet_received;     /* Total packets received */
static int                        packet_processed;    /* Packets accepted for processing */
static int                        packet_transmitted;  /* Total packets transmitted */
static int                        packet_ignored;      /* Packets not processed */
#if CONFIG_LASTLINK_RECEIVE_ONLY_FROM_TABLE
/* If table is defined, allow packages only from entries in the table */
static int                        receive_only_from[CONFIG_LASTLINK_RECEIVE_ONLY_FROM_TABLE];
#endif
static radio_t**                  radio_table;
static int                        activity_count = -1;

typedef struct protocol_entry {
    bool        (*process)(packet_t*);                 /* Process function; return true if processed */
    const char* (*format)(const  packet_t*);           /* Format for printing */
} protocol_entry_t;

static protocol_entry_t           protocol_table[CONFIG_LASTLINK_MAX_PROTOCOL_NUMBER + 1];

/* Duplicate packet detector table */
duplicate_packet_list_t           duplicate_packets;

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

bool linklayer_lock(void)
{
    return os_acquire_recursive_mutex(linklayer_mutex);
}

bool linklayer_unlock(void)
{
    return os_release_recursive_mutex(linklayer_mutex);
}

/* Return true if the packet is intended specifically for us */
bool linklayer_packet_is_for_this_node(const packet_t* p)
{
    return get_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN) == linklayer_node_address;
}


#ifdef CONFIG_LASTLINK_RECEIVE_ONLY_FROM_TABLE
/*
 * Assign receive_only addresses.  If the first slot is non-zero, only permit incoming
 * packets from the addresses in the table.
 */
void linklayer_set_receive_only_from(const char* addresses)
{
    ESP_LOGD(TAG, "%s: setting to %s", __func__, addresses);

    const char *p = addresses;
    
    for (size_t slot = 0; slot < ELEMENTS_OF(receive_only_from) && p != NULL; ++slot) {
        receive_only_from[slot] = strtol(p, NULL, 0);
        p = (const char*) strchr(p, ',');
        if (p != NULL) {
            ++p;
        }
    }

    for (size_t slot = 0; slot < ELEMENTS_OF(receive_only_from) && receive_only_from[slot] != 0; ++slot) {
        ESP_LOGD(TAG, "%s: receive_only_from[%d] is %d", __func__, slot, receive_only_from[slot]);
    }
}

static bool is_valid_address(int address)
{
    bool ok = true;

    if (address != linklayer_node_address && receive_only_from[0] != 0) {
        ok = false;
        for (size_t slot = 0; slot < ELEMENTS_OF(receive_only_from) && !ok; ++slot) {
            /* First one found triggers OK */
            ok = address == receive_only_from[slot];
        }
        if (! ok) {
            ESP_LOGD(TAG, "%s: Ignoring packet from %d", __func__, address);
        }
    }

    return ok;
}
#else
#define is_valid_address(addr)   (true)
#endif
       
/*
 * Packets for this node (not internal packets checked elsewhere) is a packet
 * with a valid address (not in exclude list) and not from us (echoed packets)
 * and destination is either this node or a broadcast.
 */
static inline bool is_packet_for_this_node(const packet_t* packet)
{
    int sender = get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN);
    int origin = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
    int dest   = get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN);

    return is_valid_address(sender) && origin != linklayer_node_address && (dest == linklayer_node_address || dest == BROADCAST_ADDRESS);
}

static inline bool is_routed_through(const packet_t* packet)
{
    return get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN) == linklayer_node_address ||
           get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN) == BROADCAST_ADDRESS;
}

static inline bool is_internal_packet(const packet_t* packet)
{
    return get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN) == linklayer_node_address &&
           get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN) == linklayer_node_address;
}




/*
 * Create a generic packet
 *
 * Entry:
 *      dest          Target address
 *      protocol        Protocol number
 *      length          Payload length
 */
packet_t* linklayer_create_generic_packet(int dest, int protocol, int length)
{
    packet_t* p = allocate_packet();
    if (p != NULL) {
         p->crc_ok = true;

         /* Set to header length plus payload required */
         p->length = HEADER_LEN + length;
         set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
         set_uint_field(p, HEADER_FLAGS, FLAGS_LEN, 0);
         /* Origin is always our node address */
         set_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
         set_uint_field(p, HEADER_DEST_ADDRESS, ADDRESS_LEN, dest);
         set_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN, linklayer_node_address);
         // Sequence numbers are created whenever a packet is sent from this node.
         set_uint_field(p, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());
         set_uint_field(p, HEADER_PROTOCOL, PROTOCOL_LEN, protocol);
         set_uint_field(p, HEADER_METRIC, METRIC_LEN, 0);
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

    packet_t* p = linklayer_create_generic_packet(BROADCAST_ADDRESS, BEACON_PROTOCOL, length);
    if (p != NULL) {
        set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        set_int_field(p, HEADER_METRIC, METRIC_LEN, MAX_METRIC); /* Large metric so it doesn't get retransmitted */
        int moved = set_str_field(p, BEACON_NAME, length, name);
        /* Update packet length */
        p->length = HEADER_LEN + moved;
    }
    return p;
}

/*
 * All process functions must release the packet when complete and ref_packet before passing it on.
 */
static bool beacon_packet_process(packet_t* packet)
{
    bool consumed = false;

    if (packet != NULL) {
        const char* name = get_str_field(packet, BEACON_NAME, BEACON_NAME_LEN);

        ESP_LOGD(TAG, "Beacon: dest %d origin %d routeto %d sender %d metric %d name '%s'",
                 get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN),
                 get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                 get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN),
                 get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN),
                 get_int_field(packet, HEADER_METRIC, METRIC_LEN),
                 name);

        free((void*) name);

        consumed = true;

        release_packet(packet);
    }

    return consumed;
}

static const char* beacon_packet_format(const packet_t* packet)
{
    char* info;

    const char* name = get_str_field(packet, BEACON_NAME, BEACON_NAME_LEN);

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
packet_t* routeannounce_packet_create(int dest)
{
    packet_t* packet = linklayer_create_generic_packet(dest, ROUTEANNOUNCE_PROTOCOL, ROUTEANNOUNCE_LEN);
    if (packet != NULL) {
        set_uint_field(packet, ROUTEANNOUNCE_FLAGS, FLAGS_LEN, node_flags);
        //set_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
        //set_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN, linklayer_node_address);
    }

    return packet;
}

static bool routeannounce_packet_process(packet_t* packet)
{
    bool consumed = false;

    if (packet != NULL) {
        //linklayer_print_packet("Route Announce", packet);
        if (route_table_lock()) {
            if (linklayer_packet_is_for_this_node(packet)) {
                consumed = true;

                /* See if this supplied a route to the ORIGIN */
                route_t* route = find_route(get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN));

                if (route != NULL) {
                    /* If this packet was for us, then release any pending route packets to queue now */
                    /* We now have a route, so release all packets waiting to go on this route. */
                    route_release_packets(route);
                }
            }

            route_table_unlock();

        } else {
            /* Unable to get mutex */
        }

        release_packet(packet);
    }

    return consumed;
}

static const char* routeannounce_packet_format(const packet_t* packet)
{
    char* info;

    int flags = get_uint_field(packet, ROUTEANNOUNCE_FLAGS, FLAGS_LEN);

    asprintf(&info, "Route Announce: flags %02x", flags);

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
packet_t* routerequest_packet_create(int address)
{
    packet_t* p = linklayer_create_generic_packet(address, ROUTEREQUEST_PROTOCOL, ROUTEREQUEST_LEN);

    if (p != NULL) {
        set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        //set_uint_field(p, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
        //set_uint_field(p, HEADER_SENDER_ADDRESS, ADDRESS_LEN, linklayer_node_address);
        set_uint_field(p, ROUTEREQUEST_FLAGS, FLAGS_LEN, node_flags);
    }

    return p;
}

static bool routerequest_packet_process(packet_t* packet)
{
    bool consumed = false;

    if (packet != NULL) {
        //linklayer_print_packet("Route Request", p);

        if (linklayer_lock()) {

            /* TODO: Need may brakes to avoid transmitting too many at once !! (Maybe ok for testing) */
            
            /* If packet is dest for our node, announce route back to origin with same seqeuence and new metric. */
            if (linklayer_packet_is_for_this_node(packet)) {
                packet_t* ra = routeannounce_packet_create(get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN));

                /* Label it as going to specific radio of origin */
                ra->radio_num = packet->radio_num;
                linklayer_send_packet(ra);

                consumed = true;

            } else {
                /* By not 'consuming' the packet, this packet will be resent with an updated metric */
            }

            linklayer_unlock();

        } else {
            /* Cannot get mutex */
        }

        release_packet(packet);
    }

    return consumed;
}

static const char* routerequest_packet_format(const packet_t* packet)
{
    char* info;

    int flags = get_uint_field(packet, ROUTEREQUEST_FLAGS, FLAGS_LEN);

    asprintf(&info, "Route Request: flags %02x", flags);

    return info;
}

/*
 * A RouteError is returned to the origin for any packet that cannot be delivered.
 */
packet_t* routeerror_packet_create(int dest, int address, const char* reason)
{
    packet_t* p = linklayer_create_generic_packet(dest, ROUTEERROR_PROTOCOL, ROUTEERROR_LEN);
    if (p != NULL) {
        set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, dest);
        set_uint_field(p, ROUTEERROR_ADDRESS, ADDRESS_LEN, address);
        set_str_field(p, ROUTEERROR_REASON, REASON_LEN, reason);
    }

    return p;
}

static bool routeerror_packet_process(packet_t* packet)
{
    bool consumed = false;

    if (packet != NULL) {

        if (linklayer_lock()) {

            if (linklayer_packet_is_for_this_node(packet)) {
                /* Get address from packet */
                int address = get_uint_field(packet, ROUTEERROR_ADDRESS, ADDRESS_LEN);
                if (address >= 0) {
                    /* Delete route associated with this address */
                    route_remove(address);
                }

                consumed = true;
            }

            linklayer_unlock();

        } else {
            /* Cannot get mutex */
        }
        release_packet(packet);
    }

    return consumed;
}

static const char* routeerror_packet_format(const packet_t* packet)
{
    char* info;

    int address = get_uint_field(packet, ROUTEERROR_ADDRESS, ADDRESS_LEN);
    const char* reason = get_str_field(packet, ROUTEERROR_REASON, REASON_LEN);

    asprintf(&info, "Route Error: address %x \"%s\"", address, reason);

    free((void*) reason);

    return info;
}


/*
 * A Data packet is used to convey any data between nodes.  This packet
 * will be overloaded for any purpose that is needed.
 */
packet_t* data_packet_create(int dest, int protocol, uint8_t* data, int length)
{
    packet_t* p;

    if (DATA_PAYLOAD + length <= DATA_LEN) {
        p = linklayer_create_generic_packet(dest, protocol, length);

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

static char tohex(int v)
{
    v &= 0x0F;
    if (v <= 9) {
        return '0' + v;
    } else {
        return 'A' + v - 10;
    }
}

const char* linklayer_escape_raw_data(const uint8_t* data, size_t length)
{
    /* Determine length of output data field */
    int outlen = 0;

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
        for (int index = 0; index < length; ++index) {
            if (isprint(data[index])) {
                *outp++ = data[index];
            } else if (data[index] == '\n') {
                *outp++ = '\\';
                *outp++ = 'n';
            } else if (data[index] == '\r') {
                *outp++ = '\\';
                *outp++ = 'r';
            } else if (data[index] == '\a') {
                *outp++ = '\\';
                *outp++ = 'a';
            } else if (data[index] == '\t') {
                *outp++ = '\\';
                *outp++ = 't';
            } else if (data[index] == '\f') {
                *outp++ = '\\';
                *outp++ = 'f';
            } else if (data[index] == '\b') {
                *outp++ = '\\';
                *outp++ = 'b';
            } else {
                *outp++ = '\\';
                *outp++ = 'x';
                *outp++ = tohex(data[index] >> 4);
                *outp++ = tohex(data[index]);
            }
        }
        *outp = '\0';
    }

    return outdata;
}

static const char* default_packet_format(const packet_t* packet)
{
    char* info;

    /* Extract from end of header to end of packet */
    const char* data = linklayer_escape_raw_data(packet->buffer + HEADER_LEN, packet->length - HEADER_LEN);

    asprintf(&info, "Data: \"%s\"", data);

    free((void*) data);

    return info;
}

#ifdef NOTUSED
int button_interrupts;
static void test_button_handler(void* param)
{
    ++button_interrupts;
}
#endif

os_queue_t linklayer_set_promiscuous_mode(bool mode) {

    if (mode) {
        if (promiscuous_queue == NULL) {
            promiscuous_queue = os_create_queue(NUM_PACKETS, sizeof(packet_t*));
        }
    } else {
        if (promiscuous_queue != NULL) {
            os_delete_queue(promiscuous_queue);
            promiscuous_queue = NULL;
        }
    }

    return promiscuous_queue;
}

bool linklayer_set_debug(bool enable)
{
    bool old_debug = debug_flag;
    debug_flag = enable;
    return old_debug;
}

/*
 * When set, radio will not answer
 */
bool linklayer_set_listen_only(bool enable)
{
    listen_only = enable;
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

ESP_LOGD(TAG, "%s: radio_num %d allocated radio_t at %p", __func__, radio_num, radio);

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

ESP_LOGD(TAG, "%s: allocated radio_table at %p", __func__, radio_table);
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
#if 0
    radio->on_transmit = linklayer_on_transmit;
#endif
    radio->transmit_queue = os_create_queue(NUM_PACKETS, sizeof(packet_t*));
    radio->activity_indicator = linklayer_activity_indicator;
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
#if 0
           radio->on_transmit = NULL;
#endif
           os_delete_queue(radio->transmit_queue);
           radio->transmit_queue = NULL;
           radio->reset_device = NULL;
        }
    }

    return ok;
}
int linklayer_allocate_sequence(void)
{
    int sequence = 0;

    if (linklayer_lock()) {
        /* Create new sequence number but skip the undefined one */
        do {
            sequence = ++sequence_number;
        } while (sequence == UNDEFINED_SEQUENCE_NUMBER);
        linklayer_unlock();
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
    } else if (listen_only) {
        /* In listen-only mode, just discard sending packets */
        release_packet(packet);
    } else {
        int dest = get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN);

#if 11111111111
        if (dest != BROADCAST_ADDRESS && dest > 6) {
            linklayer_print_packet("BAD DEST", packet);
        }
#endif /* 11111111111 */

        /*
         * A packet with a origin and destination is ready to transmit.
         * Label the from address and if no to address, attempt to route
         */

        /* Indicate coming from us */
        set_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN, linklayer_node_address);

        /* If origin is NULL address, set it to us */
        // if (get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN) == NULL_ADDRESS) {
        //    set_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
        // }

        /* Every outbound packet from this node gets a new sequence number */
        //if (get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN) == linklayer_node_address) {
        //    set_uint_field(packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());
        //}

        /* If packet is destined for local address, side-step and just put into receive queue for radio 0 */
        if (dest == linklayer_node_address) {
             set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, linklayer_node_address);
             /* Tell caller the packet has been routed */
             packet_tell_routed_callback(packet, true);
             linklayer_on_receive(radio_table[0], ref_packet(packet));
        } else {

            /*
             * If the routeto is NULL, then we compute next hop based on route table.
             * If no route table, create pending NULL route and cache packet for later retransmission.
             */
            unsigned int routeto = get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN);

            radio_t* radio = NULL;  /* Gets the target radio device address */

            if (dest == BROADCAST_ADDRESS) {
                /* Route to broadcast as well */
                set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);

            } else if (routeto == NULL_ADDRESS) {

                route_table_lock();

                route_t* route = find_route(dest);

                /* If no route and we are the originator, create a dummy and make a request */
                if (route == NULL && get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN) == linklayer_node_address) {
                    route = create_route(packet->radio_num,
                                         dest,                                                                 /* Want a route to this destination */
                                         MAX_METRIC,                                                           /* Largest possible */
                                         NULL_ADDRESS,                                                         /* No route assigned */
                                         linklayer_node_address,                                               /* We created it */
                                         UNDEFINED_SEQUENCE_NUMBER,                                            /* Sequence number not defined */
                                         NO_HOST_FLAGS);                                                       /* No flags assigned */

                    /* Queue with it's pending ownership */
                    route_put_pending_packet(route, packet);

                    if (debug_flag) {
                        linklayer_print_packet("Routing", packet);
                    }

                    packet = NULL;

                    /* Create a route request to be sent instead */
                    route_start_routerequest(route);
                }

                if (route == NULL) { 
                    /* Don't have a route and it's not ours so drop it */
                    release_packet(packet);
                    packet = NULL;

                } else if (route->routeto == NULL_ADDRESS) {
                    /* Still have pending route, add packet to queue */
                    route_put_pending_packet(route, packet);
                    /* And drop it */
                    packet = NULL;

                } else if (routeto == get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN)) {
                    /* We are trying to send back to sender.  I smell a loop... */
                    /* Destroy the route that gave this to us */
                    route_delete(route);
                    route = NULL;

                    /* Will need to be regenerated by sender */
                    release_packet(packet);
                    packet = NULL;

                } else {
                    /* We have a route so select routeto and radio */
                    set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, route->routeto);
                    if (route->radio_num != UNKNOWN_RADIO) {
                        radio = radio_table[route->radio_num];
                    } else {
                        release_packet(packet);
                        packet = NULL;
                    }

                    packet_tell_routed_callback(packet, true);
                }
                route_table_unlock();
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

                /* In promiscuous mode, deliver to receiver so it can handle it (but not process it) */
                if (promiscuous_queue != NULL) {
                    packet_t *p_packet = duplicate_packet(packet);
                    if (p_packet != NULL) {
                        p_packet->transmitted = true;
                        os_put_queue(promiscuous_queue, p_packet);
                    }
                }

                linklayer_transmit_packet(radio, ref_packet(packet));
            }
        }
    }

    if (packet != NULL) {
        release_packet(packet);
    }
}

void linklayer_send_packet_update_metric(packet_t* packet)
{
    int metric = get_uint_field(packet, HEADER_METRIC, METRIC_LEN);
    if (++metric < MAX_METRIC) {
        set_uint_field(packet, HEADER_METRIC, METRIC_LEN, metric);
        linklayer_send_packet(packet);
    } else {
linklayer_print_packet("METRIC EXPIRED", packet);
        release_packet(packet);
    }
}

/*
 * linklayer_transmit_packet

 * Put the packet onto the radio send queue.  If the queue now contains one item,
 * activate the send by issuing a send_packet to the radio.
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
//ESP_LOGD(TAG, "%s: sending %p on radio %d", __func__, packet, radio_num);
            linklayer_transmit_packet(radio_table[radio_num], ref_packet(packet));
        }
        release_packet(packet);
#ifdef NOTUSED
    } else if (get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN) == BROADCAST_ADDRESS) {
        /* Issue delayed start for broadcast packets */
        start_delay_param_t *sdp = (start_delay_param_t*) malloc(sizeof(start_delay_param_t));
        if (sdp != NULL) {
            sdp->packet = packet;
            sdp->radio = radio;
            int delay = ((esp_random() + linklayer_node_address) % 5) * 20 + 20;
            os_timer_t timer = os_create_timer("delay_packet", delay, sdp, linklayer_transmit_packet_delayed);
            os_start_timer(timer);
        } else {
            release_packet(packet);
            ESP_LOGE(TAG, "%s: unable to allocate delay_param", __func__);
        }
#endif
    } else {
        if (get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN) == BROADCAST_ADDRESS) {
            /* Issue a random delay on each transmit request */
            packet->delay = true;
        }
        if (os_put_queue(radio->transmit_queue, packet)) {
            /* See if the queue was empty */
            if (os_items_in_queue(radio->transmit_queue) == 1) {
                /* Start the transmission */
                radio->transmit_start(radio);
            }
        }
    }
}

#ifdef NOTUSED
static void linklayer_transmit_packet_delayed(os_timer_t timer)
{
    start_delay_param_t *delay_param = (start_delay_param_t*) os_get_timer_data(timer);

    if (os_put_queue(delay_param->radio->transmit_queue, delay_param->packet)) {
        if (os_items_in_queue(delay_param->radio->transmit_queue) == 1) {
            delay_param->radio->transmit_start(delay_param->radio);
        }
    } else {
        ESP_LOGE(TAG, "%s: unable to queue packet", __func__);
        release_packet(delay_param->packet);
    }

    free((void*) delay_param);
    os_delete_timer(timer);
}
#endif

bool linklayer_register_protocol(int protocol, bool (*protocol_processor)(packet_t*), const char* (*protocol_format)(const packet_t*))
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

    if (protocol >= 0 && protocol < ELEMENTS_OF(protocol_table)) {
        if (protocol_table[protocol].process != NULL) {
            protocol_table[protocol].process = NULL;
            protocol_table[protocol].format = NULL;
            ok = true;
        }
    }

    return ok;
}

bool linklayer_put_received_packet(packet_t* packet)
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
    //linklayer_print_packet("on_receive", packet);

    if (packet != NULL) {
        packet_received += 1;

        if (packet->crc_ok) {

            if (listen_only) {
                /* In promiscuous mode, deliver to receiver so it can handle it (but not process it) */
                if (promiscuous_queue != NULL) {
                    os_put_queue(promiscuous_queue, duplicate_packet(packet));
                }
            /* Ignore redundant packets from the same origin and sequence number */
            } else {
                /* In promiscuous mode, deliver to receiver so it can handle it (but not process it) */
                if (promiscuous_queue != NULL) {
                    os_put_queue(promiscuous_queue, duplicate_packet(packet));
                }
    
                /* Check for packets arriving with the same sequence number - ignore them */
                if (is_internal_packet(packet) || !is_duplicate_packet(&duplicate_packets, packet)) {
                    bool consumed = false;
    
                    /* If the packet is for us or routed through us, process it, if it's the first time through */
                    if (is_internal_packet(packet) || is_packet_for_this_node(packet) || is_routed_through(packet)) {
    
                        /* Update route table */
                        update_route(packet->radio_num,
                                     get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),           /* This is the destination */
                                     get_uint_field(packet, HEADER_METRIC, METRIC_LEN) + 1,                /* Metric is 1+ hops */
                                     get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN),           /* Route to this node to send it */
                                     get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),           /* Route provided by this node */
                                     get_uint_field(packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN),  /* Suppliers sequence number */
                                     get_uint_field(packet, HEADER_FLAGS, FLAGS_LEN));                     /* Suppliers flags */
                
                        /* Try to process the packet by the protocol field */
                        int protocol = get_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN);
    
                        /* Is it a valid protocol? */
                        if (protocol >= 0 && protocol < ELEMENTS_OF(protocol_table)) {
                            /* Does it have a process function? */
                            if (protocol_table[protocol].process != NULL) {
                                ++packet_processed;  /* It is processed */
    
                                /* Perform the processing and remember if we did something local */
                                consumed = protocol_table[protocol].process(ref_packet(packet));
    
                            } else {
                                linklayer_print_packet("NOT REGISTERED", packet);
                                //ESP_LOGE(TAG, "%s: protocol not registered: %d", __func__, protocol);
                                consumed = true;
                            }
                        } else {
                            linklayer_print_packet("BAD PROTOCOL", packet);
                            //ESP_LOGE(TAG, "%s: bad protocol: %d", __func__, protocol);
                            consumed = true;
                        }
                    } 

                    /* If the packet was not consumed, see how to forward on */
                    if (!consumed) {
                        int routeto = get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN);
    
                        /* If routed to this node, then send onward by routing or rebroadcasting */
                        if (routeto == linklayer_node_address || routeto == BROADCAST_ADDRESS) {
                            /* re-route if not broadcast */
                            if (routeto != BROADCAST_ADDRESS) {
                                set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
                            }
    
                            linklayer_send_packet_update_metric(ref_packet(packet));
                        }
                    }
                }
            }
        } else {
ESP_LOGI(TAG, "%s: packet crc error; len %d bytes", __func__, packet->length);
            packet_errors_crc++;
        }

        release_packet(packet);
    }
}

#if 0
/*
 * linklayer_on_transmit.
 *
 * Called from the radio driver when it is ready to transmit another packet.
 *
 * Returns next packet to transmit or NULL if nothing available.
 */
static packet_t* linklayer_on_transmit(radio_t* radio, bool first_packet)
{
    /* Pull packet from transmit queue and discard */
    packet_t* packet = NULL;

    if (first_packet) {
        if (!os_peek_queue(radio->transmit_queue, (os_queue_item_t*) &packet)) {
            packet = NULL;
        }

    } else if (os_get_queue_with_timeout(radio->transmit_queue, (os_queue_item_t*) &packet, 0)) {
        release_packet(packet);

        /* Peek next packet to send */
        if (!os_peek_queue(radio->transmit_queue, (os_queue_item_t*) &packet)) {
            packet = NULL;
        }
    }

    /* Return next packet to send or NULL if no more. */
    return packet;
}
#endif

static void linklayer_activity_indicator(radio_t* radio, bool active)
{
//ESP_LOGI(TAG, "%s: active %s", __func__, active ? "TRUE" : "FALSE");

    if (activity_count < 0) {
//ESP_LOGI(TAG, "%s: initialize activity GPIO %d", __func__, CONFIG_LASTLINK_LED_ACTIVITY_GPIO);

        gpio_config_t io = {
            .intr_type = GPIO_PIN_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << CONFIG_LASTLINK_LED_ACTIVITY_GPIO,
            .pull_down_en = 0,
            .pull_up_en = 0,
        };

        gpio_config(&io);

        io.pin_bit_mask = 1ULL << 2; /* Do second port */
        gpio_config(&io);

        activity_count = 0;
    }

    if (active) {
        activity_count++;
    } else if (activity_count != 0) {
        activity_count--;
    } else {
        ESP_LOGE(TAG, "%s: trying to set activity_count < 0", __func__);
    }

    gpio_set_level(CONFIG_LASTLINK_LED_ACTIVITY_GPIO, activity_count != 0); 
    gpio_set_level(2, activity_count != 0); 
}

void linklayer_print_packet(const char* reason, packet_t* packet)
{
    char* buffer = linklayer_format_packet(packet);

    if (buffer != NULL) {

#if CONFIG_LASTLINK_DEBUG_PACKET_ALLOCATION
        ESP_LOGE(TAG, "%s: (%p) %s [%s:%d]", reason, packet, buffer, packet->last_referenced_filename, packet->last_referenced_lineno);
#else
        ESP_LOGE(TAG, "%s: (%p) %s", reason, packet, buffer);
#endif

        free((void*) buffer);

    } else {
        ESP_LOGD(TAG, "%s: NULL packet", __func__);
    }
}

char* linklayer_format_packet(packet_t* packet)
{
    char *buffer = NULL;

    if (packet != NULL) {
        int routeto  = get_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN);
        int flags    = get_uint_field(packet, HEADER_FLAGS, FLAGS_LEN);
        int origin   = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
        int dest     = get_uint_field(packet, HEADER_DEST_ADDRESS, ADDRESS_LEN);
        int sender   = get_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN);
        int sequence = get_uint_field(packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN);
        int protocol = get_uint_field(packet, HEADER_PROTOCOL, PROTOCOL_LEN);
        int metric   = get_uint_field(packet, HEADER_METRIC, METRIC_LEN);

        const char* info = linklayer_packet_format(packet, protocol);

        asprintf(&buffer, "R=%04x O=%04x D=%04x S=%04x F=%02x Seq=%d Metric=%d Proto=%d Ref=%d Radio=%d L=%d: %s",
                 routeto,
                 origin,
                 dest,
                 sender,
                 flags,
                 sequence,
                 metric,
                 protocol,
                 packet->ref,
                 packet->radio_num,
                 packet->length,
                 info ? info : "");

        free((void*) info);
    }

    return buffer;
}

static const char* linklayer_packet_format(const packet_t* packet, int protocol)
{
    const char* text;

    if (protocol >= 0 && protocol < ELEMENTS_OF(protocol_table) && protocol_table[protocol].format != NULL) {
        text = protocol_table[protocol].format(packet);
    } else {
        text = default_packet_format(packet);
    }

    return text;
}

void linklayer_release_packets_in_queue(os_queue_t queue) {
    if (queue != NULL) {
        packet_t* packet;
        while (os_get_queue_with_timeout(queue, (os_queue_item_t*) &packet, 0)) {
            release_packet(packet);
        }
    }
}

static int linklayer_print_lock_status(int argc, const char** argv)
{
    printf("linklayer_lock: %s\n", os_get_mutex_count(linklayer_mutex) ? "UNLOCKED" : "LOCKED");
    return 0;
}

/* Creates the linklayer */
bool linklayer_init(int address, int flags, int announce)
{
    bool ok = false;

    ESP_LOGD(TAG, "%s: address %d flags 0x%02x announce %d", __func__, address, flags, announce);

    linklayer_node_address = address;
    linklayer_mutex = os_create_recursive_mutex();
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
        sequence_number = UNDEFINED_SEQUENCE_NUMBER;
        debug_flag = false;
        announce_interval = announce;

        if (announce_interval != 0) {
            /* Start the route announce thread */
        }

        route_table_init();

        packet_errors_crc = 0;
        packet_processed = 0;
        packet_received = 0;
        packet_transmitted = 0;
        packet_ignored = 0;

        ok = true;

        /* Reserve core packet protocols here */
        if (linklayer_register_protocol(BEACON_PROTOCOL,         beacon_packet_process,        beacon_packet_format)        &&
            linklayer_register_protocol(ROUTEREQUEST_PROTOCOL,   routerequest_packet_process,  routerequest_packet_format)  &&
            linklayer_register_protocol(ROUTEANNOUNCE_PROTOCOL,  routeannounce_packet_process, routeannounce_packet_format) &&
            linklayer_register_protocol(ROUTEERROR_PROTOCOL,     routeerror_packet_process,    routeerror_packet_format)) {

            for (int radio_num = 0; radio_num < NUM_RADIOS; ++radio_num) {
                ok = ok && linklayer_add_radio(radio_num, &radio_config[radio_num]);
            }
        } else {
           ok = false;
        }
    }

#if CONFIG_LASTLINK_ENABLE_SOCKET_LAYER
    if (ok) {
        ok = ls_socket_init() == LSE_NO_ERROR;
    }
#endif /* CONFIG_LASTLINK_ENABLE_SOCKET_LAYER */


#if CONFIG_LASTLINK_TABLE_COMMANDS
    add_command("ll", linklayer_print_lock_status);
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */

    return ok;
}


bool linklayer_deinit(void)
{
#if CONFIG_LASTLINK_TABLE_COMMANDS
    remove_command("ll");
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */

    if (linklayer_lock()) {

#if CONFIG_LASTLINK_ENABLE_SOCKET_LAYER
        ls_socket_deinit();
#endif

        if (announce_thread != NULL) {
            os_delete_thread(announce_thread);
            announce_thread = NULL;
        }

        if (receive_queue != NULL) {
            os_delete_queue(receive_queue);
            receive_queue = NULL;
        }

        route_table_deinit();

        /* Deinitialize and remove all radios */
        for (int radio_num = 0; radio_num < NUM_RADIOS; ++radio_num) {
            linklayer_remove_radio(radio_num);
        }

        linklayer_unlock();

        os_delete_mutex(linklayer_mutex);
        linklayer_mutex = NULL;
    }

    return linklayer_mutex == NULL;
}

