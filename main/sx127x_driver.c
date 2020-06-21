/*
 * sx127x_driver.c
 *
 * The driver for all SX127x chips.  Supports SPI currently but is rather agnostic to the interface
 * type.  The caller who instantiates this handler supplies pointers to read and write register and
 * buffer functions, as well as some ancillary (e.g. attach to interrupt) functions that allows
 * performing the lower-level platform specific operations.
 */

#undef USE_FHSS
#include "sdkconfig.h"

#undef DISPLAY_HANDLER_STATE

/*
 * Driver for sx127x chip.
 */
#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED

#define LORA_HEADER_OVERHEAD   22

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "esp_system.h"
#include "esp_log.h"

#include "radio.h"
#include "packets.h"
#include "os_specific.h"
#include "sx127x_driver.h"
#include "linklayer.h"
#include "simpletimer.h"

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
#include "commands.h"
#endif

#ifdef CONFIG_LASTLINK_CRC16_PACKETS
#include "crc16.h"
#endif

#define TAG "sx127x_driver"

#undef MEASURE_RX_TIME

static bool radio_stop(radio_t* radio);                              /* Stop and disassemble the radio */

static bool set_sleep_mode(radio_t* radio);                          /* Set sleeping mode (low power) */
static bool set_standby_mode(radio_t* radio);                        /* Set standby mode */
static bool set_receive_mode(radio_t* radio);                        /* Set "real" receive mode */
static bool set_cad_detect_mode(radio_t* radio);                     /* Set cad detect mode */

static bool set_txpower(radio_t* radio, int power);                  /* Set transmitter power level */
static int get_txpower(radio_t* radio);                              /* Get current transmit power */
static int set_channel(radio_t* radio, int channel);                 /* Set channel */
static int get_channel(radio_t* radio);                              /* Get channel */
static int set_datarate(radio_t* radio, int datarate);               /* Set datarate */
static int get_datarate(radio_t* radio);                             /* Get datarate */

typedef struct sx127x_private_data sx127x_private_data_t;  // Forward ref

static void catch_interrupt(void *param);
static void rx_handle_interrupt(radio_t* radio, sx127x_private_data_t *data);
#ifdef USE_FHSS
static void fhss_handle_interrupt(radio_t* radio, sx127x_private_data_t *data);
#endif /* USE_FHSS */

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
void print_status(radio_t *radio);
#endif

static bool tx_handle_interrupt(radio_t* radio, sx127x_private_data_t *data);
static void transmit_start(radio_t *radio);
static void global_interrupt_handler(void* param);
static bool radio_stop(radio_t* radio);
static bool release_lock(radio_t* radio);
static bool radio_start(radio_t* radio);
static int get_packet_snr(radio_t* radio);
static bool set_transmit_mode(radio_t* radio);
static int get_packet_rssi(radio_t* radio);
static bool acquire_lock(radio_t* radio);
static bool release_lock(radio_t* radio);
static bool set_bandwidth(radio_t* radio, int bw);
static bool set_spreading_factor(radio_t*, int spreading_factor);
static int get_message_time(radio_t* radio, int length);
static int set_coding_rate(radio_t* radio, int rate);
static int set_preamble_length(radio_t* radio, int length);
#ifdef NOTUSED
static int get_preamble_length(radio_t* radio);
static int get_coding_rate(radio_t* radio);
static int get_spreading_factor(radio_t* radio);
#endif
static int set_enable_crc(radio_t* radio, bool enable);
static bool set_hop_period(radio_t* radio, int hop_period);
static bool set_implicit_header(radio_t* radio, bool implicit_header);
static bool set_sync_word(radio_t* radio, uint8_t sync);
static bool enable_irq(radio_t* radio, uint8_t mask);
static bool disable_irq(radio_t* radio, uint8_t mask);
static void update_data_rate(radio_t* radio);

/* Define the selected frequency domain */
#include "sx127x_table.h"


typedef enum {
    HS_STARTUP,
    HS_WAITING,                   /* Handler is waiting (default state) */
    HS_CAD_RESTART,               /* CAD timeout so restart */
    HS_RECEIVING,                 /* Handler is receiving */
    HS_RECEIVE_DONE,              /* Handler is finished receiving */
    HS_TRANSMIT_DONE,             /* Handler is finished transmitting */
    HS_TRANSMIT_DONE_PRIORITY,    /* Handler is finished transmitting priority packet */
    HS_WAIT_TX_INT,               /* Wait for TX to finish */
} handler_state_t;

#if defined(CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS) || defined(DISPLAY_HANDLER_STATE)
inline const char *handler_state_of(handler_state_t state)
{
    switch (state) {
        case HS_STARTUP:                return "HS_STARTUP";
        case HS_WAITING:                return "HS_WAITING";
        case HS_CAD_RESTART:            return "HS_CAD_RESTART";
        case HS_RECEIVING:              return "HS_RECEIVING";
        case HS_RECEIVE_DONE:           return "HS_RECEIVE_DONE";
        case HS_TRANSMIT_DONE:          return "HS_TRANSMIT_DONE";
        case HS_TRANSMIT_DONE_PRIORITY: return "HS_TRANSMIT_DONE";
        case HS_WAIT_TX_INT:            return "HS_WAIT_TX_INT";
        default:                        return "UNKNONWN";
    }
}
#endif

/* Define private data structure */
typedef struct sx127x_private_data {
    uint8_t           sync_word;
    uint8_t           preamble_length;
    uint8_t           coding_rate;
    bool              implicit_header;
    bool              implicit_header_set;
    int               hop_period;
    bool              enable_crc;
    int               channel;
    int               datarate;
    int               bandwidth;
    uint8_t           spreading_factor;
    int               data_rate_bps;
    uint8_t           tx_power;
    packet_t          *rx_next_packet;
    int               rx_interrupts;
#ifdef MEASURE_RX_TIME
    uint64_t          rx_start_time;
    uint64_t          rx_end_time;
#endif
#define RX_TIMEOUT_TIME  1000
    int               tx_interrupts;
#define TX_TIMEOUT_TIME  1000
#ifdef USE_FHSS
    int               fhss_interrupts;
#endif /* USE_FHSS */
    os_mutex_t        rlock;
    int               packet_memory_failed;
    packet_t         *current_packet;
    int               current_packet_window;          /* Calculated window number for this packet */
    int               packet_crc_errors;
    int               cad_interrupts;
    int               cad_last_interrupts;
    int               cad_detected;
    int               cad_timeouts;
    int               tx_timeouts;
    int               rx_timeouts;
    int               wakeup_ticks;
#define CAD_TIMEOUT_TIME  100
    int               handler_cycles;
    os_timer_t        handler_wakeup_timer_id;        /* ID of radio wakeup timer in case things hang */
#define HANDLER_WAKEUP_TIMER_PERIOD 1000              /* Once a second */
    simpletimer_t     handler_wakeup_timer_status;    /* So we can check status of non-readble timer */
    uint8_t           irq_flags;                      /* Flags acquired at irq scan time */
    handler_state_t   handler_state;
#ifdef DISPLAY_HANDLER_STATE
    handler_state_t   last_handler_state;
#endif

    int               window_number;
    int               window_width;
    os_timer_t        window_timer_id;
    int               window_event_count;

#define TX_MAX_TIMEOUT  5000                      /* 5 second timeout */

} sx127x_private_data_t;

static os_thread_t  global_interrupt_handler_thread;
static os_queue_t   global_interrupt_handler_queue;
static void         global_interrupt_handler(void* param);
static int          global_number_radios_active;

#define MAX_IRQ_PENDING         50


#define GPIO_DIO0         0

#define GPIO_DIO1         1
#define CAD_DIO_MAPPING           0b10101111    /* DIO0 to CAD_DONE; DIO1 to CAD_DETECTED; DIO2 to undefined */
#define RX_DIO_MAPPING            0b00001111    /* DIO0 to RX_DONE;  DIO1 to RX_TIMEOUT;   DIO2 to undefined */
#define TX_DIO_MAPPING            0b01111111    /* DIO0 to TX_DONE;  DIO1 to undefined;    DIO2 to undefined */

#define GLOBAL_IRQ_THREAD_STACK    16000
#define GLOBAL_IRQ_THREAD_PRIORITY (configMAX_PRIORITIES-1)  /* Highest priority */

#define WANTED_VERSION  0x12

typedef struct handler_queue_event {
    radio_t *radio;
    enum {
        HQI_INTERRUPT=1,   /* A hardware interrupt */
        HQI_RESET,         /* Reset from hung software */
        HQI_WINDOW,        /* A window */
    } type; 
    union {
        handler_state_t state;  
        int             window;
    };
} handler_queue_event_t;

/* Called when radio's wakeup timer fires - generates 'interrupt' to handler */
static void handler_wakeup_timer(os_timer_t timer_id)
{
    radio_t *radio = (radio_t*) os_get_timer_data(timer_id);
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (data->cad_last_interrupts == data->cad_interrupts) {
        /* No CAD interrupts detected in the past few cycles, so reset state via event */
    
        handler_queue_event_t event = {
            .type = HQI_RESET,
            .radio = radio,
            .state = data->handler_state,
        };

printf("%s: radio %d timeout\n", __func__, radio->radio_num);

        os_put_queue_with_timeout(global_interrupt_handler_queue, (os_queue_item_t) &event, 0);

    } else {
        data->cad_last_interrupts = data->cad_interrupts;
    }

    data->wakeup_ticks++;

    /* Restart the software timer that is used to indicate system timer status, which we cannot read. */
    simpletimer_start(&data->handler_wakeup_timer_status, HANDLER_WAKEUP_TIMER_PERIOD);
}

inline static int calculate_window_number(radio_t *radio, packet_t *packet)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return (get_uint_field(data->current_packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN) *
            get_uint_field(data->current_packet, HEADER_DEST_ADDRESS, ADDRESS_LEN) *
            get_uint_field(data->current_packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN) +
            2 * get_uint_field(data->current_packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN)) % radio->transmit_windows;
}

/*
 * Called to update the window.
 */
static void window_timer(os_timer_t timer_id)
{
    radio_t *radio = (radio_t*) os_get_timer_data(timer_id);
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    /* Set interval in case we were short cycled to get here */
    os_set_timer(timer_id, data->window_width);

    /* Change the window */
    data->window_number = (data->window_number + 1) % radio->transmit_windows;

    /* Count the number of window events */
    data->window_event_count++;

    /* Send the window event */
    handler_queue_event_t event = {
        .type = HQI_WINDOW,
        .radio = radio,
        .window = data->window_number,
    };

    os_put_queue_with_timeout(global_interrupt_handler_queue, (os_queue_item_t) &event, 0);
}

/*
 * Create an instance of a SX127x device.
 *
 * Entry:
 *    radio		Pointer to value to receive pointer to SX127x control information
 *
 * Returns:
 * 	  true if successful
 *
 */
bool sx127x_create(radio_t* radio)
{
    if (radio != NULL) {
        /* Start global interrupt processing thread if not yet running */
        if (global_interrupt_handler_thread == NULL) {
            global_interrupt_handler_queue = os_create_queue(MAX_IRQ_PENDING, sizeof(handler_queue_event_t));
            global_interrupt_handler_thread = os_create_thread_on_core(global_interrupt_handler, "sx127x_handler",
                                                               GLOBAL_IRQ_THREAD_STACK, GLOBAL_IRQ_THREAD_PRIORITY, NULL, 0);
        }

        /* Add callouts into the driver.  All are called with radio as first parameter */
        radio->stop             = radio_stop;
        radio->set_sleep_mode   = set_sleep_mode;
        radio->set_standby_mode = set_standby_mode;
#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
        radio->print_status     = print_status;
#endif
        radio->set_receive_mode = set_cad_detect_mode;
        radio->set_txpower      = set_txpower;
        radio->get_txpower      = get_txpower;
        radio->set_channel      = set_channel;
        radio->get_channel      = get_channel;
        radio->set_datarate     = set_datarate;
        radio->get_datarate     = get_datarate;
        radio->transmit_start   = transmit_start;
        radio->get_message_time = get_message_time;

        /* Allocate a data block for local data */
        radio->driver_private_data = malloc(sizeof(sx127x_private_data_t));

        if (radio->driver_private_data != NULL) {
            memset(radio->driver_private_data, 0, sizeof(sx127x_private_data_t));

            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

            /* Initialize defaults */
            data->sync_word                 = 0x12;    /* 0x34 LoRaWAN; 0x12 LoRa */
            data->preamble_length           = 32;      /* Was 8 */
            data->coding_rate               = 5;
            data->implicit_header           = false;
            data->implicit_header_set       = false;
            //data->hop_period                = 0;
            data->enable_crc                = true;
            data->bandwidth                 = 125000;
            data->spreading_factor          = 10;
            data->tx_power                  = 2;
            //data->rx_interrupts             = 0;
            //data->tx_interrupts             = 0;
            //data->rx_timeouts               = 0;
            //data->tx_timeouts               = 0;
            //data->cad_interrupts            = 0;
            //data->cad_detected              = 0;
            //data->handler_cycles            = 0;
            //data->window_number     = 0;

            data->rlock                     = os_create_recursive_mutex();


            data->handler_state             = HS_STARTUP;

#ifdef DISPLAY_HANDLER_STATE
            data->last_handler_state        = -1;
#endif

            /* Create the wakeup timer and start it */
            data->handler_wakeup_timer_id = os_create_repeating_timer("wakeup_timer", HANDLER_WAKEUP_TIMER_PERIOD, radio, handler_wakeup_timer);
            os_start_timer(data->handler_wakeup_timer_id);

            /* Create window event timer */
            data->window_timer_id = os_create_timer("wakeup_timer", HANDLER_WAKEUP_TIMER_PERIOD, radio, window_timer);


            /* Send initial wakeup call to handler */
            handler_queue_event_t event = {
                .radio = radio,
                .type = HQI_INTERRUPT,
            };

            os_put_queue_with_timeout(global_interrupt_handler_queue, (os_queue_item_t) &event, 0);
        }

        ++global_number_radios_active;
    }

	return radio != NULL && radio->driver_private_data != NULL && radio_start(radio);
}

/*
 * radio_delete
 *
 * stop and remove radio from system.
 *
 * TODO
 */


/************************************************************************************
 *    Thread base interrupt handler
 *
 *    As interrupts are detected, flags are passed in parameters
 *    and the radio element is queued to the global_interrupt_handler_queue.
 *
 *    This thread processes these elements one at a time and processes each
 *    passing, the radio and irq flags value at time of interrupt.
 *
 ************************************************************************************/
static void rx_handle_interrupt(radio_t* radio, sx127x_private_data_t *data)
{
    data->rx_interrupts++;

    /* Clear rx interrupt */
    data->irq_flags &= ~SX127x_IRQ_RX_DONE;

    uint8_t fifo_current = radio->read_register(radio, SX127x_REG_RX_FIFO_CURRENT);
    if (fifo_current != 0) {
        ESP_LOGE(TAG, "%s: FIFO_CURRENT not 0: %d (%02x)", __func__, fifo_current, fifo_current);
    }

    radio->write_register(radio, SX127x_REG_FIFO_PTR, fifo_current);

    int length;

    if (data->implicit_header) {
        length = radio->read_register(radio, SX127x_REG_PAYLOAD_LENGTH);
    } else {
        length = radio->read_register(radio, SX127x_REG_RX_NUM_BYTES);
    }

    /* Cannot reliably capture packet if CRC error so don't even try */
    if ((data->irq_flags & (SX127x_IRQ_VALID_HEADER | SX127x_IRQ_PAYLOAD_CRC_ERROR )) == 0) {

        /* Get a packet */
        packet_t* packet = data->rx_next_packet;
        if (packet != NULL) {

            data->rx_next_packet = NULL;

            if (radio->read_buffer(radio, SX127x_REG_FIFO, packet->buffer, length)) {
#ifdef CONFIG_LASTLINK_CRC16_PACKETS
                bool crcok = false;
                if (length >= 3) {
                    /* Check the add crc on the packets */
                    crcok = calc_crc16(CRC16_SEED, packet->buffer, length) == 0;
                    if (crcok) {
                       length = length - 2;
                    }
  #if 0
                    else {
                        /* Calculate crc on the base and display calculated and wanted values for debugging */
                        unsigned crc_wanted = calc_crc16(CRC16_SEED, packet->buffer, length - 2);
                        unsigned crc_found = (packet->buffer[length - 2] << 8) | packet->buffer[length - 1];
                        ESP_LOGE(TAG, "%s: packet length %d crc_wanted %04x crc_found %04x", __func__, length, crc_wanted, crc_found);
                    }
  #endif
                }
#else
                bool crcok = true;
#endif

                /* Buffer has been read.  Set length */
                packet->length = length;
                packet->crc_ok = crcok;
                packet->rssi = get_packet_rssi(radio);
                packet->snr = get_packet_snr(radio);
                packet->radio_num = radio->radio_num;

                /* Pass it to protocol layer */
                radio->on_receive(ref_packet(packet));

                /* Calculate window and starting point */
                int window = calculate_window_number(radio, packet);
                int time_of_flight = get_message_time(radio, packet->length);

#ifdef MEASURE_RX_TIME
int measured = (int) (data->rx_end_time - data->rx_start_time);
printf("%s: rx msg time: calc %d measured %d delta %d\n", __func__, time_of_flight, measured, time_of_flight - measured);
#endif /* MEASURE_RX_TIME */
             
                /* Synchronize window to align to next assumed slot (it is reset inside interval handler) */
                os_stop_timer(data->window_timer_id);
                data->window_number = window;
                os_set_timer(data->window_timer_id, data->window_width - time_of_flight);
            }

            release_packet(packet);

        } else {
            data->packet_memory_failed++;
        }
    } else {
//ESP_LOGE(TAG, "%s: errors %02x", __func__, data->irq_flags);
        if ((data->irq_flags & SX127x_IRQ_PAYLOAD_CRC_ERROR) != 0) {
ESP_LOGE(TAG, "%s: rxint with crc", __func__);
           data->packet_crc_errors++;
        }
        data->irq_flags &= ~(SX127x_IRQ_PAYLOAD_CRC_ERROR | SX127x_IRQ_VALID_HEADER | SX127x_IRQ_PAYLOAD_CRC_ERROR);
    }
}

#ifdef USE_FHSS
/* THIS NEEDS WORK */
static void fhss_handle_interrupt(radio_t* radio, sx127x_private_data_t *data)
{
#ifdef NOTUSED
    radio->write_register(radio, SX127x_FHSS_HOP_CHANNEL, next_channel);
#endif
    data->fhss_interrupts++;
}
#endif /* USE_FHSS */

static bool start_packet(radio_t* radio)
{
    return set_standby_mode(radio)
           && radio->write_register(radio, SX127x_REG_FIFO_PTR, TX_FIFO_BASE)
           && radio->write_register(radio, SX127x_REG_TX_FIFO_BASE, TX_FIFO_BASE)
           ;
}

static bool write_packet(radio_t* radio, packet_t* packet)
{
    bool ok = false;

#ifdef CONFIG_LASTLINK_CRC16_PACKETS
    /* Compute crc to add to message.  Can't modify packet as it may be sent more
     * than one place.  Need to compute and send crc separately.
     */
    unsigned short crc = calc_crc16(CRC16_SEED, packet->buffer, packet->length);
    unsigned char crcbuf[2];

    crcbuf[0] = crc >> 8;
    crcbuf[1] = crc & 0xFF;

    radio->write_buffer(radio, SX127x_REG_FIFO, packet->buffer, packet->length);
    radio->write_buffer(radio, SX127x_REG_FIFO, crcbuf, sizeof(crcbuf));

    /* Set the new length of the payload in process */
    radio->write_register(radio, SX127x_REG_PAYLOAD_LENGTH, packet->length + 2);
#else
    radio->write_buffer(radio, SX127x_REG_FIFO, packet->buffer, packet->length);
    radio->write_register(radio, SX127x_REG_PAYLOAD_LENGTH, packet->length);
#endif

    ok = true;

    return ok;
}

/* Called upon interrupt */
static bool tx_handle_interrupt(radio_t *radio, sx127x_private_data_t *data)
{
    /* Remove flag so we don't call again until next interrupt */
    data->irq_flags &= ~SX127x_IRQ_TX_DONE;

    data->tx_interrupts++;
    radio->activity_indicator(radio, false);

    bool priority = false;

    /* Discard current queue entry */
    if (data->current_packet != NULL) {
        
        priority = (get_uint_field(data->current_packet, HEADER_FLAGS, FLAGS_LEN) & HEADER_FLAGS_PRIORITY) != 0;

        data->current_packet->transmitting--;

        /* Count as transmitted */
        data->current_packet->transmitted++;

        /*
         * No critical race with respect to packet fields and the packet will not
         * enter transmitting mode if it is locked by another thread.  Only
         * when the current 'winner' finishes the thread will the next one be
         * able to lock and start a transmit.
         */

        /* Only do when all radios have finished. */
        if (data->current_packet->transmitting == 0) {
            /* Tell any interested listener that it has been transmitted */
            packet_tell_transmitted_callback(data->current_packet, radio);
        }

        assert(packet_unlock(data->current_packet));

        release_packet(data->current_packet);
        data->current_packet = NULL;
    }

    return priority;
}

/*
 * Called when we want to start a new packet.
 */
static bool tx_next_packet(radio_t *radio, sx127x_private_data_t *data)
{
    /* Get a new packet if we have none */
    if (data->current_packet == NULL) {
        if (os_get_queue_with_timeout(radio->transmit_queue, (os_queue_item_t) &data->current_packet, 0)) {

            /* Fetched a new packet, so calculate it's window */
            data->current_packet_window = calculate_window_number(radio, data->current_packet);
//printf("%s: window %d\n", __func__, data->current_packet_window);
        }
    }

    return data->current_packet != NULL;
}

/*
 * Move current packet to end of transmit queue for sending later.
 */
static void tx_recycle_packet(radio_t *radio, sx127x_private_data_t *data)
{
    if (data->current_packet != NULL) {
        if (os_put_queue_with_timeout(radio->transmit_queue, (os_queue_item_t) &data->current_packet, 0)) {
            data->current_packet = NULL;
        }
    }
}

static void tx_start_packet(radio_t *radio, sx127x_private_data_t *data)
{
   start_packet(radio);
   write_packet(radio, data->current_packet);
   set_transmit_mode(radio);
   radio->activity_indicator(radio, true);
}


/* Force start transmit interrupt handler */
static void transmit_start(radio_t *radio)
{
#if 0
    if (acquire_lock(radio)) {

        release_lock(radio);

    } else {
        ESP_LOGE(TAG, "%s: Cannot acquire lock", __func__);
    }
#endif
}

/*
 * Need to pass in a 'code' for the reason.
 *
 *   reasons:  Interrupt; Timer
 *
 * Normal operation is via direct interrupts, specifically receive and CAD interrupts (for
 * scanning channel available and read-packet detection.  Transmit is controlled by items
 * coming from the input queue and the 'window' tick timer.
 *
 * When a packet is ready to transmit, a determination is made as to how close it is
 * to the window edge and a timer is activated to start the operation at that time.
 *
 * The window determination is federated: each node is responsible for self-synchronizing
 * to the window period.  For example, when a packet is read, after the interrupt the
 * window starting point is calculated to be:
 *             <current time> - <'time of flight' of the packet>
 * This value is used to reset the next window event detection interrupt time.
 *
 * When the window heartbeat is detected, the system determines if a packet exists that
 * hashes to the current window interval and if so, launches it.
 *
 * Window interval hashing is a work-in-progress but is based upon a hash of the
 * origin, destination and sequence number of the packet.  This hash is reduced to
 * an N bit number and this drives the window cycle.
 *
 * For example when a packet is received, the window hash for that packet is calculated
 * and used to reset the current window number and this becomes the basis for the next
 * window to be delivered to the handler.
 *
 * The rate of the window number update is based on the maximum time-of-flight for a
 * packet at the current bandwidth / coding_rate / spreading_factor.
 *
 */
/* Param is ignored */
static void global_interrupt_handler(void* param)
{
    bool running = true;

    ESP_LOGD(TAG, "%s: running", __func__);

    while (running) {
        handler_queue_event_t event;

        if (os_get_queue(global_interrupt_handler_queue, (os_queue_item_t) &event)) {

            radio_t *radio = event.radio;


            if (radio != NULL) {
                sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

                if (event.type == HQI_RESET) {
                    /* Force state transition */
                    data->handler_state = event.state;
                    data->cad_timeouts++;
                }

#if 0
                if (event.type == HQI_WINDOW) {
                    printf("window %d\n", event.window);
                }
#endif

                data->handler_cycles++;

                if (acquire_lock(radio)) {
                    /* Remember what we have */
                    data->irq_flags |= radio->read_register(radio, SX127x_REG_IRQ_FLAGS);

                    /* Physically clear the interrupts detected */
                    radio->write_register(radio, SX127x_REG_IRQ_FLAGS, data->irq_flags & 0xFF);

#ifdef USE_FHSS
                    if (data->irq_flags & SX127x_IRQ_FHSS_CHANGE_CHANNEL) {
                        fhss_handle_interrupt(radio, data);
                        data->irq_flags &= ~SX127x_IRQ_FHSS_CHANGE_CHANNEL;
                    }
#endif /* USE_FHSS */

// printf("%s %02x %02x\n", handler_state_of(data->handler_state),  data->irq_flags, radio->read_register(radio, SX127x_REG_IRQ_FLAGS_MASK));

#ifdef DISPLAY_HANDLER_STATE
                    if (data->handler_state != data->last_handler_state) {
                        printf("Handler state %s\n", handler_state_of(data->handler_state));
                        data->last_handler_state = data->handler_state;
                    }
#endif

                    /* Allocate a new receive packet if we can */
                    if (data->rx_next_packet == NULL) {
                        data->rx_next_packet = allocate_packet();
                        /*
                         * If no packets, release from the transmit queue until we find one.  They will be regenereted
                         * at some point if needed.
                         *
                         * Note that it is possible we can release the entire transmit queue and not actually
                         * get any packets due to they might be on other queues and still have ref's.
                         *
                         * At any rate, this gets rid of the oldest outgoing packets in a packet-starved situation
                         * and many of the packets on the queue may newer copies of these old ones.
                         */

                        if (data->rx_next_packet == NULL) {
                            bool done = false;
                            packet_t *packet;
                            int count = 0;

                            do {
                                if (os_get_queue_with_timeout(radio->transmit_queue, (os_queue_item_t) &packet, 0)) {
                                    ++count;
                                    release_packet(packet);
                                    data->rx_next_packet = allocate_packet();
                                } else {
                                    done = true;
                                }
                            } while (!done && data->rx_next_packet == NULL);

                            if (data->rx_next_packet != NULL) {
                                printf("%s: found free packet after %d releases of transmit queue\n", __func__, count);
                            }
                        }
                    }

                    switch (data->handler_state) {
                        default:
                        case HS_STARTUP: {
                            /* Default state of receiving until interrupt seen or transmit packet is ready to go.
                             * To start transmit, caller places packet into the transmit queue and puts a radio*
                             * pointer into the handler queue.  The handler sees this pointer and determines if
                             * it can start a new packet at that time.  If the handerl is in receive mode and
                             * the transmit delay timer has expired, it attempts to start the packet.
                             */
                            break;
                        }

                        /*
                         * Waiting in Carrier Detect mode.  When we get an input signal, go to receive mode.
                         * If we get a packet to send, go to transmit mode if it's been long enough since
                         * lasst receive/transmit.
                         */

                        case HS_WAITING: {
                            if ((data->irq_flags & (SX127x_IRQ_CAD_DONE)) != 0) {
                                data->cad_interrupts++;

                                if ((data->irq_flags & (SX127x_IRQ_CAD_DETECTED)) != 0) {
                                    data->cad_detected++;
                                    data->handler_state = HS_RECEIVING;
                                }

                                data->irq_flags &= ~(SX127x_IRQ_CAD_DONE | SX127x_IRQ_CAD_DETECTED);
                            }

                            if ((data->handler_state == HS_WAITING) && (event.type == HQI_WINDOW)) {
//printf("%s: window %d\n", __func__, data->current_packet_window);
                                /* If packet exists and it's window id is for the current window, start it going */
                                if (tx_next_packet(radio, data)) {
                                    if (data->current_packet_window == event.window) {
                                        if (packet_lock(data->current_packet)) {
                                            /* Remains locked through the interrupt return */
                                            tx_start_packet(radio, data);
                                            data->handler_state = HS_WAIT_TX_INT;
                                        } else {
                                            /* Recycle packet to end of queue in hopes we can do some other work while waiting. */
                                            /* If not, we'll keep moving it back into the queue on each CAD interrupt for a while. */
                                            tx_recycle_packet(radio, data);
                                        }
                                    }
//else { printf("%s: window %d want %d\n", __func__, event.window, data->current_packet_window); }
                                }
                            }
                            break;
                        }

                        /*
                         * A carrier was detected.  Attempt to receive the packet or wait for timeout.
                         */
                        case HS_RECEIVING: {
                            if (data->irq_flags & SX127x_IRQ_RX_DONE) {
                                /* Packet arrived.  Process it */
#ifdef MEASURE_RX_TIME
                                data->rx_end_time = get_milliseconds();
#endif /* MEASURE_RX_TIME */
                                rx_handle_interrupt(radio, data);

                                /* Allocate a new packet if we can */
                                if (data->rx_next_packet == NULL) {
                                    data->rx_next_packet = data->rx_next_packet;
                                }
                                data->handler_state = HS_RECEIVE_DONE;

                            } else if (((data->irq_flags & (SX127x_IRQ_RX_TIMEOUT)) != 0)) {
                                data->rx_timeouts++;

                                /* Receive attempt timed out.  Return to active mode and clear receive busy */
                                data->irq_flags &= ~(SX127x_IRQ_RX_TIMEOUT);
                                data->handler_state = HS_CAD_RESTART;
                            }
                                 
                            break;
                        }

                        /* Actively transmitting - wait for interrupt */
                        case HS_WAIT_TX_INT: {
                            if (data->irq_flags & SX127x_IRQ_TX_DONE) {
                                /* Turns off indicator and releases packet */
                                data->handler_state = tx_handle_interrupt(radio, data) ? HS_TRANSMIT_DONE_PRIORITY : HS_TRANSMIT_DONE;
                            }
                            break;
                        }
                    }

                    switch (data->handler_state) {
                        case HS_STARTUP:
                        case HS_CAD_RESTART: {
                            set_cad_detect_mode(radio);
                            break;
                        }

                        case HS_WAITING: {
                            set_cad_detect_mode(radio);
                            break;
                        }

                        case HS_RECEIVING: {
                            set_receive_mode(radio);
#ifdef MEASURE_RX_TIME
                            data->rx_start_time = get_milliseconds();
#endif /* MEASURE_RX_TIME */
                            break;
                        }

                        case HS_RECEIVE_DONE: {
                            set_cad_detect_mode(radio);

                            /* And go back to waiting */
                            data->handler_state = HS_WAITING;
                            break;
                        }

                        case HS_TRANSMIT_DONE_PRIORITY: {
                            set_standby_mode(radio);
                            set_cad_detect_mode(radio);

                            /* And go back to waiting */
                            data->handler_state = HS_WAITING;
                            break;
                        }

                        case HS_TRANSMIT_DONE: {
                            set_standby_mode(radio);
                            set_cad_detect_mode(radio);


                            /* And go back to waiting */
                            data->handler_state = HS_WAITING;
                            break;
                        }

                        case HS_WAIT_TX_INT: {
                            break;
                        }
                    }

                    release_lock(radio);

                } else {
                    ESP_LOGE(TAG, "%s: unable to acquire lock", __func__);
                }
            } else {
                /* NULL delivered - shut down */
                running = false;
printf("sx127x_driver handler stopped\n");
            }
        } else {
            ESP_LOGE(TAG, "%s: interrupt queue failed to read - exiting", __func__);
            running = false;
        }
    }

    ESP_LOGD(TAG, "%s: stopped", __func__);

    os_exit_thread();
}

static bool radio_stop(radio_t* radio)
{
    bool ok = false;

    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (acquire_lock(radio)) {
        /* Disable all interrupts */
        ok = disable_irq(radio, 0xFF)
             && radio->attach_interrupt(radio, GPIO_DIO0, GPIO_PIN_INTR_DISABLE, NULL)
#ifdef GPIO_DIO1
             && radio->attach_interrupt(radio, GPIO_DIO1, GPIO_PIN_INTR_DISABLE, NULL)
#endif
             ;

        /* Kill handler window timer */
        os_delete_timer(data->window_timer_id);
        data->window_timer_id = NULL;

        /* Kill handler wakeup timer */
        os_delete_timer(data->handler_wakeup_timer_id);
        data->handler_wakeup_timer_id = NULL;

        if (ok && --global_number_radios_active == 0) {
            /* Kill interrupt thread and queue */
            os_delete_thread(global_interrupt_handler_thread);
            global_interrupt_handler_thread = NULL;

            /* Remove global queue */
            os_delete_queue(global_interrupt_handler_queue);
            global_interrupt_handler_queue = NULL;
        }

        release_lock(radio);
    }

    return ok;
}


static bool acquire_lock(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

//ESP_LOGI(TAG, "%s: acquiring radio lock", __func__);

    bool ok = os_acquire_recursive_mutex(data->rlock);

//ESP_LOGI(TAG, "%s: acquire lock %s", __func__, ok ? "SUCCESS" : "FAILED");

    return ok;
}

static bool release_lock(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

//ESP_LOGI(TAG, "%s: releasing radio lock", __func__);

    return os_release_recursive_mutex(data->rlock);
}

static bool enable_irq(radio_t* radio, uint8_t mask)
{
    bool ok = false;

    int cur_mask = radio->read_register(radio, SX127x_REG_IRQ_FLAGS_MASK);

    if (cur_mask >= 0) {
        ok = radio->write_register(radio, SX127x_REG_IRQ_FLAGS_MASK, (uint8_t) cur_mask & ~mask);
    }

    return ok;
}

static bool disable_irq(radio_t* radio, uint8_t mask)
{
    bool ok = false;

    int cur_mask = radio->read_register(radio, SX127x_REG_IRQ_FLAGS_MASK);

    if (cur_mask >= 0) {
        ok = radio->write_register(radio, SX127x_REG_IRQ_FLAGS_MASK, (uint8_t) cur_mask | mask);
    }

    return ok;
}

static bool radio_start(radio_t* radio)
{
    bool ok = false;


    if (acquire_lock(radio)) {

        sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

        radio->reset_device(radio);

        /* Read version */
        int version = -1;
        int max_tries = 5;
        while (version != WANTED_VERSION && max_tries != 0) {
            version = radio->read_register(radio, SX127x_REG_VERSION);
            max_tries--;
        }

        if (version != WANTED_VERSION) {
            ESP_LOGE(TAG, "Wrong version detected: %02x wanted %02x", version, WANTED_VERSION);
        } else {
            /* Put receiver in standby */
            set_sleep_mode(radio);

            /* Set initial default  parameters parameters */
            set_bandwidth(radio, data->bandwidth);
            set_spreading_factor(radio, data->spreading_factor);
            set_txpower(radio, data->tx_power);
            set_implicit_header(radio, data->implicit_header);
            set_coding_rate(radio, data->coding_rate);
            set_preamble_length(radio, data->preamble_length);
            set_sync_word(radio, data->sync_word);
            set_enable_crc(radio, data->enable_crc);
            set_hop_period(radio, data->hop_period);

            /* LNA Boost */
            radio->write_register(radio, SX127x_REG_LNA, radio->read_register(radio, SX127x_REG_LNA) | 0x03);

            /* auto AGC enable */
            radio->write_register(radio, SX127x_REG_MODEM_CONFIG_3, 0x04);

            /* FIFO Bases */
            radio->write_register(radio, SX127x_REG_TX_FIFO_BASE, TX_FIFO_BASE);
            radio->write_register(radio, SX127x_REG_RX_FIFO_BASE, RX_FIFO_BASE);


            /* Mask all IRQs */
            disable_irq(radio, 0xFF);

            /* Clear all interrupts */
            radio->write_register(radio, SX127x_REG_IRQ_FLAGS, 0xFF);

            /* Capture receive/transmit interrupts */
            radio->attach_interrupt(radio, GPIO_DIO0, GPIO_PIN_INTR_POSEDGE, catch_interrupt);
#ifdef GPIO_DIO1
            radio->attach_interrupt(radio, GPIO_DIO1, GPIO_PIN_INTR_POSEDGE, catch_interrupt);
#endif

#ifdef USE_FHSS
            if (data->hop_period != 0) {
                /* Enable FHSS interrupt */
                enable_irq(radio, SX127x_IRQ_FHSS_CHANGE_CHANNEL);
            }
#endif

            /* Configure the unit for receive channel 0; probably overriden by caller */
            set_channel(radio, 0);

            ESP_LOGD(TAG, "SX127x radio started");

            ok = true;
        }
        release_lock(radio);
    }

    return ok;
}


static int get_packet_rssi(radio_t* radio)
{
    int rssi = -9999;

    if (acquire_lock(radio)) {
        rssi = radio->read_register(radio, SX127x_REG_PACKET_RSSI) - 157;
        if (channel_table.low_domain_freq < 868E6) {
            rssi = rssi + 7;
        }
    }
    release_lock(radio);

    return rssi;
}

/* Returns SNR in .1 db units (e.g. 1 db SNR would return 10 */
static int get_packet_snr(radio_t* radio)
{
    int snr = -9999;

    if (acquire_lock(radio)) {
        snr = (int) ((radio->read_register(radio, SX127x_REG_PACKET_SNR) * 10.0) / 4.0);
        release_lock(radio);
    }

    return snr;
}


static bool set_standby_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok =  disable_irq(radio, 0xFF)
           && radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_STANDBY)
        ;

        release_lock(radio);
    }

    return ok;
}

static bool set_sleep_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok =  disable_irq(radio, 0xFF)
           && radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_SLEEP);
        release_lock(radio);
    }

    return ok;
}


/*
 * Set receive mode
 */
static bool set_receive_mode(radio_t* radio)
{
    bool ok = false;

//ESP_LOGI(TAG, "%s: set_receive_mode, __func__);

    if (acquire_lock(radio)) {
        ok =  disable_irq(radio, SX127x_IRQ_CAD_DONE | SX127x_IRQ_CAD_DETECTED | SX127x_IRQ_TX_DONE)
           && radio->write_register(radio, SX127x_REG_DIO_MAPPING_1, RX_DIO_MAPPING)
           && radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_RX_SINGLE)
           && enable_irq(radio, SX127x_IRQ_RX_DONE | SX127x_IRQ_RX_TIMEOUT)
           ;

if (!ok) ESP_LOGE(TAG, "%s: failed", __func__);
        release_lock(radio);
    }

    return ok;
}

static bool set_cad_detect_mode(radio_t* radio)
{
    bool ok = false;

    /* Just put the unit into continuous receive */
    if (acquire_lock(radio)) {
        sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

        data->handler_state = HS_WAITING;

        set_standby_mode(radio);

        ok =  disable_irq(radio, SX127x_IRQ_TX_DONE | SX127x_IRQ_RX_DONE | SX127x_IRQ_RX_TIMEOUT)
           && radio->write_register(radio, SX127x_REG_DIO_MAPPING_1, CAD_DIO_MAPPING)
           && radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_CAD_DETECTION)
           && enable_irq(radio, SX127x_IRQ_CAD_DETECTED | SX127x_IRQ_CAD_DONE)
           //&& enable_irq(radio, SX127x_IRQ_CAD_DONE)
           ;

if (!ok) ESP_LOGE(TAG, "%s: failed", __func__);
        release_lock(radio);
    }

    return ok;
}

static bool set_transmit_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok =  disable_irq(radio, SX127x_IRQ_CAD_DETECTED | SX127x_IRQ_CAD_DONE | SX127x_IRQ_RX_DONE | SX127x_IRQ_RX_TIMEOUT)
           && enable_irq(radio, SX127x_IRQ_TX_DONE)
           && radio->write_register(radio, SX127x_REG_DIO_MAPPING_1, TX_DIO_MAPPING)
           && radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_TX)
            ;

        release_lock(radio);
    }

    return ok;
}


/*
 * Set power from -15 to +17 dbm.  Chose PA mode as apprpriate
 */
static bool set_txpower(radio_t* radio, int power)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: power %d", __func__, power);

    if (acquire_lock(radio)) {
        sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

        if (power < -4) {
            power = -4;
        } else if (power > 17) {
            power = 17;
        }

        data->tx_power = power;

        if (power >= 2) {
            /* BOOST mode on */
            radio->write_register(radio, SX127x_REG_PA_CONFIG, SX127x_PA_BOOST | (power - 2));
#ifdef NOTUSED
/* TEST */
            radio->read_register(radio, SX127x_REG_PA_CONFIG);
/* END TEST */
#endif

        } else {
            /* BOOST mode off (close enough) */
            power -= 4;
            radio->write_register(radio, SX127x_REG_PA_CONFIG, power);
        }

        release_lock(radio);

        ok = true;
    }

    return ok;
}

static int get_txpower(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->tx_power;
}

/*
 * set_channel
 *
 * selects the channel and sets the datarate to the lowest for the channel.
 *
 * Returns old channel if successful else -1.
 */
static int set_channel(radio_t* radio, int channel)
{
    int old_channel = -1;

ESP_LOGI(TAG, "%s: channel %d", __func__, channel);

    if (channel >= 0 && channel < ELEMENTS_OF(channel_table.channels)) {

        old_channel = get_channel(radio);

        if (acquire_lock(radio)) {

            set_sleep_mode(radio);

            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

            const channel_entry_sx127x_t *chanp = &channel_table_sx127x.channels[channel];


            /* Set frequency control */
            radio->write_register(radio, SX127x_REG_FREQ_MSB, chanp->freq_high);
            radio->write_register(radio, SX127x_REG_FREQ_MID, chanp->freq_mid);
            radio->write_register(radio, SX127x_REG_FREQ_LSB, chanp->freq_low);

//#ifdef NOTUSED
/*TEST*/
            radio->read_register(radio, SX127x_REG_FREQ_MSB);
            radio->read_register(radio, SX127x_REG_FREQ_MID);
            radio->read_register(radio, SX127x_REG_FREQ_LSB);
/*END TEST*/
//#endif

            radio->set_datarate(radio, 0);
            //set_datarate(radio, 3);

            radio->set_txpower(radio, channel_table_sx127x.datarates[chanp->datarate_group][0].tx);

            data->channel = channel;

            radio->set_receive_mode(radio);

            release_lock(radio);
        }
    }

    return old_channel;
}

static int get_channel(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->channel;
}

/*
 * Set the datarate.
 *
 * Return old datarate if successful, otherwise -1.
 */
static int set_datarate(radio_t* radio, int datarate)
{
    int old_datarate = -1;

    if (acquire_lock(radio)) {
        old_datarate = get_datarate(radio);

        sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

        int datarate_group = channel_table_sx127x.channels[data->channel].datarate_group;

        const datarate_entry_sx127x_t* dataratep = channel_table.datarates[datarate_group];

        if (datarate >= 0 && datarate < ELEMENTS_OF(channel_table.datarates[datarate_group]) && dataratep[datarate].payload != 0) {
            int sf = dataratep[datarate].sf;
            int bw = dataratep[datarate].bw;
            int cr = dataratep[datarate].cr;
            int tx = dataratep[datarate].tx;


            if (!set_bandwidth(radio, bw) || !set_spreading_factor(radio, sf) || !set_coding_rate(radio, cr) || !set_txpower(radio, tx)) {
                /* Restore datarate (caution - recursion); better not be an error on the old datarate. */
                set_datarate(radio, data->datarate);
                old_datarate = -1;
            } else {
                /* Ok */
                data->datarate = datarate;
            }
        }

        release_lock(radio);
    }

    return old_datarate;
}

static int get_datarate(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->datarate;
}


/*
 *  Set bandwidth
 */
static bool set_bandwidth(radio_t* radio, int bw)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

ESP_LOGI(TAG, "%s: %d", __func__, bw);

        int bwcode = -1;
        int bwindex = 0;

        /* Look for bandwidth in code table */
        while (bwcode < 0 && bwindex < ELEMENTS_OF(channel_table_sx127x.bandwidth_bins)) {
            if (bw == channel_table_sx127x.bandwidth_bins[bwindex]) {
                bwcode = bwindex;
            } else {
                bwindex++;
            }
        }

        if (bwcode >= 0) {
          ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_1,
                                     (radio->read_register(radio, SX127x_REG_MODEM_CONFIG_1) & 0x0f) | (bwcode << 4));
        }

        if (ok) {
            data->bandwidth = bw;
        }
        release_lock(radio);
    }

    update_data_rate(radio);

    return ok;
}

#ifdef NOTUSED
static int get_bandwidth(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->bandwidth;
}
#endif

static bool set_spreading_factor(radio_t* radio, int spreading_factor)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: sf %d", __func__, spreading_factor);

    if (spreading_factor >= 6 && spreading_factor <= 12) {

        if (acquire_lock(radio)) {

            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

            data->spreading_factor = spreading_factor;

            /* Set 'low data rate' flag if long symbol time otherwise clear it */
            int config3 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_3);
            if (config3 >= 0) {

                if (1000.0 / ((float) data->bandwidth / (float) (1<<spreading_factor)) > -1) {
                    config3 |= 0x08;
                } else {
                    config3 &= ~0x08;
                }

                int config2 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_2);

                if (config2 >= 0) {

                    ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_3, config3)
                         && radio->write_register(radio, SX127x_REG_DETECTION_OPTIMIZE, (spreading_factor == 6) ? 0xc5 : 0xc3)
                         && radio->write_register(radio, SX127x_REG_DETECTION_THRESHOLD, (spreading_factor == 6) ? 0x0c : 0x0a)
                         && radio->write_register(radio, SX127x_REG_MODEM_CONFIG_2, (config2 & 0x0F) | (spreading_factor << 4));
                         ;

/*TEST*/
                    radio->read_register(radio, SX127x_REG_MODEM_CONFIG_2);  /* READBACK */
/* END TEST */
                }
            }

            release_lock(radio);
        }
    }

    update_data_rate(radio);

    return ok;
}

#ifdef NOTUSED
static int get_spreading_factor(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->spreading_factor;
}
#endif


static int set_coding_rate(radio_t* radio, int rate)
{
    bool ok = false;

    if (rate >= 5 && rate <= 8) {

ESP_LOGI(TAG, "%s: %d", __func__, rate);

        if (acquire_lock(radio)) {
            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;
            data->coding_rate = rate;

            int config1 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_1);
            if (config1 >= 0) {
                ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_1, (config1 & 0xF1) | ((rate - 4) << 1));
            }

            release_lock(radio);
        }
    }

    update_data_rate(radio);

    return ok;
}

#ifdef NOTUSED
static int get_coding_rate(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->coding_rate;
}
#endif



static int set_preamble_length(radio_t* radio, int length)
{
    bool ok = false;
ESP_LOGI(TAG, "%s: length %d", __func__, length);

    if (acquire_lock(radio)) {

        sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;
        data->preamble_length = length;

        ok = radio->write_register(radio, SX127x_REG_PREAMBLE_MSB, (length >> 8)) &&
             radio->write_register(radio, SX127x_REG_PREAMBLE_LSB, length);

        release_lock(radio);
    }

    update_data_rate(radio);

    return ok;
}

#ifdef NOTUSED
static int get_preamble_lengt(radio_t *radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->preamble_length;
}
#endif


static int set_enable_crc(radio_t* radio, bool enable)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: enable %s", __func__, enable ? "TRUE" : "FALSE");

    if (acquire_lock(radio)) {
        int config2 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_2);

        if (config2 >= 0) {
            if (enable) {
                config2 |= 0x04;
            } else {
                config2 &= ~0x04;
            }
            ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_2, config2);
#if 0
/*TEST*/
            /* Read back to verify */
            radio->read_register(radio, SX127x_REG_MODEM_CONFIG_2);
/*END TEST*/
#endif
        }

        release_lock(radio);
    }

    return ok;
}

static bool set_hop_period(radio_t* radio, int hop_period)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok = radio->write_register(radio, SX127x_REG_HOP_PERIOD, hop_period);
        release_lock(radio);
    }

    return ok;
}

static bool set_sync_word(radio_t* radio, uint8_t sync)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: sync %02x", __func__, sync);

    if (acquire_lock(radio)) {
        ok = radio->write_register(radio, SX127x_REG_SYNC_WORD, sync);
        release_lock(radio);
    }

    return ok;
}


static bool set_implicit_header(radio_t* radio, bool implicit_header)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: enable %s", __func__, implicit_header ? "TRUE" : "FALSE");

    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (acquire_lock(radio)) {
        int config1 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_1);
        if (config1 >= 0) {
            if (implicit_header) {
                config1 |= 0x01;
            } else {
                config1 &= ~0x01;
            }

            ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_1, config1);
        }
        data->implicit_header = implicit_header;
        data->implicit_header_set = true;
        release_lock(radio);
    }

    return ok;
}

/*
 * Calculates BPS for selected data rate
 */
static void update_data_rate(radio_t *radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    data->data_rate_bps = (data->spreading_factor * 4 * data->bandwidth) / (data->coding_rate * (1 << data->spreading_factor));

    /* Set the width of the transmit window in milliseconds */
    data->window_width = (get_message_time(radio, MAX_PACKET_LEN) * radio->window_width_percent) / 100;

    /* Change the window update rate.  Things may be a bit wonky until the first packet is received */
    os_set_timer(data->window_timer_id, data->window_width);
}

/*
 * Return message time in milliseconds for a packet of given length in bytes (plus preamble overhead.)
 */
static int get_message_time(radio_t* radio, int length)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    int time = ((length + data->preamble_length + LORA_HEADER_OVERHEAD) * 8 * 1000) / data->data_rate_bps;

//ESP_LOGI(TAG, "%s: sf %d bw %d length %d is %d", __func__, get_spreading_factor(radio), get_bandwidth(radio), length, time);
    return time;
}


/*
 * All interrupts come here and are queued for later processing by a global handling thread.
 */
static void catch_interrupt(void *param)
{            
    radio_t *radio = (radio_t*) param;

    bool awakened;

    handler_queue_event_t event = {
        .radio = radio,
        .type = HQI_INTERRUPT,
    };

    os_put_queue_from_isr(global_interrupt_handler_queue, (os_queue_item_t) &event, &awakened);

    if (awakened) {
        portYIELD_FROM_ISR();
    }
}

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
void print_status(radio_t *radio)
{
    sx127x_private_data_t data;

    bool ok = false;

    int live_irq_flags;
    int live_irq_mask;

    if (acquire_lock(radio)) {
        memcpy(&data, radio->driver_private_data, sizeof(sx127x_private_data_t));
        live_irq_flags = radio->read_register(radio, SX127x_REG_IRQ_FLAGS);
        live_irq_mask = radio->read_register(radio, SX127x_REG_IRQ_FLAGS_MASK);
        release_lock(radio);
        ok = true;
    }

    if (ok) {
        printf("\nRadio %d:\n", radio->radio_num);
        printf("Transmit queue length:  %d\n", os_items_in_queue(radio->transmit_queue));
        printf("Waiting output:         %s\n", data.current_packet ? "YES" : "NO");
        printf("sync_word:              %02x\n", data.sync_word);
        printf("preamble_length:        %d\n", data.preamble_length);
        printf("coding_rate:            %d\n", data.coding_rate);
        printf("implicit_header:        %s\n", data.implicit_header ? "YES" : "NO");
        printf("hop_period:             %d\n", data.hop_period);
        printf("enable_crc:             %s\n", data.enable_crc ? "YES" : "NO");
        printf("channel:                %d\n", data.channel);
        printf("datarate:               %d\n", data.datarate);
        printf("bandwidth:              %d\n", data.bandwidth);
        printf("spreading_factor:       %d\n", data.spreading_factor);
        printf("tx_power:               %d\n", data.tx_power);
        printf("rx_interrupts:          %d\n", data.rx_interrupts);
        printf("rx_timeouts:            %d\n", data.rx_timeouts);
        printf("tx_interrupts:          %d\n", data.tx_interrupts);
        printf("tx_timeouts:            %d\n", data.tx_timeouts);
        printf("cad_interrupts:         %d\n", data.cad_interrupts);
        printf("cad_detected:           %d\n", data.cad_detected);
        printf("cad_timeouts:           %d\n", data.cad_timeouts);
#ifdef USE_FHSS
        printf("fhss_interrupts:        %d\n", data.fhss_interrupts);
#endif /* USE_FHSS */
        printf("packet_memory_failed:   %d\n", data.packet_memory_failed);
        printf("packet_crc_errors:      %d\n", data.packet_crc_errors);
        printf("saved irq_flags:        %02x\n", data.irq_flags);
        printf("live irq_flags:         %02x\n", live_irq_flags);
        printf("live irq_mask:          %02x\n", live_irq_mask);
        printf("handler_state:          %s\n", handler_state_of(data.handler_state));
        printf("handler_cycles:         %d\n", data.handler_cycles);
        printf("wakeup_ticks:           %d\n", data.wakeup_ticks);
        printf("window_number:          %d\n", data.window_number);
        printf("current_packet_window:  %d\n", data.current_packet_window);
        printf("window_width:           %d\n", data.window_width);
        printf("window_event_count:     %d\n", data.window_event_count);

        int wakeup_timer_remaining = simpletimer_remaining(&data.handler_wakeup_timer_status);

        if (simpletimer_is_running(&data.handler_wakeup_timer_status)) {
            printf("wakeup_timer:           %d\n", wakeup_timer_remaining);
        } else {
            printf("wakeup_timer:           %s\n", simpletimer_is_expired(&data.handler_wakeup_timer_status) ? "expired" : "stopped");
        }
    } else {
        printf("Unable to lock data\n");
    }
}
#endif

#endif

