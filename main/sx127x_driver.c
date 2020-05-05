#include "sdkconfig.h"

/*
 * Driver for sx127x chip.
 */
#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED

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
#include "os_freertos.h"
#include "sx127x_driver.h"
#include "linklayer.h"

#define TAG "sx127x_driver"

static bool radio_stop(radio_t* radio);                              /* Stop and disassemble the radio */
static bool set_sleep_mode(radio_t* radio);                          /* Set sleeping mode (low power) */
static bool set_standby_mode(radio_t* radio);                        /* Set standby mode */
static bool set_receive_mode(radio_t* radio);                        /* Set active receiving mode */
static bool set_txpower(radio_t* radio, int power);                  /* Set transmitter power level */
static int get_txpower(radio_t* radio);                              /* Get current transmit power */
static bool set_channel(radio_t* radio, int channel);                /* Set channel */
static int get_channel(radio_t* radio);                              /* Get channel */
static bool set_datarate(radio_t* radio, int datarate);              /* Set datarate */
static int get_datarate(radio_t* radio);                             /* Get datarate */

static void catch_interrupt(void *param);
static bool rx_handle_interrupt(radio_t* radio);
static bool fhss_handle_interrupt(radio_t* radio);
static bool tx_handle_interrupt(radio_t* radio, bool from_interrupt);
static void transmit_start(radio_t *radio);
static void global_interrupt_handler(void* param);
static bool radio_stop(radio_t* radio);
static bool release_lock(radio_t* radio);
static bool radio_start(radio_t* radio);
static int  get_packet_snr(radio_t* radio);
static bool set_transmit_mode(radio_t* radio);
static int get_packet_rssi(radio_t* radio);
static bool acquire_lock(radio_t* radio);
static bool release_lock(radio_t* radio);
static bool set_bandwidth(radio_t* radio, int bw);
static bool set_spreading_factor(radio_t*, int spreading_factor);
static int  get_spreading_factor(radio_t* radio);
static int set_coding_rate(radio_t* radio, int rate);
static int set_preamble_length(radio_t* radio, int length);
static int set_enable_crc(radio_t* radio, bool enable);
static bool set_hop_period(radio_t* radio, int hop_period);
static bool set_implicit_header(radio_t* radio, bool implicit_header);
static bool set_sync_word(radio_t* radio, uint8_t sync);
static bool enable_irq(radio_t* radio, uint8_t mask);
static bool disable_irq(radio_t* radio, uint8_t mask);

/* Define the selected frequency domain */
#include "sx127x_table.h"

/* Define private data structure */
typedef struct sx127x_private_data {
    uint8_t    sync_word;
    uint8_t    preamble_length;
    uint8_t    coding_rate;
    bool       implicit_header;
    bool       implicit_header_set;
    int        hop_period;
    bool       enable_crc;
    int        channel;
    int        datarate;
    int        bandwidth;
    uint8_t    spreading_factor;
    uint8_t    tx_power;
    int        rx_interrupts;
    int        tx_interrupts;
    int        fhss_interrupts;
    os_mutex_t rlock;
    int        packet_memory_failed;
    int        packet_crc_errors;
    uint16_t   irq_flags;                      /* Bottom 8 are hardware flags acquired at irq scan time */
#define SX127x_FORCE_START_TRANSMIT   0x100    /* Extra 'irq flag' for starting new transmit activity */
} sx127x_private_data_t;

static os_thread_t  global_interrupt_handler_thread;
static os_queue_t   global_interrupt_handler_queue;
static void         global_interrupt_handler(void* param);
static int          global_number_radios_active;

#define MAX_IRQ_PENDING         50

#define FHSS_ENABLED            FALSE

#define RECEIVE_IRQ_DIO                 0
#define TRANSMIT_FHSS_TIMEOUT_IRQ_DIO   1

#define GLOBAL_IRQ_THREAD_STACK    32768
#define GLOBAL_IRQ_THREAD_PRIORITY (configMAX_PRIORITIES-1)  /* Highest priority */

#define WANTED_VERSION  0x12

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
        /* Add callouts into the driver.  All are called with radio as first parameter */
        radio->stop             = radio_stop;
        radio->set_sleep_mode   = set_sleep_mode;
        radio->set_standby_mode = set_standby_mode;
        radio->set_receive_mode = set_receive_mode;
        radio->set_txpower      = set_txpower;
        radio->get_txpower      = get_txpower;
        radio->set_channel      = set_channel;
        radio->get_channel      = get_channel;
        radio->set_datarate     = set_datarate;
        radio->get_datarate     = get_datarate;
        radio->transmit_start   = transmit_start;

        /* Allocate a data block for local data */
        radio->driver_private_data = malloc(sizeof(sx127x_private_data_t));
        if (radio->driver_private_data != NULL) {
            memset(radio->driver_private_data, 0, sizeof(sx127x_private_data_t));

            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

            /* Initialize defaults */
            data->sync_word                 = 0x34;
            data->preamble_length           = 8;
            data->coding_rate               = 5;
            data->implicit_header           = false;
            data->implicit_header_set       = false;
            data->hop_period                = 0;
            data->enable_crc                = true;
            data->bandwidth                 = 125000;
            data->spreading_factor          = 10;
            data->tx_power                  = 2;
            data->rx_interrupts             = 0;
            data->tx_interrupts             = 0;
            data->rlock                     = os_create_recursive_mutex();
        }

        /* Start global interrupt processing thread if not yet running */
        if (global_interrupt_handler_thread == NULL) {
            global_interrupt_handler_queue = os_create_queue(MAX_IRQ_PENDING, sizeof(radio_t*));
            global_interrupt_handler_thread = os_create_thread_on_core(global_interrupt_handler, "sx127x_svc",
                                                               GLOBAL_IRQ_THREAD_STACK, GLOBAL_IRQ_THREAD_PRIORITY, NULL, 0);
        }

        ++global_number_radios_active;
    }

	return radio != NULL && radio->driver_private_data != NULL && radio_start(radio);
}

/************************************************************************************
 *    Thread base interrupt handler
 *
 *    As interrupts are detected, flags are passed in parameters
 *    and the radio element is queued to the global_interrupt_handler_queue.
 *
 *    This thread processes these elements one at a time and processes each
 *    passing, the radio and irq flags value at time of interrupt.
 *
 *    Errors are reported if the interrupts are not cleared.
 ************************************************************************************/
static bool rx_handle_interrupt(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    data->rx_interrupts++;

    int length;

    radio->write_register(radio, SX127x_REG_FIFO_PTR, radio->read_register(radio, SX127x_REG_RX_FIFO_CURRENT));

    if (data->implicit_header) {
        length = radio->read_register(radio, SX127x_REG_PAYLOAD_LENGTH);
    } else {
        length = radio->read_register(radio, SX127x_REG_RX_NUM_BYTES);
    }

    /* Cannot reliably capture packet if CRC error so don't even try */
    if ((data->irq_flags & (SX127x_IRQ_VALID_HEADER | SX127x_IRQ_PAYLOAD_CRC_ERROR | SX127x_IRQ_RX_TIMEOUT)) == 0) {
        /* Get a packet */
        packet_t* packet = allocate_packet();

        if (packet != NULL) {

            if (radio->read_buffer(radio, SX127x_REG_FIFO, packet->buffer, length)) {
                /* Buffer has been read.  Set length */
                packet->length = length;
                packet->crc_ok = true;
                packet->rssi = get_packet_rssi(radio);
                packet->snr = get_packet_snr(radio);
                packet->radio_num = radio->radio_num;

ESP_LOGI(TAG, "%s: packet len %d rssi %d radio %d", __func__, packet->length, packet->rssi, packet->radio_num);

                set_standby_mode(radio);
                set_receive_mode(radio);

                /* Pass it to protocol layer */
                radio->on_receive(radio, ref_packet(packet));
            }

            release_packet(packet);

        } else {
            data->packet_memory_failed++;
        }
    } else if ((data->irq_flags & SX127x_IRQ_PAYLOAD_CRC_ERROR) != 0) {
       set_standby_mode(radio);
       radio->write_register(radio, SX127x_REG_FIFO_PTR, RX_FIFO_BASE);
       set_receive_mode(radio);
       data->packet_crc_errors++;
    }

    return true;
}

/* THIS NEEDS WORK */
static bool fhss_handle_interrupt(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

#ifdef NOTUSED
    radio->write_register(radio, SX127x_FHSS_HOP_CHANNEL, next_channel);
#endif
    data->fhss_interrupts++;

    return true;
}

static bool radio_busy(radio_t *radio)
{
    return (radio->read_register(radio, SX127x_REG_MODEM_STATUS) & SX127x_MODEM_STATUS_HEADER_VALID) != 0;
}

//  static void wait_if_radio_busy(radio_t *radio, int delay_seed)
//  {
//      /* Wait 0 to 50 ms */
//      int r = ((esp_random() + delay_seed) % 6) * 20 + 50;
//  ESP_LOGI(TAG, "%s: r wait %d", __func__, r);
//      os_delay(r);
//  
//      uint8_t status = radio->read_register(radio, SX127x_REG_MODEM_STATUS);
//      if ((status & SX127x_MODEM_STATUS_HEADER_VALID) != 0) {
//          ESP_LOGI(TAG, "************************************ radio %d waiting %02x...", radio->radio_num, status);
//          while ((radio->read_register(radio, SX127x_REG_MODEM_STATUS) & SX127x_MODEM_STATUS_HEADER_VALID) != 0) {
//              os_delay(1);
//              //os_delay(((esp_random() + delay_seed) % 10 + 1) * 50);
//          }
//          ESP_LOGI(TAG, "************************************ radio %d finished waiting", radio->radio_num);
//      }
//  }

static bool start_packet(radio_t* radio)
{
    return set_standby_mode(radio)
           && radio->write_register(radio, SX127x_REG_FIFO_PTR, TX_FIFO_BASE)
           && radio->write_register(radio, SX127x_REG_PAYLOAD_LENGTH, 0)
           ;
}

static bool write_packet(radio_t* radio, packet_t* packet)
{
    bool ok = false;

    int current = radio->read_register(radio, SX127x_REG_PAYLOAD_LENGTH);
    if (current != 0) {
        ESP_LOGE(TAG, "%s: PAYLOAD_LENGTH is not 0: %d", __func__, current);
        /* Reset payload length */
        radio->write_register(radio, SX127x_REG_PAYLOAD_LENGTH, 0);
    }

    radio->write_buffer(radio, SX127x_REG_FIFO, packet->buffer, packet->length);

    /* Set the new length of the payload in process */
    radio->write_register(radio, SX127x_REG_PAYLOAD_LENGTH, packet->length);

    ok = true;

    return ok;
}

static bool tx_handle_interrupt(radio_t* radio, bool from_interrupt)
{
    bool ok = false;

    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

ESP_LOGI(TAG, "%s", __func__);

    ok = true;
 
    if (from_interrupt) {
        data->tx_interrupts++;
        radio->activity_indicator(radio, false);
    }

    /* Discard current queue entry and get next packet to send */
    packet_t* packet = radio->on_transmit(radio, ! from_interrupt);

    if (packet != NULL) {

        ESP_LOGI(TAG, "%s: modem %02x", __func__, radio->read_register(radio, SX127x_REG_MODEM_STATUS));

        os_delay((esp_random() % 4 + 1) * 150);

        start_packet(radio);
        write_packet(radio, packet);
        set_transmit_mode(radio);

        radio->activity_indicator(radio, true);

    } else {

//ESP_LOGI(TAG, "%s: no packet", __func__);

        /* Other return to receive mode. */
        set_receive_mode(radio);
    }

    return ok;
}


/* Force start transmit interrupt handler */
static void transmit_start(radio_t *radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (acquire_lock(radio)) {

        data->irq_flags |= SX127x_FORCE_START_TRANSMIT;
        os_put_queue(global_interrupt_handler_queue, (os_queue_item_t) radio);
        release_lock(radio);

    } else {
        ESP_LOGE(TAG, "%s: Cannot acquire lock", __func__);
    }
}

/* Param is ignored */
static void global_interrupt_handler(void* param)
{
    bool running = true;

    ESP_LOGD(TAG, "%s: running", __func__);

    while (running) {
        radio_t *radio;

        if (os_get_queue(global_interrupt_handler_queue, (os_queue_item_t*) &radio)) {

//ESP_LOGI(TAG, "%s: got %p", __func__, radio);

            if (radio != NULL) {
                sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

                if (acquire_lock(radio)) {
                    /* Clear interrupt */
                    data->irq_flags |= radio->read_register(radio, SX127x_REG_IRQ_FLAGS);
                    radio->write_register(radio, SX127x_REG_IRQ_FLAGS, data->irq_flags & 0xFF);

                    while (data->irq_flags != 0) {

//ESP_LOGI(TAG, "%s: ******************************************* radio %d flags %02x", __func__, radio->radio_num, data->irq_flags);

                        /* Only process one flag per scan - in priority order */
                        if (data->irq_flags & SX127x_IRQ_FHSS_CHANGE_CHANNEL) {
                            if (fhss_handle_interrupt(radio)) {
                                data->irq_flags &= ~SX127x_IRQ_FHSS_CHANGE_CHANNEL;
                            }
                        }

#ifdef NOTUSED
                        else if (data->irq_flags & SX127x_IRQ_CAD_DONE) {
                            /* Turn off CAD detect flag */
                            data->cad_detected = false;
                            ESP_LOGD(TAG, "%s: CAD_DONE detected", __func__);
                            data->irq_flags &= ~SX127x_IRQ_CAD_DONE;
                        }

                        else if (data->irq_flags & SX127x_IRQ_CAD_DETECTED) {
                            /* Turn on CAD detect flag */
                            data->cad_detected = true;
                            ESP_LOGD(TAG, "%s: CAD detected", __func__);
                            data->irq_flags &= ~SX127x_IRQ_CAD_DETECTED;
                        }
#endif
                        else if (data->irq_flags & (SX127x_IRQ_RX_DONE | SX127x_IRQ_RX_TIMEOUT)) {
                            if (rx_handle_interrupt(radio)) {
                                data->irq_flags &= ~(SX127x_IRQ_RX_DONE | SX127x_IRQ_PAYLOAD_CRC_ERROR | SX127x_IRQ_VALID_HEADER | SX127x_IRQ_RX_TIMEOUT);
                            }
                        }

                        else if (data->irq_flags & SX127x_FORCE_START_TRANSMIT) {
                            if (tx_handle_interrupt(radio, false)) {
                                /* Kill both flags at once */
                                data->irq_flags &= ~(SX127x_FORCE_START_TRANSMIT | SX127x_IRQ_TX_DONE);
                            }
                        }

                        else if (data->irq_flags & SX127x_IRQ_TX_DONE ) {
                            if (tx_handle_interrupt(radio, true)) {
                                data->irq_flags &= ~SX127x_IRQ_TX_DONE;
                            }
                        }

                        else {
                            ESP_LOGE(TAG, "%s: Interrupt not processed: %04x on radio %d", __func__, data->irq_flags, radio->radio_num);
                            data->irq_flags = 0;   /* Remove unprocessed flags */
                        }
                    }

                    release_lock(radio);
                } else {
                    ESP_LOGE(TAG, "%s: unable to acquire lock", __func__);
                }
            } else {
                /* NULL delivered - shut down */
                running = false;
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

    if (acquire_lock(radio)) {
        /* Disable all interrupts */
        ok = disable_irq(radio, 0xFF)
             && radio->attach_interrupt(radio, RECEIVE_IRQ_DIO,               GPIO_PIN_INTR_DISABLE, NULL)
             && radio->attach_interrupt(radio, TRANSMIT_FHSS_TIMEOUT_IRQ_DIO, GPIO_PIN_INTR_DISABLE, NULL)
             ;

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

            radio->write_register(radio, SX127x_REG_TX_FIFO_BASE, TX_FIFO_BASE);
            radio->write_register(radio, SX127x_REG_RX_FIFO_BASE, RX_FIFO_BASE);


#ifdef NOTUSED
/* TEST */
            for (int bit = 0; bit < 8; ++bit) {
                radio->write_register(radio, SX127x_REG_IRQ_FLAGS_MASK, 1 << bit);
                radio->read_register(radio, SX127x_REG_IRQ_FLAGS_MASK);
            }
/* END TEST */
#endif

            /* Mask all IRQs */
            disable_irq(radio, 0xFF);

            /* Clear all interrupts */
            radio->write_register(radio, SX127x_REG_IRQ_FLAGS, 0xFF);
            /* Capture receive/transmit interrupts */
            radio->attach_interrupt(radio, RECEIVE_IRQ_DIO,               GPIO_PIN_INTR_POSEDGE, catch_interrupt);
            radio->attach_interrupt(radio, TRANSMIT_FHSS_TIMEOUT_IRQ_DIO, GPIO_PIN_INTR_POSEDGE, catch_interrupt);

            /* Enable interrupts of interest */
#ifdef NOTUSED
            enable_irq(radio, SX127x_IRQ_TX_DONE | SX127x_IRQ_RX_DONE | SX127x_IRQ_CAD_DONE | SX127x_IRQ_CAD_DETECTED);
#else
            //enable_irq(radio, SX127x_IRQ_TX_DONE | SX127x_IRQ_RX_DONE);
            //enable_irq(radio, SX127x_IRQ_RX_DONE);
#endif

            if (data->hop_period != 0) {
                /* Enable FHSS interrupt */
                enable_irq(radio, SX127x_IRQ_FHSS_CHANGE_CHANNEL);
            }

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
        ok = radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_STANDBY);
        release_lock(radio);
    }

    return ok;
}

static bool set_sleep_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok = radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_SLEEP);
        release_lock(radio);
    }

    return ok;
}


static bool set_receive_mode(radio_t* radio)
{
    bool ok = false;

//ESP_LOGI(TAG, "%s: set_receive_mode", __func__);

    if (acquire_lock(radio)) {
        ok =  disable_irq(radio, SX127x_IRQ_TX_DONE)
           && radio->write_register(radio, SX127x_REG_DIO_MAPPING_1, 0b00000000)
           && radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_RX_CONTINUOUS)
           && enable_irq(radio, SX127x_IRQ_RX_DONE) 
           ;

        //ESP_LOGI(TAG, "%s: radio %d status %02x", __func__, radio->radio_num, radio->read_register(radio, SX127x_REG_MODEM_STATUS));
        release_lock(radio);
    }

    return ok;
}

static bool set_transmit_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok =  disable_irq(radio, SX127x_IRQ_RX_DONE)
           && enable_irq(radio, SX127x_IRQ_TX_DONE) 
           && radio->write_register(radio, SX127x_REG_DIO_MAPPING_1, 0b01000000)
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
 * Returns true if successful.
 */
static bool set_channel(radio_t* radio, int channel)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: channel %d", __func__, channel);

    if (channel >= 0 && channel < ELEMENTS_OF(channel_table.channels)) {
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

            //set_datarate(radio, 0);
            set_datarate(radio, 3);

            set_txpower(radio, channel_table_sx127x.datarates[chanp->datarate_group][0].tx);

            data->channel = channel;

            set_receive_mode(radio);

            release_lock(radio);

            ok = true;
        }
    }

    return ok;
}

static int get_channel(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->channel;
}

static bool set_datarate(radio_t* radio, int datarate)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

        int datarate_group = channel_table_sx127x.channels[data->channel].datarate_group;

        const datarate_entry_sx127x_t* dataratep = channel_table.datarates[datarate_group];

        if (datarate >= 0 && datarate < ELEMENTS_OF(channel_table.datarates[datarate_group]) && dataratep[datarate].payload != 0) {
            int sf = dataratep[datarate].sf;
            int bw = dataratep[datarate].bw;
            int cr = dataratep[datarate].cr;
            int tx = dataratep[datarate].tx;

            data->datarate = datarate;

            ok = set_bandwidth(radio, bw) && set_spreading_factor(radio, sf) && set_coding_rate(radio, cr) && set_txpower(radio, tx);
        }
        release_lock(radio);
    }

    return ok;
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

    return ok;
}

static int get_bandwidth(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->bandwidth;
}

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

    return ok;
}

static int get_spreading_factor(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;
    return data->spreading_factor;
}


static int set_coding_rate(radio_t* radio, int rate)
{
    bool ok = false;

    if (rate >= 5 && rate <= 8) {

ESP_LOGI(TAG, "%s: %d", __func__, rate);

        if (acquire_lock(radio)) {
            int config1 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_1);
            if (config1 >= 0) {
                ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_1, (config1 & 0xF1) | ((rate - 4) << 1));
            }

            release_lock(radio);
        }
    }

    return ok;
}


static int set_preamble_length(radio_t* radio, int length)
{
    bool ok = false;
ESP_LOGI(TAG, "%s: length %d", __func__, length);

    if (acquire_lock(radio)) {

        ok = radio->write_register(radio, SX127x_REG_PREAMBLE_MSB, (length >> 8)) &&
             radio->write_register(radio, SX127x_REG_PREAMBLE_LSB, length);

        release_lock(radio);
    }

    return ok;
}

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
 * All interrupts come here and are queued for later processing by a global handling thread.
 */
static void catch_interrupt(void *param)
{
    bool awakened;
    os_put_queue_from_isr(global_interrupt_handler_queue, (os_queue_item_t) param, &awakened);

    if (awakened) {
        portYIELD_FROM_ISR();
    }
}

#endif

