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
static bool transmit_packet(radio_t* radio, packet_t* packet);       /* Send packet immediately */

static void catch_interrupt(void *param);
static void rx_handle_interrupt(radio_t* radio);
static void fhss_handle_interrupt(radio_t* radio);
static void tx_handle_interrupt(radio_t* radio);
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
    bool       cad_detected;  /* Shows state of current CAD detected state */
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
    uint8_t    interrupt_flags;
    int        packet_memory_failed;
} sx127x_private_data_t;

static os_thread_t  global_interrupt_handler_thread;
static os_queue_t   global_interrupt_handler_queue;
static void         global_interrupt_handler(void* param);
static int          global_number_radios_active;

#define MAX_IRQ_PENDING         20

#define FHSS_ENABLED            FALSE

#define RECEIVE_CADDETECTED_CADDONE_IRQ_DIO    0
#define TRANSMIT_FHSS_IRQ_DIO                  1

#define GLOBAL_IRQ_THREAD_STACK    16384
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
        radio->transmit_packet  = transmit_packet;

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
            data->spreading_factor          = 7;
            data->tx_power                  = 2;
            data->rx_interrupts             = 0;
            data->tx_interrupts             = 0;
            data->rlock                     = os_create_recursive_mutex();
            data->interrupt_flags           = 0;
        }

        /* Start global interrupt processing thread if not yet running */
        if (global_interrupt_handler_thread == NULL) {
            global_interrupt_handler_queue = os_create_queue(MAX_IRQ_PENDING, sizeof(radio_t*));
            global_interrupt_handler_thread = os_create_thread(global_interrupt_handler, "sx127x_irq_thread",
                                                               GLOBAL_IRQ_THREAD_STACK, GLOBAL_IRQ_THREAD_PRIORITY, NULL);
        }

        ++global_number_radios_active;
    }

	return radio != NULL && radio->driver_private_data != NULL && radio_start(radio);
}

/************************************************************************************
 *    Thread base interrupt handler
 *
 *    As interrupts are detected, flags are logged in data->interrupt_flags
 *    and the radio element is queued to the global_interrupt_handler_queue.
 *
 *    This thread processes these elements one at a time and processes each
 *    IRQ logged in the interrupt_flags.
 *
 *    Errors are reported if the interrupts are not cleared.
 ************************************************************************************/
static void rx_handle_interrupt(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (data->interrupt_flags & SX127x_IRQ_RX_DONE) {
        data->interrupt_flags &= ~SX127x_IRQ_RX_DONE;

        data->rx_interrupts++;

        int length;

        radio->write_register(radio, SX127x_REG_FIFO_PTR, radio->read_register(radio, SX127x_REG_RX_FIFO_CURRENT));
        if (data->implicit_header) {
            length = radio->read_register(radio, SX127x_REG_PAYLOAD_LENGTH);
        } else {
            length = radio->read_register(radio, SX127x_REG_RX_NUM_BYTES);
        }

        /* Get a packet */
        packet_t* packet = allocate_packet();
        if (packet != NULL) {
            if (radio->read_buffer(radio, SX127x_REG_FIFO, packet->buffer, length)) {
                /* Buffer has been read.  Set length */
                packet->length = length;
                packet->crc_ok = (data->interrupt_flags & SX127x_IRQ_PAYLOAD_CRC_ERROR) == 0;
                packet->rssi = get_packet_rssi(radio);
                packet->radio_num = radio->radio_num;

                /* Pass it to protocol layer */
                radio->on_receive(radio, ref_packet(packet));
            }

            release_packet(packet);
        } else {
             data->packet_memory_failed++;
        }
    }
}

/* THIS NEEDS WORK */
static void fhss_handle_interrupt(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (data->interrupt_flags & SX127x_IRQ_FHSS_CHANGE_CHANNEL) {
        data->interrupt_flags &= ~SX127x_IRQ_FHSS_CHANGE_CHANNEL;

#ifdef NOTUSED
        radio->write_register(radio, SX127x_FHSS_HOP_CHANNEL, next_channel);
#endif
        data->fhss_interrupts++;
    }
}


static void tx_handle_interrupt(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (data->interrupt_flags & SX127x_IRQ_TX_DONE) {
        data->interrupt_flags &= ~SX127x_IRQ_TX_DONE;

        data->tx_interrupts++;

        /* Discard current queue entry and get next packet to send */
        packet_t* packet = radio->on_transmit(radio);
        if (packet != NULL) {
            /* Pause if CAD is detected */
            while (data->cad_detected) {
                os_delay(1);
            }
            transmit_packet(radio, packet);
        } else {
            /* Other return to receive mode. */
            set_receive_mode(radio);
        }
    }
}


/* Param is ignored */
static void global_interrupt_handler(void* param)
{
    bool running = true;

    while (running) {
        radio_t* radio;

        if (os_get_queue(global_interrupt_handler_queue, (os_queue_item_t*) &radio)) {

            if (acquire_lock(radio)) {
                sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

                if (data->interrupt_flags & SX127x_IRQ_FHSS_CHANGE_CHANNEL) {
                    fhss_handle_interrupt(radio);
                }

                if (data->interrupt_flags & SX127x_IRQ_CAD_DONE) {
                    /* Turn off CAD detect flag */
                    data->cad_detected = false;
                    ESP_LOGI(TAG, "CAD_DONE detected");
                    data->interrupt_flags &= ~SX127x_IRQ_CAD_DONE;
                }

                if (data->interrupt_flags & SX127x_IRQ_CAD_DETECTED) {
                    /* Turn on CAD detect flag */
                    data->cad_detected = true;
                    ESP_LOGI(TAG, "CAD detected");
                    data->interrupt_flags &= ~SX127x_IRQ_CAD_DETECTED;
                }

                if (data->interrupt_flags & SX127x_IRQ_RX_DONE) {
                    rx_handle_interrupt(radio);
                }

                if (data->interrupt_flags & SX127x_IRQ_TX_DONE) {
                    tx_handle_interrupt(radio);
                }

                if (data->interrupt_flags != 0) {
                    ESP_LOGE(TAG, "Interrupt not processed: %02x on radio %d", data->interrupt_flags, radio->radio_num);
                    data->interrupt_flags = 0;
                }

                release_lock(radio);
            }
        } else {
            /* NULL delivered - shut down */
            running = false;
        }
    }
}

static bool radio_stop(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        /* Disable all interrupts */
        ok = disable_irq(radio, 0xFF)
             && radio->attach_interrupt(radio, RECEIVE_CADDETECTED_CADDONE_IRQ_DIO, DISABLED, NULL)
             && radio->attach_interrupt(radio, TRANSMIT_FHSS_IRQ_DIO, DISABLED, NULL)
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

    return os_acquire_recursive_mutex(data->rlock);
}

static bool release_lock(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

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

        // sx127x_reset();

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
            /* Put receiver in sleep */
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

            /* Configure the unit for receive channel 0; probably overriden by caller */
            set_channel(radio, 0);

            /* LNA Boost */
            radio->write_register(radio, SX127x_REG_LNA, radio->read_register(radio, SX127x_REG_LNA) | 0x03);

            /* auto AGC enable */
            radio->write_register(radio, SX127x_REG_MODEM_CONFIG_3, 0x04);

            radio->write_register(radio, SX127x_REG_TX_FIFO_BASE, TX_FIFO_BASE);
            radio->write_register(radio, SX127x_REG_RX_FIFO_BASE, RX_FIFO_BASE);

            /* Mask all IRQs */
            disable_irq(radio, 0xFF);

            /* Clear all interrupts */
            radio->write_register(radio, SX127x_REG_IRQ_FLAGS, 0xFF);

            /* Capture receive/transmit interrupts */
            radio->attach_interrupt(radio, RECEIVE_CADDETECTED_CADDONE_IRQ_DIO, RISING, catch_interrupt);
            radio->attach_interrupt(radio, TRANSMIT_FHSS_IRQ_DIO, RISING, catch_interrupt);

            /* Enable interrupts of interest */
            enable_irq(radio, SX127x_IRQ_TX_DONE | SX127x_IRQ_RX_DONE | SX127x_IRQ_CAD_DONE | SX127x_IRQ_CAD_DETECTED);

            if (data->hop_period != 0) {
                /* Enable FHSS interrupt */
                enable_irq(radio, SX127x_IRQ_FHSS_CHANGE_CHANNEL);
            }

            radio->set_receive_mode(radio);

            ESP_LOGI(TAG, "SX127x radio started");

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

static int get_packet_snr(radio_t* radio)
{
    int snr = -9999;

    if (acquire_lock(radio)) {
        snr = radio->read_register(radio, SX127x_REG_PACKET_SNR) / 4.0;
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

    if (acquire_lock(radio)) {
        ok =  radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_RX_CONTINUOUS)
           && radio->write_register(radio, SX127x_REG_DIO_MAPPING_1, 0b00000000)
           ;

        release_lock(radio);
    }

    return ok;
}

static bool set_transmit_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok =   radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_TX)
            && radio->write_register(radio, SX127x_REG_DIO_MAPPING_1, 0b01000000)
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

    if (channel >= 0 && channel < ELEMENTS_OF(channel_table.channels)) {
        if (acquire_lock(radio)) {
            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

            const channel_entry_sx127x_t *chanp = &channel_table_sx127x.channels[channel];


            /* Set frequency control */
            radio->write_register(radio, SX127x_REG_FREQ_MSB, chanp->freq_high);
            radio->write_register(radio, SX127x_REG_FREQ_MSB, chanp->freq_mid);
            radio->write_register(radio, SX127x_REG_FREQ_MSB, chanp->freq_low);

            set_datarate(radio, 0);
            set_txpower(radio, channel_table_sx127x.datarates[chanp->datarate_group][0].tx);

            data->channel = channel;

            ok = true;

            release_lock(radio);
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

    if (spreading_factor >= 6 && spreading_factor <= 12) {

        if (acquire_lock(radio)) {

            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;
            /* Set 'low data rate' flag if long symbol time otherwise clear it */
            int config3 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_3);
            if (config3 >= 0) {

                if (1000.0 / ((float) data->bandwidth / (float) (1<<data->spreading_factor)) > -1) {
                    config3 |= 0x08;
                } else {
                    config3 &= ~0x08;
                }

                int config2 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_2) & 0x0F;

                if (config2 >= 0) {

                    ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_3, config3)
                         && radio->write_register(radio, SX127x_REG_DETECTION_OPTIMIZE, (data->spreading_factor == 6) ? 0xc5 : 0xc3)
                         && radio->write_register(radio, SX127x_REG_DETECTION_THRESHOLD, (data->spreading_factor == 6) ? 0x0c : 0x0a)
                         && radio->write_register(radio, SX127x_REG_MODEM_CONFIG_2, config2 | ((data->spreading_factor << 4) & 0x0F))
                         ;
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

    if (acquire_lock(radio)) {
        int config2 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_2);

        if (config2 >= 0) {
            if (enable) {
                config2 |= 0x04;
            } else {
                config2 &= ~0x04;
            }
            ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_2, config2);
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

    if (acquire_lock(radio)) {
        ok = radio->write_register(radio, SX127x_REG_SYNC_WORD, sync);
        release_lock(radio);
    }

    return ok;
}


static bool set_implicit_header(radio_t* radio, bool implicit_header)
{
    bool ok = false;

    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (implicit_header == data->implicit_header && data->implicit_header_set) {
        ok = true;
    } else if (acquire_lock(radio)) {
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
    radio_t* radio = (radio_t*) param;
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    /* Clear interrupt */
    int flags = radio->read_register(radio, SX127x_REG_IRQ_FLAGS);
    radio->write_register(radio, SX127x_REG_IRQ_FLAGS, flags);

    /* Save flags that have occurred */
    data->interrupt_flags |= flags;

    os_put_queue_from_isr(global_interrupt_handler_queue, (os_queue_item_t) radio);
}

static bool start_packet(radio_t* radio)
{
    return set_standby_mode(radio) &&
           radio->write_register(radio, SX127x_REG_FIFO_PTR, TX_FIFO_BASE) &&
           radio->write_register(radio, SX127x_REG_PAYLOAD_LENGTH, 0);
}

static bool write_packet(radio_t* radio, packet_t* packet)
{
    bool ok = false;

    /* Read how much is in the fifo */
    int current = radio->read_register(radio, SX127x_REG_PAYLOAD_LENGTH);
    if (current >= 0) {

        /* Calculate how much room */
        int size = packet->length - TX_FIFO_BASE - current;

        if (packet->length < size) {
            size = packet->length;
        }

        radio->write_buffer(radio, SX127x_REG_FIFO, packet->buffer, size);

        /* Set the new length of the payload in process */
        radio->write_register(radio, SX127x_REG_PAYLOAD_LENGTH, current + size);

        ok = true;
    }

    return ok;
}

static bool transmit_packet(radio_t* radio, packet_t* packet)
{
    bool ok = false;

    if (acquire_lock(radio)) {

        start_packet(radio);
        write_packet(radio, packet);
        set_transmit_mode(radio);

        release_lock(radio);

        ok = true;
    }

    return ok;
}

#endif

