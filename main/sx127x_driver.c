#include "sdkconfig.h"

/*
 * Driver for sx127x chip.
 */
#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/* Stop and disassemble the radio */
static bool radio_stop(radio_t* radio);

/* Set sleeping mode (low power) */
static bool set_sleep_mode(radio_t* radio);

/* Set standby mode */
static bool set_standby_mode(radio_t* radio);

/* Set active receiving mode */
static bool set_receive_mode(radio_t* radio);

/* Set transmitter power level */
static bool set_txpower(radio_t* radio, int power);

/* Get current transmit power */
static int get_txpower(radio_t* radio);

/* Set channel */
static bool set_channel(radio_t* radio, int channel);

/* Get channel */
static int get_channel(radio_t* radio);

/* Set datarate */
static bool set_datarate(radio_t* radio, int datarate);

/* Get datarate */
static int get_datarate(radio_t* radio);

static void (*transmit_packet)(radio_t* radio, packet_t* packet);

/* Define the selected frequency domain */
#include sx127x_table.h

/* Define private data structure */
typdef sx127x_private_data {
    uint8_t    sync_word;
    uint8_t    preamble_length;
    uint8_t    coding_rate;
    bool       implicit_header;
    bool       implicit_header_set;
#ifdef NOTUSED
    int        hop_period;
#endif
    bool       enable_crc;
    int        bandwidth;
    uint8_t    spreading_factor;
    uint8_t    tx_power;
    int        rx_interrupts;
    int        tx_interrupts:
    os_mutex_t rlock;
    uintt_t    interrupt_flags;
} sx127x_private_data_t;

os_thread_t  global_interrupt_handler_thread;
os_queue_t   global_interrupt_handler_queue;
int          global_number_of_active_radios;

#define FHSS_ENABLED            TRUE

#define RECEIVE_IRQ_DIO         0
#define TRANSMIT_IRQ_DIO        0
#ifdef FHSS_ENABLED
#define FHSS_IRQ_DIO            1
#endif

#define GLOBAL_IRQ_THREAD_STACK    16384
#define GLOBAL_IRQ_THREAD_PRIORITY 1

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
int
sx127x_create(radio_t* radio)
    if (radio != NULL) {
        radio->stop             = radio_stop;
        radio->set_sleep_mode = set_sleep_mode;
        radio->set_standby_mote = set_standy_mode;
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
            memset(radio-driver_private_data, 0, sizeof(sx127x_private_data_t));

            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

            /* Initialize defaults */
            data->sync_word                 = 0x34;
            data->preamble_length           = 8;
            data->coding_rate               = 5;
            data->implicit_header           = false;
            data->implicit_header_set       = false;
#ifdef NOTUSED
            data->hop_period                = 0;
#endif
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
            global_interrupt_handler_queue = os_create_queue();
            global_interrupt_handler_thread = os_create_thread(global_interrupt_handler, "sx127x_irq_thread",
                                                               GLOBAL_IRQ_THREAD_STACK, GLOBAL_IRQ_THREAD_PRIORITY, NULL);
        }

        ++global_number_of_radios_active;
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

        data->rx_interrupts += 1

        radio->write_register(radio, SX127x_REG_FIFO_PTR, radio->read_register(radio, SX127x_REG_RX_FIFO_CURRENT));
        if (data->implicit_header) {
            length = radio->read_register(radio, SX127x_REG_PAYLOAD_LENGTH);
        } else {
            length = radio->read_register(radio, SX127x_REG_RX_NUM_BYTES);

        /* Get a packet */
        packet = allocate_packet_from_isr();
        if (packet != NULL) {
            if (radio->read_buffer(SX127x_REF_FIFO, packet->buffer, length)) {
                /* Buffer has been read.  Set length */
                packet->length = length;
                packet->crc_ok = (flags & SX127x_IRQ_PAYLOAD_CRC_ERROR) == 0;
                packet->rssi = get_packet_rssi(radio);
                packet->radio_num = radio->radio_num;

                /* Pass it to protocol layer */
                radio->on_receive(radio, packet);
             } else {
                /* Failed to read - release packet */
                release_packet(packet);
             }
        } else {
             data->packet_memory_failed++;
        }
    }
}
 
#ifdef FHSS_ENABLED
/* THIS NEEDS WORK */
static void fhss_handle_interrupt(radio_t* radio)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (data->interrupt_flags & SX127x_IRQ_FHSS_DONE) {
        data->interrupt_flags &= ~SX127x_IRQ_FHSS_DONE;

        radio->write_register(radio, SX127x_FHSS_CHANNEL, next_channel);
        data->fhss_interrupts += 1;
    }
}
#endif


static void tx_handle_interrupt(void* param)
{
    radio_t* radio = (radio_t*) param;
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (data->interrupt_flags & SX127x_IRQ_TX_DONE) {
        data->interrupt_flags &= ~SX127x_IRQ_TX_DONE;

        data->_tx_interrupts += 1
        /* Discard current queue entry and get next packet to send */
        packet_t* packet = radio->on_transmit(radio);
        if (packet != NULL) {
            /* If delay has expired */
            transmit_packet(radio, packet)
        } else {
            /* Other return to receive mode. */
            set_receive_mode(radio);
        }
    }
}


/* Param is ignored */
void global_interrupt_handler(void* param)
{
    bool running = true;

    while (running) {
        radio_t* radio = (radio_t*) os_get_queue(global-interrupt_handler_queue);

        if (radio != NULL) {

            if (acquire_lock(radio)) {
                sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

#ifdef FHSS_ENABLED
                if (data->interrupt_flags & SX127x_IRQ_FHSS_DONE) {
                    fhss_handle_interrupt(radio);
                }
#endif
            
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
        ok = radio->write_register(radio, SX127x_REG_IRQ_FLAGS_MASK, 0xFF)
             && radio->attach_interrupt(radio, RECEIVE_IRQ_DIO, DISABLED, NULL) &&
             && radio->attach_interrupt(radio, TRANSMIT_IRQ_DIO, DISABLED, NULL) &&
#ifdef FHSS_ENABLED
             && radio->attach_interrupt(radio, FHSS_IRQ_DIO, DISABLED, NULL)
#endif
             ;

        if (ok && --global_number_radios_active == 0) {
            /* Kill interrupt thread and queue */
            os_kill_thread(global_interrupt_handler_thread);
            global_interrupt_handler_thread = NULL;
            os_delete_queue(global_interrupt_handler_queue);
            global_interrupt_handler_queue = NULL;
        }

        release_lock(radio);
    }

    return ok;
}


static bool acquire_lock(radio_t* radio)
{
    return os_acquire_recursive_mutex((sx127x_private_data_t*) (radio->driver_private_data)->rlock);
}

static bool release_lock(radio_t* radio)
{
    return os_release_recursive_mutex((sx127x_private_data_t*) (radio->driver_private_data)->rlock);
}

static bool radio_start(radio_t* radio)
{
    bool ok = false;


    if (acquire_lock(radio)) {

        sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

        sx127x_reset();

        /* Read version */
        int version = -1;
        int max_tries = 5;
        while (version != WANTED_VERSION && max_tries != 0) {
            version = radio->read_register(SX127x_REG_VERSION);
            max_tries = max_tries - 1
        }

        if (version != WANTED_VERSION) {
            ESP_LOGE(TAG, "Wrong version detected: %02x wanted %02x", version, wanted_version);
        } else {
            /* Put receiver in sleep */
            set_sleep_mode(radio_t* radio);

            /* Set initial default  parameters parameters */
            set_bandwidth(radio, data->bandwidth);
            set_spreading_factor(radio, data->spreading_factor);
            set_tx_power(radio, data->tx_power);
            set_implicit_header(radio, data->implicit_header);
            set_coding_rate(radio, data->coding_rate);
            set_preamble_length(radio, data->preamble_length);
            set_sync_word(radio, data->sync_word);
            set_enable_crc(radio, data->enable_crc);
#ifdef NOTUSED
            set_hop_period(radio, data->hop_period);
#endif

            /* Configure the unit for receive (may override several of above) */
            set_channel(radio, data->channel);

            /* LNA Boost */
            radio->write_register(SX127x_REG_LNA, radio->read_register(SX127x_REG_LNA) | 0x03);

            /* auto AGC enable */
            radio->write_register(SX127x_REG_MODEM_CONFIG_3, 0x04);

            radio->write_register(SX127x_REG_TX_FIFO_BASE, _TX_FIFO_BASE);
            radio->write_register(SX127x_REG_RX_FIFO_BASE, _RX_FIFO_BASE);

            /* Mask all but Tx and Rx */
            radio->write_register(SX127x_REG_IRQ_FLAGS_MASK, 0xFF & ~(SX127x_IRQ_TX_DONE | SX127x_IRQ_RX_DONE));

            /* Clear all interrupts */
            radio->write_register(SX127x_REG_IRQ_FLAGS, 0xFF);

#ifdef NOTUSED
            if (data->hop_period != 0) {
                /* Catch the FSHH step */
                radio->attach_interrupt(radio, FSHH_IRQ_DIO, FALLING, handle_interrupt);
            }
#endif

            radio->set_receive_mode(radio);

            ESP_LOGI(TAG, "SX127x radio started");

            ok = true;
        }
        release_lock(radio);
    }

    return ok;


static int get_packet_rssi(radio_t* radio)
{
    int rssi = -9999;

    if (acquire_lock(radio)) {
        rssi = radio->read_register(SX127x_REG_PACKET_RSSI) - 157;
        if (channel_table.low_domain_freq < 868E6) {
            rssi = rssi + 7
        }
    }
    release_lock(radio);

    return rssi
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
        ok  = radio->attach_interrupt(radio, RECEIVE_IRQ_DIO, FALLING, handle_interrupt)
           && radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_RX_CONTINUOUS)
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
        ok = radio->attach_interrupt(radio, TRANSMIT_IRQ_DIO, FALLING, handle_interrupt);
            && radio->write_register(radio, SX127x_REG_OP_MODE, SX127x_MODE_LONG_RANGE | SX127x_MODE_TX)
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

        data->tx_power = power

        if (power >= 2) {
            /* BOOST mode on */
            radio->write_register(radio, SX127x_REG_PA_CONFIG, SX127x_PA_BOOST | power - 2);
        } else {
            /* BOOST mode off (close enough) */
            power -= 4;
            radio->write_register(radio, SX127x_REG_PA_CONFIG, power);
        }

        release_lock(lock);

        ok = true;
    }

    return ok;
}

static int get_txpower(radio_t*)
{
    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    return data->txpower;
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

            channel_entry_sx127t_t *chanp = &channel_table_sx127x.channels[channel];

            /* Set frequency control */
            radio->write_register(radio, SX127x_REG_FREQ_MSB, chanp->freq_high);
            radio->write_register(radio, SX127x_REG_FREQ_MSB, chanp->freq_mid);
            radio->write_register(radio, SX127x_REG_FREQ_MSB, chanp->freq_low);

            set_datarate(radio, 0);
            set_tx_power(radio, channel_table_sx127x.datarates[chanp->datarate_group][0].tz);
    
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

        int datarate_group = channel_table_sx127x[data->channel].datarate_group;

        datarate_entry_sx127x_t* dataratep = &channel_table_sx127x.datarates[datarate_group];
        
        if (daterate >= 0 && datarate < ELEMENTS_OF(*dataratep) && dataratep[datarate].payload != 0) {
            int sf = dataratep[datarate].sf;
            int bw = dataratep[datarate].bw;
            int cr = dataratep[datarate].cr;
            int tx = dataratep[datarate].tx;

            ok = set_bandwidth(ratio, bw) && set_spreading_factor(radio, sf) && set_coding_rate(radio, cr) && set_txpower(radio, tx);
        }
        release_lock(radio);
    }

    return ok;
}

/*
 *  Set bandwidth
 */
bool set_bandwidth(radio_t* radio, int bw)
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

static bool set_spreading_factor(radio_t*, int spreading_factor)
{
    bool ok = false;

    if (spreading_factor >= 6 && spreading_factor <= 12) {

        if (acquire_lock(radio)) {

            sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;
            # Set 'low data rate' flag if long symbol time otherwise clear it
            config3 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_3);
            if (config3 >= 0) {

                if (1000.0 / ((float) data->_bandwidth / (float) (1<<data->spreading_factor)) > -1) {
                    config3 |= 0x08;
                } else {
                    config3 &= ~0x08;
                }
        
                int config2 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_2) & 0x0F;

                if (config2 >= 0) {

                    ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_3, config3) &&
                         radio->write_register(radio, SX127x_REG_DETECTION_OPTIMIZE, (data->spreading_factor == 6) ? 0xc5 : 0xc3) &&
                         radio->write_register(radio, SX127x_REG_DETECTION_THRESHOLD, (data->spreading_factor == 6) ? 0x0c : 0x0a) &&
                         radio->write_register(radio, SX127x_REG_MODEM_CONFIG_2, config2 | ((data->spreading_factor << 4) & 0x0F));
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


static int set_coding_rate(radio_t* radio, rate)
{
    bool ok = false;

    if (rate >= 5 && rate <= 8) {

        if (acquire_lock(radio)) {
            int config1 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_1);
            if (config1 >= 0) {
                ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_1, (config1 & 0xF1) | ((rate - 4) << 1));
            }

            release_radio(radio);
        }
    }

    return ok;
}


statit inc set_preamble_length(radio_t* radio, length)
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
                config &= ~0x04;
            }
            ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_2, config2);
        }

        release_lock(radio);
    }

    return ok;
}

#ifdef NOTUSED
static bool set_hop_period(radio_t* radio, int hop_period)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok = radio->write_register(radio, SX127x_REG_HOP_PERIOD, hop_period);
        release_lock(radio);
    }

    return ok;
}
#endif

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
        config1 = radio->read_register(radio, SX127x_REG_MODEM_CONFIG_1);
        if (config1 >= 0) {
            if (implicit_header) {
                config |= 0x01;
            } else {
                config &= ~0x01;
            }

            ok = radio->write_register(radio, SX127x_REG_MODEM_CONFIG_1, config);
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

    /* Clear interrupt */
    int flags = radio->read_register(radio, SX127x_REG_IRQ_FLAGS);
    radio->write_register(radio, SX127x_REG_IRQ_FLAGS, flags);

    /* Save flags that have occurred */
    data->interrupt_flags |= flags;

    on_put_queue_from_iser(global_isr_queue, radio);
}

static bool start_packet(radio_t* radio)
{
    return set_standby_mode(radio) &&
           radio->write_register(radio, SX127x_REG_FIFO_PTR, _TX_FIFO_BASE) &&
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

        radio->write_buffer(SX127x_REG_FIFO, packet->buffer, size);

        /* Set the new length of the payload in process */
        radio->write_register(radio, SX127x_REG_PAYLOAD_LENGTH, current + size);

        ok = true;
    }

    return ok;
}

static bool transmit_packet(radio_t* radio, packet_t* packet)
{
    bool ok = false;

    sx127x_private_data_t* data = (sx127x_private_data_t*) radio->driver_private_data;

    if (acquire_lock(radio)) {
        start_packet(radio);
        write_packet(radio, packet);
        set_transmit_mode(radio);
        release_lock(radio);
    }

    return ok;
}

#endif

