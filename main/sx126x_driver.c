/*
 * sx126x_driver.c
 *
 */
#include "sdkconfig.h"
#ifdef CONFIG_LASTLINK_RADIO_SX126x_ENABLED

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
#include "sx126x_driver.h"
#include "linklayer.h"
#include "simpletimer.h"

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
#include "commands.h"
#endif

#ifdef CONFIG_LASTLINK_CRC16_PACKETS
#include "crc16.h"
#endif

#define TAG "sx126x_driver"

static bool radio_stop(radio_t* radio);                              /* Stop and disassemble the radio */

static bool set_sleep_mode(radio_t* radio);                          /* Set sleeping mode (low power) */
static bool set_standby_mode(radio_t* radio);                        /* Set standby mode */
static bool set_receive_mode(radio_t* radio);                        /* Set "real" receive mode */
static bool set_cad_detect_mode(radio_t* radio);                     /* Set cad detect mode */
static bool set_inactive(radio_t* radio, bool inactive);             /* Set inactive/active mode */

static bool set_txpower(radio_t* radio, int power);                  /* Set transmitter power level */
static int get_txpower(radio_t* radio);                              /* Get current transmit power */
static int set_channel(radio_t* radio, int channel);                 /* Set channel */
static int get_channel(radio_t* radio);                              /* Get channel */
static int set_datarate(radio_t* radio, int datarate);               /* Set datarate */
static int get_datarate(radio_t* radio);                             /* Get datarate */

static bool set_dio_irq_mask(radio_t* radio, int irq_mask, int dio0_mask, int dio1_mask, int dio2_mask);

typedef struct sx126x_private_data sx126x_private_data_t;  // Forward ref


static void catch_interrupt(void *param);
static void rx_handle_interrupt(radio_t *radio);
#ifdef USE_FHSS
static void fhss_handle_interrupt(radio_t *radio);
#endif /* USE_FHSS */

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
void print_status(command_context_t* context, radio_t *radio);
#endif

static bool tx_handle_interrupt(radio_t *radio);
static void transmit_start(radio_t *radio);
static void global_interrupt_handler(void* param);
static bool radio_stop(radio_t* radio);
static bool release_lock(radio_t* radio);
static bool radio_start(radio_t* radio);
static bool acquire_lock(radio_t* radio);
static bool release_lock(radio_t* radio);

static bool set_bandwidth(radio_t* radio, int bw);
static bool set_spreading_factor(radio_t* radio, int spreading_factor);
static int get_message_time(radio_t* radio, int length);
static int set_coding_rate(radio_t* radio, int rate);
static int set_preamble_length(radio_t* radio, int length);
#ifdef USE_FHSS
static bool set_hop_period(radio_t* radio, int hop_period);
#endif
static bool set_implicit_header(radio_t* radio, bool implicit_header);
#ifdef NOTUSED
static bool set_sync_word(radio_t* radio, uint8_t sync);
#endif

#include "sx126x_table.h"

#define RX_RECEIVE_TIME  5        /* 5 mS receive time */
#define RX_IDLE_TIME     20       /* 20 mS idle time */

#define SX_TIMER_INTERVAL  15.256    /* Microseconds per tick interval */

#define SX_TIMER(ms) ((int) ((((ms * 1000)) / SX_TIMER_INTERVAL) + 0.5))

typedef enum {
    HS_STARTUP,
    HS_WAITING,                   /* Handler is waiting (default state) */
    HS_WAITING_TIMER,             /* Handler is waiting for cad_restart timer */
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
        case HS_WAITING_TIMER:          return "HS_WAITING_TIMER";
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

typedef struct sx126x_private_data {
    radio_t*          radio;
    uint8_t           sync_word;
    uint8_t           preamble_length;
    uint8_t           coding_rate;
    bool              implicit_header;
    bool              implicit_header_set;
    int               hop_period;
    int               channel;
    uint8_t           datarate;
    uint8_t           bandwidth;
    uint8_t           spreading_factor;
    bool              enable_crc;
    int               data_rate_bps;
    uint8_t           tx_power;
    uint8_t           tx_ramp;
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
//#define CAD_TIMEOUT_TIME  100
    int               handler_cycles;
    os_timer_t        handler_wakeup_timer_id;        /* ID of radio wakeup timer in case things hang */
    os_timer_t        cad_restart_timer_id;           /* If used, is timer id of cad restart timer */
#define HANDLER_WAKEUP_TIMER_PERIOD 1000              /* Once a second */
    simpletimer_t     handler_wakeup_timer_status;    /* So we can check status of non-readble timer */
    uint16_t          irq_flags;                      /* Flags acquired at irq scan time */
    uint16_t          irq_mask;                       /* Current IRQ mask selection */
    uint16_t          dio_mask[3];                    /* Current DIO0..2 selections */
    handler_state_t   handler_state;
#ifdef DISPLAY_HANDLER_STATE
    handler_state_t   last_handler_state;
#endif

    int               window_number;
    int               window_width;
    os_timer_t        window_timer_id;
    int               window_event_count;
} sx126x_private_data_t;

static os_thread_t  global_interrupt_handler_thread;
static os_queue_t   global_interrupt_handler_queue;
static void         global_interrupt_handler(void* param);
static int          global_number_radios_active;

#define MAX_IRQ_PENDING    50

#define GPIO_DIO0         0
#define GPIO_DIO1         1

#define GLOBAL_IRQ_THREAD_STACK    8192
#define GLOBAL_IRQ_THREAD_PRIORITY (configMAX_PRIORITIES-1)  /* Highest priority */

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
    radio_t* radio = (radio_t*) os_get_timer_data(timer_id);
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    if (radio->inactive) {
        /* inactive initiated */
        ESP_LOGD(TAG, "%s: inactive detected", __func__);

        /* Stop the periodic wakeup timer */
        os_stop_timer(data->handler_wakeup_timer_id);
    } else {
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
}

inline static int calculate_window_number(radio_t* radio, packet_t *packet)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    int window;

    if (radio->transmit_windows != 1) {
        window = (get_uint_field(data->current_packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN) *
            get_uint_field(data->current_packet, HEADER_DEST_ADDRESS, ADDRESS_LEN) *
            get_uint_field(data->current_packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN) +
            2 * get_uint_field(data->current_packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN)) % radio->transmit_windows;
    } else {
        window = 0;
    }

    return window;
}

/*
 * Called to update the window.
 */
static void window_timer(os_timer_t timer_id)
{
    radio_t* radio = (radio_t*) os_get_timer_data(timer_id);
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

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


/******************************************************
 * These are the physical device transaction handlers.
 ******************************************************/
static inline void sx_put(uint8_t* buffer, int offset, int len, int value) __attribute__((always_inline));

static inline void sx_put(uint8_t* buffer, int offset, int len, int value)
{
    switch (len) {
        case 4:
            buffer[offset++] = value >> 24;
            /* Falls through. */

        case 3:
            buffer[offset++] = value >> 16;
            /* Falls through. */

        case 2:
            buffer[offset++] = value >> 8;
            /* Falls through. */

        case 1:
            buffer[offset]   = value;
            break;
    }
}

#define SX_PUT(buffer, field, value) sx_put(buffer, field, field##_bytes, value)

static inline uint32_t sx_get(uint8_t* buffer, int offset, int len) __attribute__((always_inline));

static inline uint32_t sx_get(uint8_t* buffer, int offset, int len)
{
    unsigned int value = 0;

    switch (len) {
        case 4:
            value |= buffer[offset++] << 24;
            /* Falls through. */

        case 3:
            value |= buffer[offset++] << 16;
            /* Falls through. */

        case 2:
            value |= buffer[offset++] << 8;
            /* Falls through. */

        case 1:
            value |= buffer[offset];
    }
    return value;
}

#define SX_GET(buffer, field) sx_get(buffer, field, field##_bytes)


#define SX_ALIGN(value, field) ((value & field) / (((unsigned int) -field) % (field)))

/*
 * Place device in sleep mode
 */
static bool dev_SetSleep(radio_t* radio, bool retain, bool rtc)
{
    uint8_t buffer[SX126x_SetSleep_bytes] = {0};

    if (retain) {
        SX_PUT(buffer, SX126x_SetSleep_Param, SX126x_SetSleep_WarmStart | rtc ? SX126x_SetSleep_Rtc : 0);
    }
    
    return radio->io_transact(radio, SX126x_SetSleep, -1, buffer, sizeof(buffer), NULL, 0);
}

/*
 * Place device in standby mode
 */
static bool dev_SetStandby(radio_t* radio, bool xosc)
{
    uint8_t buffer[SX126x_SetStandby_bytes] = {0};

    SX_PUT(buffer, SX126x_SetStandby_Param, xosc ? SX126x_SetStandby_XOSC : SX126x_SetStandby_RC);
    
    return radio->io_transact(radio, SX126x_SetStandby, -1, buffer, sizeof(buffer), NULL, 0);
}

/*
 * Place device into Freq Synthesis mode mode.
 */
static bool dev_SetFs(radio_t* radio)
{
    return radio->io_transact(radio, SX126x_SetFs, -1, NULL, 0, NULL, 0);
}

/*
 * Place device into transmit mode.
 */
static bool dev_SetTx(radio_t* radio, uint32_t timeout)
{
    uint8_t buffer[SX126x_SetTx_bytes] = {0};

    SX_PUT(buffer, SX126x_SetTx_Timeout, timeout);

    return radio->io_transact(radio, SX126x_SetTx, -1, buffer, sizeof(buffer), NULL, 0);
}

/*
 * Place device into receive mode.
 */
static bool dev_SetRx(radio_t* radio, uint32_t timeout)
{
    uint8_t buffer[SX126x_SetRx_bytes] = {0};

    SX_PUT(buffer, SX126x_SetRx_Timeout, timeout);

    return radio->io_transact(radio, SX126x_SetRx, -1, buffer, sizeof(buffer), NULL, 0);
}

/*
 * Place device into receive mode.
 */
static bool dev_SetRxDutyCycle(radio_t* radio, int rx_period, int sleep_period)
{
    uint8_t buffer[SX126x_SetRxDutyCycle_bytes] = {0};

    SX_PUT(buffer, SX126x_SetRxDutyCycle_rxPeriod, rx_period);
    SX_PUT(buffer, SX126x_SetRxDutyCycle_sleepPeriod, sleep_period);

    return radio->io_transact(radio, SX126x_SetRxDutyCycle, -1, buffer, sizeof(buffer), NULL, 0);
}

/*
 * Place device into CAD mode.
 */
static bool dev_SetCad(radio_t* radio)
{
    return radio->io_transact(radio, SX126x_SetCad, -1, NULL, 0, NULL, 0);
}

static bool dev_SetCadParams(radio_t* radio, int symbol_num, int detect_peak, int detect_min, int exit_mode, int timeout)
{
    uint8_t buffer[SX126x_SetCadParams_bytes] = {0};

    SX_PUT(buffer, SX126x_SetCadParams_cadSymbolNum, symbol_num);
    SX_PUT(buffer, SX126x_SetCadParams_cadDetPeak, detect_peak);
    SX_PUT(buffer, SX126x_SetCadParams_cadDetMin, detect_min);
    SX_PUT(buffer, SX126x_SetCadParams_cadExitMode, exit_mode);
    SX_PUT(buffer, SX126x_SetCadParams_cadTimeout, timeout);

    return radio->io_transact(radio, SX126x_SetCadParams, -1, buffer, sizeof(buffer), NULL, 0);
}

static bool dev_SetBufferBaseAddress(radio_t* radio, int txbase, int rxbase)
{
    uint8_t buffer[SX126x_SetBufferBaseAddress_bytes] = {0};

    SX_PUT(buffer, SX126x_SetBufferBaseAddress_TX, txbase);
    SX_PUT(buffer, SX126x_SetBufferBaseAddress_RX, rxbase);

    return radio->io_transact(radio, SX126x_SetBufferBaseAddress, -1, buffer, sizeof(buffer), NULL, 0);
}

static bool dev_SetLoRaSymbNumTimeout(radio_t* radio, int symbnum)
{
    uint8_t buffer[SX126x_SetLoRaSymbNumTimeout] = {0};

    SX_PUT(buffer, SX126x_SetLoRaSymbNumTimeout_SymbNum, symbnum);

    return radio->io_transact(radio, SX126x_SetBufferBaseAddress, -1, buffer, sizeof(buffer), NULL, 0);
}

static bool dev_GetStatus(radio_t* radio, int* chipmode, int* status)
{
    uint8_t buffer[SX126x_GetStatus_bytes] = {0};

    bool ok = radio->io_transact(radio, SX126x_GetStatus, -1, NULL, 0, buffer, sizeof(buffer));
    if (ok) {
        if (chipmode != NULL) {
           *chipmode = SX_ALIGN(buffer[0], SX126x_GetStatus_Chipmode);
        }

        if (status != NULL) {
           *status = SX_ALIGN(buffer[0], SX126x_GetStatus_CommandStatus);
        }
    }
    return ok;
}

static bool dev_GetRxBufferStatus(radio_t* radio, uint8_t* status, uint8_t* length, uint8_t* start)
{
    uint8_t buffer[SX126x_GetRxBufferStatus_bytes] = {0};

    bool ok = radio->io_transact(radio, SX126x_GetRxBufferStatus, -1, NULL, 0, buffer, sizeof(buffer));
    if (ok) {
        if (status != NULL) {
           *status = SX_GET(buffer, SX126x_GetRxBufferStatus_Status);
        }

        if (length != NULL) {
           *length = SX_GET(buffer, SX126x_GetRxBufferStatus_PayloadLengthRx);
        }

        if (start != NULL) {
           *start = SX_GET(buffer, SX126x_GetRxBufferStatus_RxStartBufferPointer);
        }
    }

    return ok;
}

static bool dev_GetIrqStatus(radio_t* radio, uint8_t* status, uint16_t* irqflags)
{
    uint8_t buffer[SX126x_GetIrqStatus_bytes] = {0};
    bool ok = radio->io_transact(radio, SX126x_GetIrqStatus, -1, NULL, 0, buffer, sizeof(buffer));

    if (ok) {
        if (status != NULL) {
            *status = SX_GET(buffer, SX126x_GetIrqStatus_Status);
        }
        if (irqflags != NULL) {
            *irqflags = SX_GET(buffer, SX126x_GetIrqStatus_IrqFlags);
        }
    }

    return ok;
}

static bool dev_ClearIrqStatus(radio_t* radio, uint16_t irqflags)
{
    uint8_t buffer[SX126x_ClearIrqStatus_bytes] = {0};

    SX_PUT(buffer, SX126x_ClearIrqStatus_IrqFlags, irqflags);

    return radio->io_transact(radio, SX126x_ClearIrqStatus, -1, buffer, sizeof(buffer), NULL, 0);
}

/*
 * Perform transaction to set the frequency values.
 */
static bool dev_SetFrequency(radio_t* radio, uint32_t hz)
{
    uint8_t buffer[SX126x_SetRfFrequency_bytes];

    SX_PUT(buffer, SX126x_SetRfFrequency_Hz, hz);

    return radio->io_transact(radio, SX126x_SetRfFrequency, -1, buffer, sizeof(buffer), NULL, 0) == ESP_OK;
}

/*
 * Peform the transaction to write out the IRQ and DIO masks.
 */
static bool dev_UpdateDioIrqMasks(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    uint8_t dio_irq_settings[SX126x_SetDioIrqParams_bytes] = {0};

    SX_PUT(dio_irq_settings, SX126x_SetDioIrqParams_IrqMask, data->irq_mask);
    SX_PUT(dio_irq_settings, SX126x_SetDioIrqParams_DIO0, data->dio_mask[0]);
    SX_PUT(dio_irq_settings, SX126x_SetDioIrqParams_DIO1, data->dio_mask[1]);
    SX_PUT(dio_irq_settings, SX126x_SetDioIrqParams_DIO2, data->dio_mask[2]);

    return radio->io_transact(radio, SX126x_SetDioIrqParams, -1, dio_irq_settings, sizeof(dio_irq_settings), NULL, 0) == ESP_OK;
}

/*
 * Update tx params
 */
static bool dev_SetTxParams(radio_t* radio, int tx_power, int tx_ramp)
{
    uint8_t tx_params[SX126x_SetTxParams_bytes] = {0};

    SX_PUT(tx_params, SX126x_SetTxParams_TxPower, tx_power);
    SX_PUT(tx_params, SX126x_SetTxParams_TxRamp, tx_ramp);

    return radio->io_transact(radio, SX126x_SetTxParams, -1, tx_params, sizeof(tx_params), NULL, 0) == ESP_OK;
}

static bool dev_UpdatePacketParams(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    uint8_t packet_params[SX126x_SetPacketParams_bytes] = {0};

    SX_PUT(packet_params, SX126x_SetPacketParams_PreambleLength, data->preamble_length);
    SX_PUT(packet_params, SX126x_SetPacketParams_HeaderType, data->implicit_header ? SX126x_SetPacketParams_HeaderType_Variable : SX126x_SetPacketParams_HeaderType_Fixed);
    SX_PUT(packet_params, SX126x_SetPacketParams_CrcType, data->enable_crc ? SX126x_SetPacketParams_CrcType_On : SX126x_SetPacketParams_CrcType_Off);
    SX_PUT(packet_params, SX126x_SetPacketParams_InvertIQ, SX126x_SetPacketParams_InvertIQ_Off);

    return radio->io_transact(radio, SX126x_SetTxParams, -1, packet_params, sizeof(packet_params), NULL, 0) == ESP_OK;
}

static bool dev_UpdateCadParams(radio_t* radio)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;
    
    int cadSymbolNum = SX126x_SetCadPraams_cadSymbolNum_CAD_ON_1_SYMB;
    int cadDetPeak;
    int cadDetMin;
    int cadDetTimeout = SX_TIMER(10);
    int cadDetExitMode = SX126x_SetCadParams_cadExitMode_CAD_ONLY;

    switch (data->spreading_factor) {
        default:
        case 5:    cadDetPeak = 18;  cadDetMin = 10;  break;
        case 6:    cadDetPeak = 19;  cadDetMin = 10;  break;
        case 7:    cadDetPeak = 20;  cadDetMin = 10;  break;
        case 8:    cadDetPeak = 21;  cadDetMin = 10;  break;
        case 9:    cadDetPeak = 22;  cadDetMin = 10;  break;
        case 10:   cadDetPeak = 23;  cadDetMin = 10;  break;
        case 11:   cadDetPeak = 24;  cadDetMin = 10;  break;
        case 12:   cadDetPeak = 25;  cadDetMin = 10;  break;
    }

    return dev_SetCadParams(radio, cadSymbolNum, cadDetPeak, cadDetMin, cadDetExitMode, cadDetTimeout);
}

/*
 * Peform the transaction to read the most recent packet status.
 */
static bool dev_GetPacketStatus(radio_t* radio, uint8_t* status, int8_t* rssi, int8_t* snr, int8_t* lora_rssi)
{
    uint8_t buffer[SX126x_GetPacketStatus_bytes];

    bool ok = radio->io_transact(radio, SX126x_GetPacketStatus, -1, NULL, 0, buffer, sizeof(buffer)) == ESP_OK;
    if (ok) {
        if (status != NULL) {
            *status = SX_GET(buffer, SX126x_GetPacketStatus_Status);
        }
        if (rssi != NULL) {
            *rssi = SX_GET(buffer, SX126x_GetPacketStatus_RssiPkt);
        }
        if (snr != NULL) {
            *snr = SX_GET(buffer, SX126x_GetPacketStatus_SnrPkt);
        }
        if (lora_rssi != NULL) {
            *lora_rssi = SX_GET(buffer, SX126x_GetPacketStatus_SignalRssiPkt);
        }
    }

    return ok;
}

static bool dev_SetPaConfig(radio_t* radio, int duty_cycle, int hp_max, int device_sel, int pa_lut)
{
    uint8_t buffer[SX126x_SetPaConfig_bytes] = {0};

    SX_PUT(buffer, SX126x_SetPaConfig_paDutyCycle, duty_cycle);
    SX_PUT(buffer, SX126x_SetPaConfig_hpMax, hp_max);
    SX_PUT(buffer, SX126x_SetPaConfig_deviceSel, device_sel);
    SX_PUT(buffer, SX126x_SetPaConfig_paLut, pa_lut);

    return radio->io_transact(radio, SX126x_SetPaConfig, -1, buffer, sizeof(buffer), NULL, 0);
}

static bool set_dio_irq_mask(radio_t* radio, int irq_mask, int dio0_mask, int dio1_mask, int dio2_mask)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    if (irq_mask > 0) {
        data->irq_mask = irq_mask;
    }
    if (dio0_mask > 0) {
        data->dio_mask[0] = dio0_mask;
    } 
    if (dio1_mask > 0) {
        data->dio_mask[1] = dio1_mask;
    } 
    if (dio2_mask > 0) {
        data->dio_mask[2] = dio2_mask;
    } 

    return dev_UpdateDioIrqMasks(radio);
}

static bool dev_ReadBuffer(radio_t* radio, int address, uint8_t* buffer, int len)
{
    return radio->io_transact(radio, SX126x_ReadBuffer, address, NULL, 0, buffer, len);
}

static bool dev_ReadRegister(radio_t* radio, uint16_t address, uint8_t* buffer, int len)
{
    return false;
}

static bool dev_SetPacketType(radio_t* radio, uint8_t packet_type)
{
    uint8_t buffer[SX126x_SetPacketType_bytes] = {0};

    SX_PUT(buffer, SX126x_SetPacketType_PacketType, packet_type);

    return radio->io_transact(radio, SX126x_SetPacketType, -1, buffer, sizeof(buffer), NULL, 0);
}


/*This should be called after every change to the SF, BW, CR, or datarate values*/
static bool dev_UpdateModulationParams(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    uint8_t buffer[SX126x_SetModulationParams_bytes] = {0};

    SX_PUT(buffer, SX126x_SetModulationParams_SF, data->spreading_factor);
    SX_PUT(buffer, SX126x_SetModulationParams_BW, data->bandwidth);
    SX_PUT(buffer, SX126x_SetModulationParams_CR, data->coding_rate);

    uint8_t LdOpt = SX126x_SetModulationParams_LdOpt_OFF;

    // The parameter LdOpt corresponds to the Low Data Rate Optimization (LDRO). This parameter is usually set when the LoRa®
    // symbol time is equal or above 16.38 ms (typically for SF11 with BW125 and SF12 with BW125 and BW250). See
    // Section 6.1.1.4 "Low Data Rate Optimization" on page 39.
    if ((data->spreading_factor == 11 && data->bandwidth == BW125000) || (data->spreading_factor == 12 && data->bandwidth == BW250000)) {
        LdOpt = SX126x_SetModulationParams_LdOpt_ON;
    }

    SX_PUT(buffer, SX126x_SetModulationParams_LdOpt, LdOpt);
    
    return radio->io_transact(radio, SX126x_SetModulationParams, -1, buffer, sizeof(buffer), NULL, 0);
}

/******************************************************
 * End of the physical device transaction handlers.
 ******************************************************/
/*
 * Create an instance of a SX126x device.
 *
 * Entry:
 *    radio		Pointer to value to receive pointer to SX126x control information
 *
 * Returns:
 * 	  true if successful
 *
 */
bool sx126x_create(radio_t* radio)
{
    if (radio != NULL) {
        /* Start global interrupt processing thread if not yet running */
        if (global_interrupt_handler_thread == NULL) {
            global_interrupt_handler_queue = os_create_queue(MAX_IRQ_PENDING, sizeof(handler_queue_event_t));
            global_interrupt_handler_thread = os_create_thread_on_core(global_interrupt_handler, "sx126x_handler",
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
        radio->set_inactive     = set_inactive;

        /* Allocate a data block for local data */
        radio->driver_private_data = malloc(sizeof(sx126x_private_data_t));

        if (radio->driver_private_data != NULL) {
            memset(radio->driver_private_data, 0, sizeof(sx126x_private_data_t));


            sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

            /* Initialize defaults */
            data->radio                     = radio;   /* Point back to owner radio */
            data->sync_word                 = 0x12;    /* 0x34 LoRaWAN; 0x12 LoRa */
            data->preamble_length           = 32;      /* Was 8 */
            data->coding_rate               = 5;
            data->implicit_header           = false;
            data->implicit_header_set       = false;
            data->tx_power                  = 2;

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

	return radio != NULL && radio->driver_private_data != NULL && radio_start(radio->driver_private_data);
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
 ************************************************************************************/
static void rx_handle_interrupt(radio_t *radio)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    data->rx_interrupts++;

    /* Clear rx interrupt */
    data->irq_flags &= ~SX126x_IrqFlags_RxDone;

    uint8_t status;
    uint8_t message_start;
    uint8_t message_length;

    if (dev_GetRxBufferStatus(radio, &status, &message_length, &message_start)) {

         /* Cannot reliably capture packet if CRC error so don't even try */
         if ((data->irq_flags & (SX126x_IrqFlags_CrcErr | SX126x_IrqFlags_Timeout )) == 0) {

            /* Get a packet */
            packet_t* packet = data->rx_next_packet;
            if (packet != NULL) {

                data->rx_next_packet = NULL;

                uint8_t status;

                if (dev_GetPacketStatus(radio, &status, &packet->rssi, &packet->snr, NULL) &&
                    dev_ReadBuffer(radio, message_start, packet->buffer, message_length)) {
#ifdef CONFIG_LASTLINK_CRC16_PACKETS
                    bool crcok = false;
                    if (message_length >= 3) {
                        /* Check the add crc on the packets */
                        crcok = calc_crc16(CRC16_SEED, packet->buffer, message_length) == 0;
                        if (crcok) {
                           message_length = message_length - 2;
                        }
                    }
#else
                    bool crcok = true;
#endif

                    /* Buffer has been read.  Set length */
                    packet->length = message_length;
                    packet->crc_ok = crcok;
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
        }
    } else {
//ESP_LOGE(TAG, "%s: errors %02x", __func__, data->irq_flags);
        if ((data->irq_flags & SX126x_IrqFlags_CrcErr) != 0) {
ESP_LOGE(TAG, "%s: rxint with crc", __func__);
           data->packet_crc_errors++;
        }
        data->irq_flags &= ~(SX126x_IrqFlags_CrcErr | SX126x_IrqFlags_HeaderValid | SX126x_IrqFlags_Timeout);
    }
}

static bool write_packet(radio_t* radio, packet_t* packet)
{
    // FIXME
return false;
}

static bool tx_handle_interrupt(radio_t *radio)
{
    // FIXME
return false;
}

static bool tx_next_packet(radio_t* radio)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    // FIXME
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

static bool tx_recycle_packet(radio_t* radio)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    bool ok = true;
    if (data->current_packet != NULL) {
        ok = os_put_queue_with_timeout(radio->transmit_queue, (os_queue_item_t) &data->current_packet, 0);
        if (ok) {
            data->current_packet = NULL;
        } 
    }
    return ok;
}


static bool tx_start_packet(radio_t* radio)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    bool ok = write_packet(radio, data->current_packet)
           && set_dio_irq_mask(radio, SX126x_IrqFlags_TxDone, SX126x_IrqFlags_TxDone, 0, 0)
           && dev_SetTx(radio, SX_TIMER(TX_TIMEOUT_TIME));

    if (ok && radio->activity_indicator != NULL) {
        radio->activity_indicator(radio, true);
    }

    return ok;
}

static void transmit_start(radio_t* radio)
{
    // STUB - not used
}

static void cad_restart(os_timer_t timer_id)
{
ESP_LOGD(TAG, "%s", __func__);
    radio_t *radio = (radio_t*) os_get_timer_data(timer_id);
    set_cad_detect_mode(radio);
}

static void global_interrupt_handler(void* param)
{
    // FIXME
    bool running = true;

    ESP_LOGD(TAG, "%s: running", __func__);

    while (running) {
        handler_queue_event_t event;

        if (os_get_queue(global_interrupt_handler_queue, (os_queue_item_t) &event)) {

            radio_t *radio = event.radio;

            if (radio != NULL) {
                sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

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
                    uint8_t status;
                    uint16_t irq_flags;

                    if (! dev_GetIrqStatus(radio, &status, &irq_flags)) {
                        status = 0;
                        irq_flags = 0;
                    }

                    /* Remember new interrupt flags not yet serviced */
                    data->irq_flags |= irq_flags;

                    /* Physically clear the interrupts detected */
                    dev_ClearIrqStatus(radio, irq_flags);

#ifdef USE_FHSS
                    if (data->irq_flags & SX126x_IRQ_FHSS_CHANGE_CHANNEL) {
                        fhss_handle_interrupt(data);
                        data->irq_flags &= ~SX126x_IRQ_FHSS_CHANGE_CHANNEL;
                    }
#endif /* USE_FHSS */

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
                            if ((data->irq_flags & (SX126x_IrqFlags_CadDone)) != 0) {
                                data->cad_interrupts++;

                                if ((data->irq_flags & (SX126x_IrqFlags_CadDetected)) != 0) {
                                    data->cad_detected++;
                                    data->handler_state = HS_RECEIVING;
                                }

                                data->irq_flags &= ~(SX126x_IrqFlags_CadDone | SX126x_IrqFlags_CadDetected);
                            }

                            if ((data->handler_state == HS_WAITING) && (event.type == HQI_WINDOW)) {
//printf("%s: window %d\n", __func__, data->current_packet_window);
                                /* If packet exists and it's window id is for the current window, start it going */
                                if (tx_next_packet(radio)) {
                                    if (data->current_packet_window == event.window) {
                                        if (packet_lock(data->current_packet)) {
                                            /* Remains locked through the interrupt return */
                                            tx_start_packet(radio);
                                            data->handler_state = HS_WAIT_TX_INT;
                                        } else {
                                            /* Recycle packet to end of queue in hopes we can do some other work while waiting. */
                                            /* If not, we'll keep moving it back into the queue on each CAD interrupt for a while. */
                                            tx_recycle_packet(radio);
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
                            if (data->irq_flags & SX126x_IrqFlags_RxDone) {
                                /* Packet arrived.  Process it */
#ifdef MEASURE_RX_TIME
                                data->rx_end_time = get_milliseconds();
#endif /* MEASURE_RX_TIME */
                                rx_handle_interrupt(radio);

                                /* Allocate a new packet if we can */
                                if (data->rx_next_packet == NULL) {
                                    data->rx_next_packet = data->rx_next_packet;
                                }
                                data->handler_state = HS_RECEIVE_DONE;

                            } else if (((data->irq_flags & (SX126x_IrqFlags_Timeout)) != 0)) {
                                data->rx_timeouts++;

                                /* Receive attempt timed out.  Return to active mode and clear receive busy */
                                data->irq_flags &= ~(SX126x_IrqFlags_Timeout);
                                data->handler_state = HS_CAD_RESTART;
                            }

                            break;
                        }

                        /* Actively transmitting - wait for interrupt */
                        case HS_WAIT_TX_INT: {
                            if (data->irq_flags & SX126x_IrqFlags_TxDone) {
                                /* Turns off indicator and releases packet */
                                data->handler_state = tx_handle_interrupt(radio) ? HS_TRANSMIT_DONE_PRIORITY : HS_TRANSMIT_DONE;
                            }
                            break;
                        }
                    }

                    switch (data->handler_state) {
                        case HS_STARTUP:
                        case HS_CAD_RESTART: {
                            dev_SetCad(radio);
                            data->handler_state = HS_WAITING;
                            break;
                        }

                        case HS_WAITING: {
                            if (radio->cad_restart_delay != 0) {
                                // Fire a timer in <cad_restart_delay> ms to check start cad waiting again
                                if (data->cad_restart_timer_id == NULL) {
                                    ESP_LOGD(TAG, "%s: creating cad_restart_timer_id", __func__);
                                    data->cad_restart_timer_id = os_create_timer("cad_restart", radio->cad_restart_delay, radio, cad_restart);
                                }
                                os_start_timer(data->cad_restart_timer_id);
                                data->handler_state = HS_WAITING_TIMER;
                            } else {
                                set_cad_detect_mode(radio);
                                data->handler_state = HS_WAITING;
                            }
                            break;
                        }

                        case HS_WAITING_TIMER: {
                            // Stall
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
printf("sx126x_driver handler stopped\n");
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

    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    if (acquire_lock(radio)) {
        /* Disable all interrupts */
        ok = set_dio_irq_mask(radio, 0, 0, 0, 0)
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
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    return os_acquire_recursive_mutex(data->rlock);
}

static bool release_lock(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    return os_release_recursive_mutex(data->rlock);
}


static bool radio_start(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {

        sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

        radio->reset_device(radio);

        /* Set initial default  parameters parameters */
        data->tx_ramp = SX126x_SetTxParams_TxRamp_10uS;

        /* Set LORA packets */
        dev_SetPacketType(radio, SX126x_SetPacketType_PacketType_LORA);

        dev_SetRxDutyCycle(radio, SX_TIMER(RX_RECEIVE_TIME), SX_TIMER(RX_IDLE_TIME));

        /* Put receiver in standby */
        set_sleep_mode(radio);

        set_txpower(radio, data->tx_power);
        dev_UpdatePacketParams(radio);

        /* Mask all IRQs */
        set_dio_irq_mask(radio, 0, 0, 0, 0);

        /* Clear all interrupts */
        dev_ClearIrqStatus(radio, 0xFFFF);

        /* Capture receive/transmit interrupts */
        radio->attach_interrupt(radio, GPIO_DIO0, GPIO_PIN_INTR_POSEDGE, catch_interrupt);
#ifdef GPIO_DIO1
        radio->attach_interrupt(radio, GPIO_DIO1, GPIO_PIN_INTR_POSEDGE, catch_interrupt);
#endif

#ifdef USE_FHSS
        // set_sync_word(radio, data->sync_word);
        set_hop_period(radio, data->hop_period);

        if (data->hop_period != 0) {
            /* Enable FHSS interrupt */
            enable_irq(radio, SX126x_IRQ_FHSS_CHANGE_CHANNEL);
        }
#endif

        /* Configure the unit for receive channel 0; probably overriden by caller */
        set_channel(radio, 0);

        ESP_LOGD(TAG, "SX126x radio started");

        ok = true;
    }

    release_lock(radio);

    return ok;
}


static bool set_standby_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok = dev_SetStandby(radio, SX126x_SetStandby_XOSC);
        release_lock(radio);
    }

    return ok;
}

static bool set_sleep_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        dev_SetSleep(radio, /* retain */true, /* rtc */true);
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

// ESP_LOGI(TAG, "%s: radio %d", __func__, radio->radio_num);

    if (acquire_lock(radio)) {
        if (radio->inactive) {
            /* Leave in sleep mode */
            ok = radio->set_sleep_mode(radio);
        } else {
            ok = set_dio_irq_mask(radio, SX126x_IrqFlags_RxDone | SX126x_IrqFlags_Timeout, SX126x_IrqFlags_TxDone, SX126x_IrqFlags_Timeout, 0)
               && dev_SetRx(radio, SX_TIMER(RX_TIMEOUT_TIME));
        }

        release_lock(radio);
    }

    return ok;
}

static bool set_cad_detect_mode(radio_t* radio)
{
    bool ok = false;

    /* Just put the unit into continuous receive */
    if (acquire_lock(radio)) {
        set_standby_mode(radio);

        ok = set_dio_irq_mask(radio, SX126x_IrqFlags_CadDetected | SX126x_IrqFlags_CadDone, SX126x_IrqFlags_CadDone, SX126x_IrqFlags_CadDetected, 0);

if (!ok) ESP_LOGE(TAG, "%s: failed", __func__);
        release_lock(radio);
    }

    return ok;
}

/* Used to power down radio to standy mode */
static bool set_inactive(radio_t* radio, bool inactive)
{
    bool ok = true;

    ESP_LOGD(TAG, "%s: radio %d %sactive", __func__, radio->radio_num, inactive ? "in" : "");

    if (acquire_lock(radio)) {
        if (inactive != radio->inactive) {
            radio->inactive = inactive;

            /* Do the work to change active mode */
            if (inactive == 0) {
                radio->set_receive_mode(radio);
                sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;
                /* Restart wakeup timer */
                os_start_timer(data->handler_wakeup_timer_id);
            }
            release_lock(radio);
        }
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
        sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;


        /* Set up PA for power level
         * From the manual:
         *  - 17 (0xEF) to +14 (0x0E) dBm by step of 1 dB if low power PA is selected
         *  - 9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
         *  Selection between high power PA and low power PA is done with the command SetPaConfig
         *  and the parameter deviceSel.  By default low power PA and +14 dBm are set.
         */
        
        int duty_cycle = 0;
        int hp_max = 0;
        int pa_lut = 1; /* Fixed for now */
        int device_sel;

        if (strcmp(radio->model, "1262") == 0) {
            device_sel = SX126x_SetPaConfig_deviceSel_SX1262;
            if (power < -17) {
                power = -17;
            } else if (power > 22) {
                power = 22;
            }

            /* The requested power */
            data->tx_power = power;

            switch (power) {
                default:  break;

                case 14:
                case 15:
                case 16:  hp_max = 2; duty_cycle = 2; power = 14; break;

                case 17:
                case 18:
                case 19:  hp_max = 3; duty_cycle = 2; power = 22; break;

                case 20:
                case 21:  hp_max = 5; duty_cycle = 3; power = 22; break;

                case 22:  hp_max = 7; duty_cycle = 4; power = 22; break;
            }
        } else {
           device_sel = SX126x_SetPaConfig_deviceSel_SX1261;

            if (power < -17) {
                power = -17;
            } else if (power > 15) {
                power = 15;
            }

            data->tx_power = power;

            /* 1261 operates only up to 15dBm */
            switch (power) {
                default:  break;
                case 13:  duty_cycle = 1;             break;
                case 14:  duty_cycle = 4;             break;
                case 15:  duty_cycle = 6; power = 14; break;
            }
        }

        dev_SetPaConfig(radio, duty_cycle, hp_max, device_sel, pa_lut);

        dev_SetTxParams(radio, power, data->tx_ramp);

        release_lock(radio);

        ok = true;
    }

    return ok;
}

static int get_txpower(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

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

            sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;
            const channel_entry_sx126x_t *chanp = &channel_table_sx126x.channels[channel];

            /* Set frequency control */
            dev_SetFrequency(radio, chanp->freq);

            radio->set_datarate(radio, 0);

            radio->set_txpower(radio, channel_table_sx126x.datarates[chanp->datarate_group][0].tx);

            data->channel = channel;

            radio->set_receive_mode(radio);

            release_lock(radio);
        }
    }

    return old_channel;
}

static int get_channel(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

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

        sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

        int datarate_group = channel_table_sx126x.channels[data->channel].datarate_group;

        const datarate_entry_sx126x_t* dataratep = channel_table.datarates[datarate_group];

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
                data->data_rate_bps = (data->spreading_factor * 4 * data->bandwidth) / (data->coding_rate * (1 << data->spreading_factor));

                /* Set the width of the transmit window in milliseconds */
                data->window_width = (get_message_time(radio, MAX_PACKET_LEN) * radio->window_width_percent) / 100;

                /* Change the window update rate.  Things may be a bit wonky until the first packet is received */
                os_set_timer(data->window_timer_id, data->window_width);

                /* Update radio with the parameters */
                dev_UpdateModulationParams(radio);

                /* Update CAD operation details */
                dev_UpdateCadParams(radio);
            }
        }

        release_lock(radio);
    }

    return old_datarate;
}

static int get_datarate(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    return data->datarate;
}

     
/*
 *  Set bandwidth
 *
 * Called locked
 */
static bool set_bandwidth(radio_t* radio, int bw)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    bool ok = true;

    int bwcode = -1;
    int bwindex = 0;

    /* Look for bandwidth in code table */
    while (bwcode < 0 && bwindex < ELEMENTS_OF(channel_table_sx126x.bandwidth_bins)) {
        if (bw == channel_table_sx126x.bandwidth_bins[bwindex]) {
            bwcode = bwindex;
        } else {
            bwindex++;
        }
    }

    data->bandwidth = bwcode;

    return ok;
}

/*
 * Called locked
 */
static bool set_spreading_factor(radio_t* radio, int spreading_factor)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    bool ok = false;

ESP_LOGI(TAG, "%s: sf %d", __func__, spreading_factor);

    if (spreading_factor >= 6 && spreading_factor <= 12) {
        ok = true;
        data->spreading_factor = spreading_factor;
    }

    return ok;
}

/*
 * called locked.
 */
static int set_coding_rate(radio_t* radio,  int rate)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;
    bool ok = false;

    if (rate >= 5 && rate <= 8) {
        ok = true;
ESP_LOGI(TAG, "%s: %d", __func__, rate);
        data->coding_rate = rate;
    }

    return ok;
}

static int set_preamble_length(radio_t* radio, int length)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    bool ok = true;
ESP_LOGI(TAG, "%s: length %d", __func__, length);

    if (length != data->preamble_length) {
        data->preamble_length = length;

        ok = dev_UpdatePacketParams(radio);
    }

    return ok;
}

#ifdef USE_FHSS
static bool set_hop_period(radio_t* radio, int hop_period)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    return radio->write_register(radio, SX126x_REG_HOP_PERIOD, hop_period);
}
#endif

#ifdef NOTUSED
static bool set_sync_word(radio_t* radio, uint8_t sync)
{
    bool ok = false;

ESP_LOGI(TAG, "%s: sync %02x", __func__, sync);

    if (acquire_lock(radio)) {
        sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;
        data->sync_word = sync;
        ok = dev_UpdatePacketParams(radio);
        release_lock(radio);
    }

    return ok;
}
#endif  

static bool set_implicit_header(radio_t* radio, bool implicit_header)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

ESP_LOGI(TAG, "%s: enable %s", __func__, implicit_header ? "TRUE" : "FALSE");

    data->implicit_header = implicit_header;
    return dev_UpdatePacketParams(radio);
}

/*
 * Return message time in milliseconds for a packet of given length in bytes (plus preamble overhead.)
 */
static int get_message_time(radio_t* radio, int length)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) (radio->driver_private_data);
    int time = ((length + data->preamble_length + LORA_HEADER_OVERHEAD) * 8 * 1000) / data->data_rate_bps;
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
void print_status(command_context_t* context, radio_t *radio)
{
    sx126x_private_data_t data;

    bool ok = false;

    if (acquire_lock(radio)) {
        memcpy(&data, radio->driver_private_data, sizeof(sx126x_private_data_t));
        release_lock(radio);
    }

    if (ok) {
        command_reply(context, "D", "Radio %d:", radio->radio_num);
        command_reply(context, "D", "Transmit queue length:  %d", os_items_in_queue(radio->transmit_queue));
        command_reply(context, "D", "Waiting output:         %s", data.current_packet ? "YES" : "NO");
        command_reply(context, "D", "sync_word:              %02x", data.sync_word);
        command_reply(context, "D", "preamble_length:        %d", data.preamble_length);
        command_reply(context, "D", "coding_rate:            %d", data.coding_rate);
        command_reply(context, "D", "implicit_header:        %s", data.implicit_header ? "YES" : "NO");
        command_reply(context, "D", "hop_period:             %d", data.hop_period);
        command_reply(context, "D", "channel:                %d", data.channel);
        command_reply(context, "D", "datarate:               %d", data.datarate);
        command_reply(context, "D", "bandwidth:              %d", data.bandwidth);
        command_reply(context, "D", "spreading_factor:       %d", data.spreading_factor);
        command_reply(context, "D", "tx_power:               %d", data.tx_power);
        command_reply(context, "D", "rx_interrupts:          %d", data.rx_interrupts);
        command_reply(context, "D", "rx_timeouts:            %d", data.rx_timeouts);
        command_reply(context, "D", "tx_interrupts:          %d", data.tx_interrupts);
        command_reply(context, "D", "tx_timeouts:            %d", data.tx_timeouts);
        command_reply(context, "D", "cad_interrupts:         %d", data.cad_interrupts);
        command_reply(context, "D", "cad_detected:           %d", data.cad_detected);
        command_reply(context, "D", "cad_timeouts:           %d", data.cad_timeouts);
#ifdef USE_FHSS
        command_reply(context, "D", "fhss_interrupts:        %d", data.fhss_interrupts);
#endif /* USE_FHSS */
        command_reply(context, "D", "packet_memory_failed:   %d", data.packet_memory_failed);
        command_reply(context, "D", "packet_crc_errors:      %d", data.packet_crc_errors);
        command_reply(context, "D", "irq_flags:              %04x", data.irq_flags);
        command_reply(context, "D", "irq_mask:               %04x", data.irq_mask);
        command_reply(context, "D", "dio0_mask:              %04x", data.dio_mask[0]);
        command_reply(context, "D", "dio1_mask:              %04x", data.dio_mask[1]);
        command_reply(context, "D", "dio2_mask:              %04x", data.dio_mask[2]);
        command_reply(context, "D", "handler_state:          %s", handler_state_of(data.handler_state));
        command_reply(context, "D", "handler_cycles:         %d", data.handler_cycles);
        command_reply(context, "D", "wakeup_ticks:           %d", data.wakeup_ticks);
        command_reply(context, "D", "window_number:          %d", data.window_number);
        command_reply(context, "D", "current_packet_window:  %d", data.current_packet_window);
        command_reply(context, "D", "window_width:           %d", data.window_width);
        command_reply(context, "D", "window_event_count:     %d", data.window_event_count);

        int wakeup_timer_remaining = simpletimer_remaining(&data.handler_wakeup_timer_status);

        if (simpletimer_is_running(&data.handler_wakeup_timer_status)) {
            command_reply(context, "D", "wakeup_timer:           %d", wakeup_timer_remaining);
        } else {
            command_reply(context, "D", "wakeup_timer:           %s", simpletimer_is_expired(&data.handler_wakeup_timer_status) ? "expired" : "stopped");
        }
    } else {
        command_reply(context, "E", "Unable to lock data");
    }
}
#endif



#endif /* CONFIG_LASTLINK_RADIO_SX126x_ENABLED */

