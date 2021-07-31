/*
 * sx126x_driver.c
 *
 */
#include "sdkconfig.h"
#ifdef CONFIG_LASTLINK_RADIO_SX126x_ENABLED

#define LORA_HEADER_OVERHEAD   22

//#define ENABLE_CAD_MODE

//#define DEBUG_LOCKS
#define DISPLAY_HANDLER_STATE

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

#include "linklayer_io.h"  /* DEBUG */

#define TAG "sx126x_driver"

static bool radio_stop(radio_t* radio);                              /* Stop and disassemble the radio */

static bool set_sleep_mode(radio_t* radio);                          /* Set sleeping mode (low power) */
static bool set_standby_mode(radio_t* radio);                        /* Set standby mode */
static bool set_receive_mode(radio_t* radio);                        /* Request state change to receive mode */
static bool activate_receive_mode(radio_t* radio);                   /* Activates receive mode */
#ifdef ENABLE_CAD_MODE
static bool set_cad_detect_mode(radio_t* radio);                     /* Request cad detect state */
static bool activate_cad_detect_mode(radio_t* radio);                /* Set cad detect mode */
#endif /* ENABLE_CAD_MODE */
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

static bool tx_handle_interrupt(radio_t *radio, bool failed);
static void transmit_start(radio_t *radio);
static void global_interrupt_handler(void* param);
static bool radio_stop(radio_t* radio);
static bool radio_start(radio_t* radio);

#ifdef DEBUG_LOCKS
static bool acquire_lock_debug(radio_t* radio, int line);
static bool release_lock_debug(radio_t* radio, int line);
#define acquire_lock(radio)  acquire_lock_debug(radio, __LINE__)
#define release_lock(radio)  release_lock_debug(radio, __LINE__)
#else
static bool acquire_lock(radio_t* radio);
static bool release_lock(radio_t* radio);
#endif

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
#ifdef ENABLE_CAD_MODE
    HS_WAITING,                   /* Handler is waiting (default state) */
    HS_WAITING_TIMER,             /* Handler is waiting for cad_restart timer */
    HS_CAD_RESTART,               /* CAD timeout so restart */
#endif /* ENABLE_CAD_MODE */
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
#ifdef ENABLE_CAD_MODE
        case HS_WAITING:                return "HS_WAITING";
        case HS_WAITING_TIMER:          return "HS_WAITING_TIMER";
        case HS_CAD_RESTART:            return "HS_CAD_RESTART";
#endif /* ENABLE_CAD_MODE */
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
    int               channel;
    uint8_t           datarate;
    uint8_t           bandwidth;
    int               bandwidth_bits;
    uint8_t           spreading_factor;
    bool              enable_crc;
    int               data_rate_bps;
    uint8_t           tx_power;
    uint8_t           tx_ramp;
    packet_t          *rx_next_packet;
    int               rx_interrupts;
    int               hqi_interrupts;
#ifdef MEASURE_RX_TIME
    uint64_t          rx_start_time;
    uint64_t          rx_end_time;
#endif
#define RX_TIMEOUT_TIME  1000
    int               tx_interrupts;
#define TX_TIMEOUT_TIME  10000
#ifdef USE_FHSS
    int               fhss_interrupts;
    int               hop_period;
#endif /* USE_FHSS */
    os_mutex_t        rlock;
#ifdef DEBUG_LOCKS
    const char*       rlock_thread;
    int               rlock_line;
    int               rlock_count;
#endif
    int               packet_memory_failed;
    packet_t         *current_packet;
    int               current_packet_window;          /* Calculated window number for this packet */
    int               packet_crc_errors;
#ifdef ENABLE_CAD_MODE
    int               cad_interrupts;
    int               cad_last_interrupts;
    int               cad_detected;
    int               cad_timeouts;
    os_timer_t        cad_restart_timer_id;           /* If used, is timer id of cad restart timer */
#define HANDLER_WAKEUP_TIMER_PERIOD 1000              /* Once a second */
    simpletimer_t     handler_wakeup_timer_status;    /* So we can check status of non-readble timer */
    os_timer_t        handler_wakeup_timer_id;        /* ID of radio wakeup timer in case things hang */
#endif /* ENABLE_CAD_MODE */
    int               tx_timeouts;
    int               rx_timeouts;
    int               wakeup_ticks;
    int               handler_cycles;
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
#define INITIAL_WINDOW_TIMER_PERIOD 1000              /* Once a second */
    int               window_event_count;
} sx126x_private_data_t;

static os_thread_t  global_interrupt_handler_thread;
static os_queue_t   global_interrupt_handler_queue;
static void         global_interrupt_handler(void* param);
static int          global_number_radios_active;

#define MAX_IRQ_PENDING    50

#define GPIO_DIO0         0
//#define GPIO_DIO1         1
#define GPIO_DIO2         2

#define GLOBAL_IRQ_THREAD_STACK    8192
#define GLOBAL_IRQ_THREAD_PRIORITY (configMAX_PRIORITIES-1)  /* Highest priority */

typedef struct handler_queue_event {
    radio_t *radio;
    enum {
        HQI_INTERRUPT=1,   /* A hardware interrupt */
#ifdef ENABLE_CAD_MODE
        HQI_RESET,         /* Reset from hung software */
#else
        HQI_SET_STATE,     /* Set to a specific  state */
#endif /* ENABLE_CAD_MODE */
        HQI_WINDOW,        /* A window */
    } type;
    union {
        handler_state_t state;
        int             window;
    };
} handler_queue_event_t;

static bool busy_wait_debug(radio_t* radio, int line)
{
    uint64_t start = get_milliseconds();

    while (radio->test_gpio(radio, radio->busy_dio_num)) {
        os_delay(0);
    }

    int elapsed = get_milliseconds() - start;

    if (elapsed >= 5) {
        ESP_LOGI(TAG, "%s: elapsed %d: line %d", __func__, elapsed, line);
    }

    return true;
}

#define busy_wait(radio) busy_wait_debug(radio, __LINE__)

#ifdef ENABLE_CAD_MODE
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
#else
static bool set_handler_state(radio_t* radio, handler_state_t state)
{
    handler_queue_event_t event = {
        .type = HQI_SET_STATE,
        .radio = radio,
        .state = state,
    };
    return  os_put_queue_with_timeout(global_interrupt_handler_queue, (os_queue_item_t) &event, 0);
}

#endif /* ENABLE_CAD_MODE */

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


#define SX_ALIGN(value, field) ((value & field) / (((unsigned int) -field) & (field)))

/*
 * Place device in sleep mode
 */
static bool dev_SetSleep(radio_t* radio, bool retain, bool rtc)
{
    uint8_t buffer[SX126x_SetSleep_bytes] = {0};

    if (retain) {
        SX_PUT(buffer, SX126x_SetSleep_Param, SX126x_SetSleep_WarmStart | rtc ? SX126x_SetSleep_Rtc : 0);
    }
    
    return radio->io_transact(radio, SX126x_SetSleep, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetSleep, 0, 0, buffer, sizeof(buffer), NULL, 0);
}

/*
 * Place device in standby mode
 */
static bool dev_SetStandby(radio_t* radio, uint8_t mode)
{
    uint8_t buffer[SX126x_SetStandby_bytes] = {0};

    SX_PUT(buffer, SX126x_SetStandby_Param, mode);
    
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetStandby, 0, 0, buffer, sizeof(buffer), NULL, 0);
    bool ok = radio->io_transact(radio, SX126x_SetStandby, 0, 0, buffer, sizeof(buffer), NULL, 0);
    if (ok) {
        os_delay(5);
        busy_wait(radio);
    }
    return ok;
}

/*
 * Place device into Freq Synthesis mode mode.
 */
static bool dev_SetFs(radio_t* radio)
{
    return radio->io_transact(radio, SX126x_SetFs, 0, 0, NULL, 0, NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetFs, 0, 0, NULL, 0, NULL, 0);
}

static bool dev_GetStatus(radio_t* radio, int* chipmode, int* command)
{
    uint8_t buffer[SX126x_GetStatus_bytes] = {0};

    bool ok = radio->io_transact(radio, SX126x_GetStatus, 0, 0, NULL, 0, buffer, sizeof(buffer)) && busy_wait(radio);
    //bool ok = busy_wait(radio) && radio->io_transact(radio, SX126x_GetStatus, 0, 0, NULL, 0, buffer, sizeof(buffer));

    if (ok) {
//dump_buffer("GetStatus", buffer, sizeof(buffer));

        if (chipmode != NULL) {
           *chipmode = SX_ALIGN(buffer[SX126x_GetStatus_Status], SX126x_GetStatus_Status_Chipmode);
        }

        if (command != NULL) {
           *command = SX_ALIGN(buffer[SX126x_GetStatus_Status], SX126x_GetStatus_Status_CommandStatus);
        }
    }
    return ok;
}

static bool dev_GetDeviceErrors(radio_t* radio, int* chipmode, int* command, uint16_t* operrors)
{
    uint8_t buffer[SX126x_GetDeviceErrors_bytes] = {0};

    bool ok = radio->io_transact(radio, SX126x_GetDeviceErrors, 0, 0, NULL, 0, buffer, sizeof(buffer)) && busy_wait(radio);
    //bool ok = busy_wait(radio) && radio->io_transact(radio, SX126x_GetDeviceErrors, 0, 0, NULL, 0, buffer, sizeof(buffer));

    if (ok) {
        if (chipmode != NULL) {
           *chipmode = SX_ALIGN(buffer[SX126x_GetDeviceErrors_Status], SX126x_GetDeviceErrors_Status_Chipmode);
        }

        if (command != NULL) {
           *command = SX_ALIGN(buffer[SX126x_GetDeviceErrors_Status], SX126x_GetDeviceErrors_Status_CommandStatus);
        }

        if (operrors != NULL) {
           *operrors = SX_GET(buffer, SX126x_GetDeviceErrors_OpErrors);
        }
    }

    return ok;
}

static bool dev_ClearDeviceErrors(radio_t* radio)
{
    uint8_t buffer[SX126x_ClearDeviceErrors_bytes] = {0};

    bool ok = radio->io_transact(radio, SX126x_ClearDeviceErrors, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //bool ok = busy_wait(radio) && radio->io_transact(radio, SX126x_ClearDeviceErrors, 0, 0, buffer, sizeof(buffer), NULL, 0);

    return ok;
}

/*
 *
 * show_device_status - debug command
 */
static void show_device_status(radio_t* radio, const char* caller)
{
    int chipmode, command;
    uint16_t operrors;

    if (dev_GetDeviceErrors(radio, &chipmode, &command, &operrors)) {
        ESP_LOGI(TAG, "%s: chipmode 0x%x command 0x%x operrors %03x", caller, chipmode, command, operrors);
    } else {
        ESP_LOGE(TAG, "%s: GetDeviceErrors failed", caller);
    }
}

/*
 * Place device into transmit mode.
 */
static bool dev_SetTx(radio_t* radio, uint32_t timeout)
{
    uint8_t buffer[SX126x_SetTx_bytes] = {0};

    SX_PUT(buffer, SX126x_SetTx_Timeout, timeout);

    //bool ok = radio->io_transact(radio, SX126x_SetTx, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    bool ok = busy_wait(radio)&& radio->io_transact(radio, SX126x_SetTx, 0, 0, buffer, sizeof(buffer), NULL, 0);

    return ok;
}

/*
 * Place device into receive mode.
 */
static bool dev_SetRx(radio_t* radio, uint32_t timeout)
{
    uint8_t buffer[SX126x_SetRx_bytes] = {0};

    SX_PUT(buffer, SX126x_SetRx_Timeout, timeout);

    return radio->io_transact(radio, SX126x_SetRx, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetRx, 0, 0, buffer, sizeof(buffer), NULL, 0);
}

/*
 * Place device into receive mode.
 */
static bool dev_SetRxDutyCycle(radio_t* radio, int rx_period, int sleep_period)
{
    uint8_t buffer[SX126x_SetRxDutyCycle_bytes] = {0};

    SX_PUT(buffer, SX126x_SetRxDutyCycle_rxPeriod, rx_period);
    SX_PUT(buffer, SX126x_SetRxDutyCycle_sleepPeriod, sleep_period);

    return radio->io_transact(radio, SX126x_SetRxDutyCycle, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetRxDutyCycle, 0, 0, buffer, sizeof(buffer), NULL, 0);
}

#ifdef ENABLE_CAD_MODE
/*
 * Place device into CAD mode.
 */
static bool dev_SetCad(radio_t* radio)
{
    return radio->io_transact(radio, SX126x_SetCad, 0, 0, NULL, 0, NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetCad, 0, 0, NULL, 0, NULL, 0);
}

static bool dev_SetCadParams(radio_t* radio, int symbol_num, int detect_peak, int detect_min, int exit_mode, int timeout)
{
    uint8_t buffer[SX126x_SetCadParams_bytes] = {0};

    SX_PUT(buffer, SX126x_SetCadParams_cadSymbolNum, symbol_num);
    SX_PUT(buffer, SX126x_SetCadParams_cadDetPeak, detect_peak);
    SX_PUT(buffer, SX126x_SetCadParams_cadDetMin, detect_min);
    SX_PUT(buffer, SX126x_SetCadParams_cadExitMode, exit_mode);
    SX_PUT(buffer, SX126x_SetCadParams_cadTimeout, timeout);

    return radio->io_transact(radio, SX126x_SetCadParams, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetCadParams, 0, 0, buffer, sizeof(buffer), NULL, 0);
}
#endif

static bool dev_SetBufferBaseAddress(radio_t* radio, int txbase, int rxbase)
{
    uint8_t buffer[SX126x_SetBufferBaseAddress_bytes] = {0};

    SX_PUT(buffer, SX126x_SetBufferBaseAddress_TX, txbase);
    SX_PUT(buffer, SX126x_SetBufferBaseAddress_RX, rxbase);

    return radio->io_transact(radio, SX126x_SetBufferBaseAddress, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetBufferBaseAddress, 0, 0, buffer, sizeof(buffer), NULL, 0);
}

static bool dev_SetLoRaSymbNumTimeout(radio_t* radio, int symbnum)
{
    uint8_t buffer[SX126x_SetLoRaSymbNumTimeout] = {0};

    SX_PUT(buffer, SX126x_SetLoRaSymbNumTimeout_SymbNum, symbnum);

    return radio->io_transact(radio, SX126x_SetLoRaSymbNumTimeout, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetLoRaSymbNumTimeout, 0, 0, buffer, sizeof(buffer), NULL, 0);
}

static bool dev_SetRxTxFallbackMode(radio_t* radio, int mode)
{
    uint8_t buffer[SX126x_SetRxTxFallbackMode_bytes] = {0};

    SX_PUT(buffer, SX126x_SetRxTxFallbackMode_Param, mode);

    return radio->io_transact(radio, SX126x_SetLoRaSymbNumTimeout, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetRxTxFallbackMode, 0, 0, buffer, sizeof(buffer), NULL, 0);
}

static bool dev_GetRxBufferStatus(radio_t* radio, uint8_t* chipmode, uint8_t* command, uint8_t* length, uint8_t* start)
{
    uint8_t buffer[SX126x_GetRxBufferStatus_bytes] = {0};

    bool ok = radio->io_transact(radio, SX126x_GetRxBufferStatus, 0, 0, NULL, 0, buffer, sizeof(buffer)) && busy_wait(radio);
    //bool ok = busy_wait(radio) && radio->io_transact(radio, SX126x_GetRxBufferStatus, 0, 0, NULL, 0, buffer, sizeof(buffer));
    if (ok) {
        if (chipmode != NULL) {
           *chipmode = SX_ALIGN(buffer[SX126x_GetRxBufferStatus_Status], SX126x_GetRxBufferStatus_Status_Chipmode);
        }

        if (command != NULL) {
           *command = SX_ALIGN(buffer[SX126x_GetRxBufferStatus_Status], SX126x_GetRxBufferStatus_Status_CommandStatus);
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

static bool dev_GetIrqStatus(radio_t* radio, uint8_t* chipmode, uint8_t* command, uint16_t* irqflags)
{
    uint8_t buffer[SX126x_GetIrqStatus_bytes] = {0};

    bool ok = radio->io_transact(radio, SX126x_GetIrqStatus, 0, 0, NULL, 0, buffer, sizeof(buffer)) && busy_wait(radio);
    //bool ok = busy_wait(radio) && radio->io_transact(radio, SX126x_GetIrqStatus, 0, 0, NULL, 0, buffer, sizeof(buffer));

    if (ok) {
        if (chipmode != NULL) {
           *chipmode = SX_ALIGN(buffer[SX126x_GetIrqStatus_Status], SX126x_GetIrqStatus_Status_Chipmode);
        }

        if (command != NULL) {
           *command = SX_ALIGN(buffer[SX126x_GetIrqStatus_Status], SX126x_GetIrqStatus_Status_CommandStatus);
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

    return radio->io_transact(radio, SX126x_ClearIrqStatus, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_ClearIrqStatus, 0, 0, buffer, sizeof(buffer), NULL, 0);
}

static bool dev_Calibrate(radio_t* radio, uint8_t modules)
{
    uint8_t buffer[SX126x_Calibrate_bytes] = {0};

    SX_PUT(buffer, SX126x_Calibrate_Param, modules);

    //return radio->io_transact(radio, SX126x_Calibrate, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    bool ok = radio->io_transact(radio, SX126x_Calibrate, 0, 0, buffer, sizeof(buffer), NULL, 0);
    if (ok) {
        os_delay(5);
        busy_wait(radio);
    }
    return ok;
}

/*
 * Perform transaction to set the frequency values.
 */
static bool dev_SetFrequency(radio_t* radio, uint32_t hz)
{
    uint8_t buffer[SX126x_SetRfFrequency_bytes];

    SX_PUT(buffer, SX126x_SetRfFrequency_Hz, hz);

    return radio->io_transact(radio, SX126x_SetRfFrequency, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetRfFrequency, 0, 0, buffer, sizeof(buffer), NULL, 0);
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

//dump_buffer("SetDioIrqParamsmasks", dio_irq_settings, sizeof(dio_irq_settings));

    return radio->io_transact(radio, SX126x_SetDioIrqParams, 0, 0, dio_irq_settings, sizeof(dio_irq_settings), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetDioIrqParams, 0, 0, dio_irq_settings, sizeof(dio_irq_settings), NULL, 0);
}

static bool dev_SetDio3AsTcxoCtrl(radio_t* radio, float voltage, int ms)
{
    uint8_t txcoctrl[SX126x_SetDio3AsTcxoCtrl_bytes] = {0};

    if (abs(voltage - 1.6) <= 0.001) {
        SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_1_6v);
    } else if (abs(voltage - 1.7) <= 0.001) {
        SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_1_7v);
    } else if (abs(voltage - 1.8) <= 0.001) {
        SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_1_8v);
    } else if (abs(voltage - 2.2) <= 0.001) {
        SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_2_2v);
    } else if (abs(voltage - 2.4) <= 0.001) {
        SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_2_4v);
    } else if (abs(voltage - 2.7) <= 0.001) {
        SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_2_7v);
    } else if (abs(voltage - 3.0) <= 0.001) {
        SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_3_0v);
    } else if (abs(voltage - 3.3) <= 0.001) {
        SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage, SX126x_SetDio3AsTcxoCtrl_tcxoVoltage_3_3v);
    } else {
        ESP_LOGE(TAG, "%s: invalid voltage %f", __func__, voltage);
    }

    SX_PUT(txcoctrl, SX126x_SetDio3AsTcxoCtrl_delay, SX_TIMER(ms));

    return radio->io_transact(radio, SX126x_SetDio3AsTcxoCtrl, 0, 0, txcoctrl, sizeof(txcoctrl), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetDio3AsTcxoCtrl, 0, 0, txcoctrl, sizeof(txcoctrl), NULL, 0);
}

static bool dev_SetDio2AsRfSwitch(radio_t* radio, bool enable)
{
    uint8_t rfswitch[SX126x_SetDio2AsRfSwitchCtrl_bytes] = {0};

    SX_PUT(rfswitch, SX126x_SetDio2AsRfSwitchCtrl_enable, enable ? SX126x_SetDio2AsRfSwitchCtrl_enabled : SX126x_SetDio2AsRfSwitchCtrl_disabled);

//dump_buffer("SetDio2AsRfSwitch", rfswitch, sizeof(rfswitch));

    return radio->io_transact(radio, SX126x_SetDio2AsRfSwitchCtrl, 0, 0, rfswitch, sizeof(rfswitch), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetDio2AsRfSwitchCtrl, 0, 0, rfswitch, sizeof(rfswitch), NULL, 0);
}

/*
 * Update tx params
 */
static bool dev_SetTxParams(radio_t* radio, int tx_power, int tx_ramp)
{
    uint8_t tx_params[SX126x_SetTxParams_bytes] = {0};

    SX_PUT(tx_params, SX126x_SetTxParams_TxPower, tx_power);
    SX_PUT(tx_params, SX126x_SetTxParams_TxRamp, tx_ramp);

    return radio->io_transact(radio, SX126x_SetTxParams, 0, 0, tx_params, sizeof(tx_params), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetTxParams, 0, 0, tx_params, sizeof(tx_params), NULL, 0);
}

static bool dev_UpdatePacketParams(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    uint8_t packet_params[SX126x_SetPacketParams_bytes] = {0};

    SX_PUT(packet_params, SX126x_SetPacketParams_PreambleLength, data->preamble_length);
    SX_PUT(packet_params, SX126x_SetPacketParams_HeaderType, data->implicit_header ? SX126x_SetPacketParams_HeaderType_Fixed : SX126x_SetPacketParams_HeaderType_Variable);
    SX_PUT(packet_params, SX126x_SetPacketParams_PayloadLength, CONFIG_LASTLINK_MAX_PACKET_LENGTH);
    SX_PUT(packet_params, SX126x_SetPacketParams_CrcType, data->enable_crc ? SX126x_SetPacketParams_CrcType_On : SX126x_SetPacketParams_CrcType_Off);
    SX_PUT(packet_params, SX126x_SetPacketParams_InvertIQ, SX126x_SetPacketParams_InvertIQ_Off);

    return radio->io_transact(radio, SX126x_SetPacketParams, 0, 0, packet_params, sizeof(packet_params), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetPacketParams, 0, 0, packet_params, sizeof(packet_params), NULL, 0);
}

#ifdef ENABLE_CAD_MODE
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

//ESP_LOGI(TAG, "%s: cadSymbolNum %d cadDetPeak %d cadDetMin %d cadDetExitMode %d cadDetTimeout %d", __func__, cadSymbolNum, cadDetPeak, cadDetMin, cadDetExitMode, cadDetTimeout);

    bool ok = dev_SetCadParams(radio, cadSymbolNum, cadDetPeak, cadDetMin, cadDetExitMode, cadDetTimeout);

//ESP_LOGI(TAG, "%s: dev_SetCalParams returned %s", __func__, ok ? "True" : "False");

    return ok;
}
#endif

/*
 * Peform the transaction to read the most recent packet status.
 */
static bool dev_GetPacketStatus(radio_t* radio, uint8_t* status, int8_t* rssi, int8_t* snr, int8_t* lora_rssi)
{
    uint8_t buffer[SX126x_GetPacketStatus_bytes] = {0};

    bool ok = radio->io_transact(radio, SX126x_GetPacketStatus, 0, 0, NULL, 0, buffer, sizeof(buffer)) && busy_wait(radio);
    //bool ok = busy_wait(radio) && radio->io_transact(radio, SX126x_GetPacketStatus, 0, 0, NULL, 0, buffer, sizeof(buffer));

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

    return radio->io_transact(radio, SX126x_SetPaConfig, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetPaConfig, 0, 0, buffer, sizeof(buffer), NULL, 0);
}

static bool set_dio_irq_mask(radio_t* radio, int irq_mask, int dio0_mask, int dio1_mask, int dio2_mask)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    if (irq_mask >= 0) {
        data->irq_mask = irq_mask;
    }
    if (dio0_mask >= 0) {
        data->dio_mask[0] = dio0_mask;
    } 
    if (dio1_mask >= 0) {
        data->dio_mask[1] = dio1_mask;
    } 
    if (dio2_mask >= 0) {
        data->dio_mask[2] = dio2_mask;
    } 

    bool ok = dev_UpdateDioIrqMasks(radio);
    if (!ok) {
        ESP_LOGE(TAG, "%s: failed to update dio/irq masks", __func__);
    }
    return ok;
}

static bool dev_ReadBuffer(radio_t* radio, int address, uint8_t* buffer, int len)
{
    /* Extra 16 bits of address to include the buffer address and a NOP.  The NOP period
     * is the return of the device Status and we don't want to store that in the output buffer.
     */
    return radio->io_transact(radio, SX126x_ReadBuffer, 16, address<<8, NULL, 0, buffer, len) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_ReadBuffer, 16, address<<8, NULL, 0, buffer, len);
}

static bool dev_WriteBuffer(radio_t* radio, int address, uint8_t* buffer, int len)
{
    bool ok = radio->io_transact(radio, SX126x_WriteBuffer, 8, address, buffer, len, NULL, 0) && busy_wait(radio);
    //bool ok = busy_wait(radio) && radio->io_transact(radio, SX126x_WriteBuffer, 8, address, buffer, len, NULL, 0);

#ifdef NOTUSED
    ESP_LOGI(TAG, "%s: len %d", __func__, len);
    if (ok) {
        dump_buffer("WriteBuffer", buffer, len);
        /* Read buffer back */
        uint8_t readbuffer[len];

        if (dev_ReadBuffer(radio, 0, readbuffer, sizeof(readbuffer))) {
            dump_buffer("Readback", readbuffer, sizeof(readbuffer));
        } else {
            ESP_LOGE(TAG, "%s: readback of buffer failed", __func__);
        }
    }
#endif

    return ok;
}

static bool dev_ReadRegister(radio_t* radio, uint16_t address, uint8_t* buffer, int len)
{
    /* The extra 24 bits of address are the REGISTER identifier + status byte. */
    return radio->io_transact(radio, SX126x_ReadRegister, 24, address<<8, NULL, 0, buffer, len) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_ReadRegister, 24, address<<8, NULL, 0, buffer, len);
}

static bool dev_WriteRegister(radio_t* radio, uint16_t address, uint8_t* buffer, int len)
{
    /* The extra 16 bits of address are the REGISTER identifier */
    return radio->io_transact(radio, SX126x_WriteRegister, 16, address, buffer, len, NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_WriteRegister, 16, address, buffer, len, NULL, 0);
}

static bool dev_SetSyncWord(radio_t* radio, uint16_t syncword_value)
{
    uint8_t syncword[2];

    syncword[0] = syncword_value >> 8;
    syncword[1] = syncword_value & 0xFF;

    return dev_WriteRegister(radio, SX126x_REG_LoRa_Sync_Word_MSB, syncword, sizeof(syncword));
}

static bool dev_SetCurrentLimit(radio_t* radio, uint8_t current_limit)
{
    return dev_WriteRegister(radio, SX126x_REG_OCP_Configuration, &current_limit, sizeof(current_limit));
}

static bool dev_SetPacketType(radio_t* radio, uint8_t packet_type)
{
    uint8_t buffer[SX126x_SetPacketType_bytes] = {0};

    SX_PUT(buffer, SX126x_SetPacketType_PacketType, packet_type);

    return radio->io_transact(radio, SX126x_SetPacketType, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetPacketType, 0, 0, buffer, sizeof(buffer), NULL, 0);
}


static uint8_t convert_bandwidth_code_to_hardware(int bw)
{
    uint8_t bw_hardware;

    switch (bw) {
        default:
            ESP_LOGE(TAG, "%s: undefined bw code: 0x%x", __func__, bw);
            /* Fall through */
        case BW7800:   bw_hardware = SX126x_SetModulationParams_BW7800;    break;
        case BW10400:  bw_hardware = SX126x_SetModulationParams_BW10400;   break;
        case BW15600:  bw_hardware = SX126x_SetModulationParams_BW15600;   break;
        case BW20800:  bw_hardware = SX126x_SetModulationParams_BW20800;   break;
        case BW31250:  bw_hardware = SX126x_SetModulationParams_BW31250;   break;
        case BW41700:  bw_hardware = SX126x_SetModulationParams_BW41700;   break;
        case BW62500:  bw_hardware = SX126x_SetModulationParams_BW62500;   break;
        case BW125000: bw_hardware = SX126x_SetModulationParams_BW125000;  break;
        case BW250000: bw_hardware = SX126x_SetModulationParams_BW250000;  break;
        case BW500000: bw_hardware = SX126x_SetModulationParams_BW500000;  break;
    }

    return bw_hardware;
}

static uint8_t convert_codingrate_code_to_hardware(int cr)
{
    uint8_t cr_hardware;

    switch (cr) {
        default:
            ESP_LOGE(TAG, "%s: undefined codingrate code: 0x%x", __func__, cr);
            /* Fall through */
        case 5: cr_hardware = SX126x_SetModulationParams_CR5; break;
        case 6: cr_hardware = SX126x_SetModulationParams_CR6; break;
        case 7: cr_hardware = SX126x_SetModulationParams_CR7; break;
        case 8: cr_hardware = SX126x_SetModulationParams_CR8; break;
    }

    return cr_hardware;
}

/*This should be called after every change to the SF, BW, CR, or datarate values*/
static bool dev_UpdateModulationParams(radio_t* radio)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

//ESP_LOGI(TAG, "%s: sf %d bw %d bandwidth_bits %d cr %d", __func__, data->spreading_factor, data->bandwidth, data->bandwidth_bits, data->coding_rate);

    uint8_t buffer[SX126x_SetModulationParams_bytes] = {0};

    SX_PUT(buffer, SX126x_SetModulationParams_SF, data->spreading_factor);
    SX_PUT(buffer, SX126x_SetModulationParams_BW, convert_bandwidth_code_to_hardware(data->bandwidth));
    SX_PUT(buffer, SX126x_SetModulationParams_CR, convert_codingrate_code_to_hardware(data->coding_rate));

    uint8_t LdOpt = SX126x_SetModulationParams_LdOpt_OFF;

    // The parameter LdOpt corresponds to the Low Data Rate Optimization (LDRO). This parameter is usually set when the LoRa®
    // symbol time is equal or above 16.38 ms (typically for SF11 with BW125 and SF12 with BW125 and BW250). See
    // Section 6.1.1.4 "Low Data Rate Optimization" on page 39.
    if ((data->spreading_factor == 11 && data->bandwidth == BW125000) || (data->spreading_factor == 12 && data->bandwidth == BW250000)) {
        LdOpt = SX126x_SetModulationParams_LdOpt_ON;
    }

    SX_PUT(buffer, SX126x_SetModulationParams_LdOpt, LdOpt);
    
    return radio->io_transact(radio, SX126x_SetModulationParams, 0, 0, buffer, sizeof(buffer), NULL, 0) && busy_wait(radio);
    //return busy_wait(radio) && radio->io_transact(radio, SX126x_SetModulationParams, 0, 0, buffer, sizeof(buffer), NULL, 0);
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
        /* Add callouts into the driver.  All are called with radio as first parameter */
        radio->stop             = radio_stop;
        radio->set_sleep_mode   = set_sleep_mode;
        radio->set_standby_mode = set_standby_mode;
#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
        radio->print_status     = print_status;
#endif
#ifdef ENABLE_CAD_MODE
        radio->set_receive_mode = set_cad_detect_mode;
#else
        radio->set_receive_mode = set_receive_mode;
#endif /* ENABLE_CAD_MODE */
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
            data->enable_crc                = true;

            data->rlock                     = os_create_recursive_mutex();


            data->handler_state             = HS_STARTUP;

#ifdef DISPLAY_HANDLER_STATE
            data->last_handler_state        = -1;
#endif
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
 ************************************************************************************/
static void rx_handle_interrupt(radio_t *radio)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    data->rx_interrupts++;

    /* Clear rx interrupt */
    data->irq_flags &= ~(SX126x_IrqFlags_RxDone | SX126x_IrqFlags_Timeout);

    uint8_t chipmode;
    uint8_t command;
    uint8_t message_start;
    uint8_t message_length;

    if (dev_GetRxBufferStatus(radio, &chipmode, &command, &message_length, &message_start)) {

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

static bool tx_handle_interrupt(radio_t *radio, bool failed)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    /* Remove flag so we don't call again until next interrupt */
    data->irq_flags &= ~(SX126x_IrqFlags_TxDone | SX126x_IrqFlags_Timeout);

    data->tx_interrupts++;
    if (radio->activity_indicator != NULL) {
        radio->activity_indicator(radio, false);
    }

    bool priority = false;

    /* Discard current queue entry */
    if (data->current_packet != NULL) {

        priority = (get_uint_field(data->current_packet, HEADER_FLAGS, FLAGS_LEN) & HEADER_FLAGS_PRIORITY) != 0;

        data->current_packet->transmitting--;

        if (!failed) {
            /* Count as transmitted */
            data->current_packet->transmitted++;
        }

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

static bool tx_next_packet(radio_t* radio)
{
    sx126x_private_data_t *data = (sx126x_private_data_t*) radio->driver_private_data;

    /* Get a new packet if we have none */
    if (data->current_packet == NULL) {
        if (os_get_queue_with_timeout(radio->transmit_queue, (os_queue_item_t) &data->current_packet, 0)) {

            /* Fetched a new packet, so calculate it's window */
            data->current_packet_window = calculate_window_number(radio, data->current_packet);
ESP_LOGI(TAG, "%s: window %d", __func__, data->current_packet_window);
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

    //bool ok = dev_SetStandby(radio, SX126x_SetStandby_XOSC);
    bool ok = dev_SetStandby(radio, SX126x_SetStandby_RC);

    if (ok) {
        show_device_status(radio, __func__);

        /* Workaround:
         * Datasheet v1.1 workaround: Before any packet transmission, bit #2 at address 0x0889 shall be set to:
         *   • 0 if the LoRa BW = 500 kHz
         *   • 1 for any other LoRa BW
         */

        uint8_t txmod[1];
        if (! dev_ReadRegister(radio, SX126x_REG_TxModulation, txmod, sizeof(txmod))) {
            ESP_LOGE(TAG, "%s: Unable to read TxModulation", __func__);
        } else {
            uint8_t txmodout[1];
            if (data->bandwidth == BW500000) {
                txmodout[0] = txmod[0] & ~0x04;
            } else {
                txmodout[0] = txmod[0] | 0x04;
            }
            if (txmod[0] != txmodout[0]) {
                if (! dev_WriteRegister(radio, SX126x_REG_TxModulation, txmod, sizeof(txmod))) {
                    ESP_LOGE(TAG, "%s: Unable to write TxModulation", __func__);
                }
            }
        }
        /* end workaround */
    } else {
        ESP_LOGE(TAG, "%s: Standby failed", __func__);
    }

    show_device_status(radio, __func__);

    ok = dev_WriteBuffer(radio, 0, data->current_packet->buffer, data->current_packet->length);

#ifdef CONFIG_LASTLINK_CRC16_PACKETS
    if (ok) {
        unsigned short crc = calc_crc16(CRC16_SEED, data->current_packet->buffer, data->current_packet->length);
        unsigned char crcbuf[2] = { crc >> 8, crc & 0xFF };

        ok = WriteBuffer(radio, data->current_packet->length, crcbuf, sizeof(crcbuf));
    }
#endif

    if (ok) {
        show_device_status(radio, __func__);

        ok = set_dio_irq_mask(radio,
                SX126x_IrqFlags_TxDone | SX126x_IrqFlags_Timeout,
                SX126x_IrqFlags_TxDone | SX126x_IrqFlags_Timeout,
                0, 0);

        if (ok) {
            show_device_status(radio, __func__);
            ok = dev_ClearIrqStatus(radio, 0xFFFF);
            if (ok) {
                ok = dev_SetTx(radio, SX_TIMER(TX_TIMEOUT_TIME));
                // ok = dev_SetTx(radio, 0);
            } else {
               ESP_LOGE(TAG, "%s: ClearIrqStatus failed", __func__);
            }
        } else {
            ESP_LOGE(TAG, "%s: set_dio_irq_mask failed", __func__);
        }
    } else {
        ESP_LOGE(TAG, "%s: dev_WriteBuffer failed", __func__);
    }

    show_device_status(radio, __func__);

ESP_LOGI(TAG, "%s: ok %s", __func__, ok ? "True" : "False");

    if (ok && radio->activity_indicator != NULL) {
        radio->activity_indicator(radio, true);
    }

    return ok;
}

static void transmit_start(radio_t* radio)
{
ESP_LOGI(TAG, "%s: called for radio %d", __func__, radio->radio_num);
    // STUB - not used
}

#ifdef ENABLE_CAD_MODE
static void cad_restart(os_timer_t timer_id)
{
ESP_LOGD(TAG, "%s", __func__);
    radio_t *radio = (radio_t*) os_get_timer_data(timer_id);
    set_cad_detect_mode(radio);
}
#endif /* ENABLE_CAD_MODE */

static void global_interrupt_handler(void* param)
{
    bool running = true;

    ESP_LOGD(TAG, "%s: running", __func__);

    while (running) {
        handler_queue_event_t event;

        if (os_get_queue(global_interrupt_handler_queue, (os_queue_item_t) &event)) {

            radio_t *radio = event.radio;

            if (radio != NULL) {
                sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

                switch (event.type) {
#ifdef ENABLE_CAD_MODE
                    case HQI_RESET: {
                        /* Force state transition */
                        data->handler_state = event.state;
                        data->cad_timeouts++;
                        break;
                    } 
#else
                    case HQI_SET_STATE: {
                        data->handler_state = event.state;
                        ESP_LOGE(TAG, "%s: HQI_SET_STATE event", __func__);
                        break;
                    }
#endif /* ENABLE_CAD_MODE */

                    case HQI_INTERRUPT: { 
                        // ESP_LOGE(TAG, "%s: interrupt detected radio %d", __func__, radio->radio_num);
                        data->hqi_interrupts++;
                        break;
                    }

                    case HQI_WINDOW: {
                        // ESP_LOGI(TAG, "%s: window %d", __func__, event.window);
                        break;
                    }
                }

                data->handler_cycles++;

                if (acquire_lock(radio)) {
                    /* Remember what we have */
                    uint8_t chipmode;
                    uint8_t command;
                    uint16_t operrors;
                    uint16_t irq_flags;

                    /* See if any errors were seen */
                    if (dev_GetDeviceErrors(radio, NULL, NULL, &operrors)) {
                         if (operrors != 0) {
                             dev_ClearDeviceErrors(radio);
                             ESP_LOGE(TAG, "%s: device errors: 0x%x", __func__, operrors);
                         }
                    }

                    if (! dev_GetIrqStatus(radio, &chipmode, &command, &irq_flags)) {
                        chipmode = 0;
                        command = 0;
                        irq_flags = 0;
//ESP_LOGI(TAG, "%s: failed GetIrqStatus", __func__);
                    }

                    /* If status says data is present, */
//ESP_LOGI(TAG, "%s: irq chipmode 0x%x command 0x%x irq_flags 0x%04x", __func__, chipmode, command, irq_flags);
     
                    if (irq_flags != 0) {
                        //ESP_LOGI(TAG, "%s: irq_flags 0x%x", __func__, irq_flags);

                        /* Remember new interrupt flags not yet serviced */
                        data->irq_flags |= irq_flags;

                        /* Physically clear the interrupts detected */
                        dev_ClearIrqStatus(radio, irq_flags);

//ESP_LOGI(TAG, "%s: clear irq status %x", __func__, irq_flags);
                    }

#ifdef USE_FHSS
                    if (data->irq_flags & SX126x_IRQ_FHSS_CHANGE_CHANNEL) {
                        fhss_handle_interrupt(radio);
                        data->irq_flags &= ~SX126x_IRQ_FHSS_CHANGE_CHANNEL;
                    }
#endif /* USE_FHSS */

#ifdef DISPLAY_HANDLER_STATE
                    if (data->handler_state != data->last_handler_state) {
                        ESP_LOGE(TAG, "Handler state %s\n", handler_state_of(data->handler_state));
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

#ifdef ENABLE_CAD_MODE
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
#endif /* ENABLE_CAD_MODE */

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
                                    data->rx_next_packet = allocate_packet();
                                }
                                data->handler_state = HS_RECEIVE_DONE;

                            } else if ((data->irq_flags & (SX126x_IrqFlags_Timeout)) != 0) {
                                data->rx_timeouts++;

                                /* Receive attempt timed out.  Return to active mode and clear receive busy */
                                data->irq_flags &= ~(SX126x_IrqFlags_Timeout);
#ifdef ENABLE_CAD_MODE
                                data->handler_state = HS_CAD_RESTART;
#else
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
                                }
//else { printf("%s: window %d want %d\n", __func__, event.window, data->current_packet_window); }
                           }
#endif /* ENABLE_CAD_MODE */

                            break;
                        }

                        /* Actively transmitting - wait for interrupt */
                        case HS_WAIT_TX_INT: {
                            /* Wait for TxDone or Timeout ... */
                            if ((data->irq_flags & (SX126x_IrqFlags_TxDone | SX126x_IrqFlags_Timeout)) != 0) {
                                bool failed = (data->irq_flags & SX126x_IrqFlags_TxDone) == 0;
                                if (failed) {
                                    ESP_LOGE(TAG, "%s: tx failed", __func__);
                                } else {
                                    ESP_LOGI(TAG, "%s: tx done", __func__);
                                }
                                /* Turns off indicator and releases packet */
                                data->handler_state = tx_handle_interrupt(radio, failed) ? HS_TRANSMIT_DONE_PRIORITY : HS_TRANSMIT_DONE;
                            }
                            break;
                        }
                    }

#ifdef DISPLAY_HANDLER_STATE
                    if (data->handler_state != data->last_handler_state) {
                        ESP_LOGE(TAG, "Handler state %s\n", handler_state_of(data->handler_state));
                        data->last_handler_state = data->handler_state;
                    }
#endif

                    switch (data->handler_state) {
                        case HS_STARTUP:
#ifdef ENABLE_CAD_MODE
                        case HS_CAD_RESTART: {
                            activate_cad_detect_mode(radio);
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
                                activate_cad_detect_mode(radio);
                                data->handler_state = HS_WAITING;
                            }
                            break;
                        }

                        case HS_WAITING_TIMER: {
                            // Stall
                            break;
                        }
#endif /* ENABLE_CAD_MODE */
                        case HS_RECEIVING: {
                            activate_receive_mode(radio);
#ifdef MEASURE_RX_TIME
                            data->rx_start_time = get_milliseconds();
#endif /* MEASURE_RX_TIME */
                            break;
                        }

                        case HS_RECEIVE_DONE: {

#ifdef ENABLE_CAD_MODE
                            activate_cad_detect_mode(radio);
                            /* And go back to waiting */
                            data->handler_state = HS_WAITING;
#else
                            data->handler_state = HS_RECEIVING;
#endif /* ENABLE_CAD_MODE */
                            break;
                        }

                        case HS_TRANSMIT_DONE_PRIORITY: {
                            set_standby_mode(radio);
#ifdef ENABLE_CAD_MODE
                            activate_cad_detect_mode(radio);

                            /* And go back to waiting */
                            data->handler_state = HS_WAITING;
#else
                            data->handler_state = HS_RECEIVING;
#endif /* ENABLE_CAD_MODE */
                            break;
                        }

                        case HS_TRANSMIT_DONE: {
                            set_standby_mode(radio);
#ifdef ENABLE_CAD_MODE
                            activate_cad_detect_mode(radio);

                            /* And go back to waiting */
                            data->handler_state = HS_WAITING;
#else
                            data->handler_state = HS_RECEIVING;
#endif /* ENABLE_CAD_MODE */
                            break;
                        }

                        case HS_WAIT_TX_INT: {
                            break;
                        }
                    }

#ifdef DISPLAY_HANDLER_STATE
                    if (data->handler_state != data->last_handler_state) {
                        ESP_LOGI(TAG, "Handler state %s\n", handler_state_of(data->handler_state));
                        data->last_handler_state = data->handler_state;
                    }
#endif

                    if (data->irq_flags != 0) {
                        ESP_LOGI(TAG, "%s: unhandled irq_flags 0x%x", __func__, data->irq_flags);
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
             && radio->attach_interrupt(radio, radio->irq_dio_num, GPIO_PIN_INTR_DISABLE, NULL)
             ;

        /* Kill handler window timer */
        os_delete_timer(data->window_timer_id);
        data->window_timer_id = NULL;

#ifdef ENABLE_CAD_MODE
        /* Kill handler wakeup timer */
        os_delete_timer(data->handler_wakeup_timer_id);
        data->handler_wakeup_timer_id = NULL;
#endif /* ENABLE_CAD_MODE */

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


#ifdef DEBUG_LOCKS
static bool acquire_lock_debug(radio_t* radio, int line)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    bool ok = os_acquire_recursive_mutex_with_timeout(data->rlock, 10000);

    if (ok) {
        data->rlock_thread = os_thread_name(os_current_thread());

        data->rlock_count += 1;

        data->rlock_line = line;

        ESP_LOGI(TAG, "%s: count %d at line %d in %s: Succeeded", __func__, data->rlock_count, line, data->rlock_thread);
    } else {
        ESP_LOGI(TAG, "%s: count %d at line %d in %s: Failed; last locked by %s at line %d", __func__, data->rlock_count, line, os_thread_name(os_current_thread()), data->rlock_thread, data->rlock_line);
    }

    return ok;
}

static bool release_lock_debug(radio_t* radio, int line)
{
    sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

    data->rlock_count -= 1;

    ESP_LOGI(TAG, "%s: count %d at line %d", __func__, data->rlock_count, line);

    return os_release_recursive_mutex(data->rlock);
}
#else
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
#endif


static bool radio_start(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {

        /* Turn on activity led */
        radio->activity_indicator(radio, true);

        sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;

        /* Start global interrupt processing thread if not yet running */
        if (global_interrupt_handler_queue == NULL) {
            global_interrupt_handler_queue = os_create_queue(MAX_IRQ_PENDING, sizeof(handler_queue_event_t));
        }

        if (global_interrupt_handler_thread == NULL) {
            global_interrupt_handler_thread = os_create_thread_on_core(global_interrupt_handler, "sx126x_handler",
                                                                       GLOBAL_IRQ_THREAD_STACK, GLOBAL_IRQ_THREAD_PRIORITY, NULL, 0);
        }

#ifdef ENABLE_CAD_MODE
        /* Create the wakeup timer and leave it stopped */
        data->handler_wakeup_timer_id = os_create_repeating_timer("wakeup_timer", HANDLER_WAKEUP_TIMER_PERIOD, radio, handler_wakeup_timer);
        os_stop_timer(data->handler_wakeup_timer_id);
#endif /* ENABLE_CAD_MODE */

        /* Create window event timer */
        data->window_timer_id = os_create_timer("wakeup_timer", INITIAL_WINDOW_TIMER_PERIOD, radio, window_timer);
        os_stop_timer(data->window_timer_id);  /* Leave it stopped */

        /* Reset physical device */
        radio->reset_device(radio);

        /* Wake it up into standby mode */
        simpletimer_t timer;
        simpletimer_start(&timer, 1000);
        while (!simpletimer_is_expired(&timer) && !set_standby_mode(radio)) {
            busy_wait(radio);
        }
            
        /* place in standby one more time */
        if (!set_standby_mode(radio)) {
            ESP_LOGE(TAG, "%s: set_standby_mode radio %d failed", __func__, radio->radio_num);
        }

        /* Initial configuration */
        /* Set initial default  parameters parameters */
        data->tx_ramp = SX126x_SetTxParams_TxRamp_200uS;

        if (!dev_SetBufferBaseAddress(radio, 0, 0)) {
            ESP_LOGE(TAG, "%s: SetBufferBaseAddress radio %d failed", __func__, radio->radio_num);
        }

        /* Set LORA packets */
        if (!dev_SetPacketType(radio, SX126x_SetPacketType_PacketType_LORA)) {
            ESP_LOGE(TAG, "%s: SetPacketType failed", __func__);
        }

        /* Set fallback to standby RC */
        if (!dev_SetRxTxFallbackMode(radio, SX126x_SetRxTxFallbackMode_Param_STDBY_RC)) {
            ESP_LOGE(TAG, "%s: SetRxTxFallback failed", __func__);
        }

        //if (!dev_SetSyncWord(radio, SX126x_REG_LoRa_Sync_Word_PRIVATE)) {
        if (!dev_SetSyncWord(radio, SX126x_REG_LoRa_Sync_Word_PUBLIC)) {
            ESP_LOGE(TAG, "%s: SetSyncWord failed", __func__);
        }


        if (strcmp(radio->model, "1262") == 0) {
            if (!dev_SetCurrentLimit(radio, SX126x_REG_OCP_Configuration_140mA)) {
                ESP_LOGE(TAG, "%s: SetCurrentLimit failed", __func__);
            }
        }

        /* Workaround: 1262 datasheet v1.1 15.2.2; change tx clamp for PA optimization */
        uint8_t txclamp[1];
        if (!dev_ReadRegister(radio, SX126x_REG_TxClampConfig, txclamp, sizeof(txclamp))) {
            ESP_LOGE(TAG, "%s: Read TxClampConfig failed", __func__);
        } else {
            txclamp[0] |= 0x1E;
            if (!dev_WriteRegister(radio, SX126x_REG_TxClampConfig, txclamp, sizeof(txclamp))) {
                ESP_LOGE(TAG, "%s: Write TxClampConfig failed", __func__);
            }
        }
        /* End workaround */
          
        if (!dev_SetRxDutyCycle(radio, SX_TIMER(RX_RECEIVE_TIME), SX_TIMER(RX_IDLE_TIME))) {
            ESP_LOGE(TAG, "%s: SetRxDutyCycle failed", __func__);
        }

        if (!set_txpower(radio, data->tx_power)) {
            ESP_LOGE(TAG, "%s: set_txpower radio %d failed", __func__, radio->radio_num);
        }

        if (!dev_UpdatePacketParams(radio)) {
            ESP_LOGE(TAG, "%s: UpdatePacketParams radio %d failed", __func__, radio->radio_num);
        }

        /* Mask all IRQs */
        if (!set_dio_irq_mask(radio, 0, 0, 0, 0)) {
            ESP_LOGE(TAG, "%s: set_dio_irq_mask radio %d failed", __func__, radio->radio_num);
        }

        /* Clear all interrupts */
        if (!dev_ClearIrqStatus(radio, 0xFFFF)) {
            ESP_LOGE(TAG, "%s: ClearIrqStatus radio %d failed", __func__, radio->radio_num);
        }

        /* Calibrate all modules */
        if (!dev_Calibrate(radio, SX126x_Calibrate_All)) {
            ESP_LOGE(TAG, "%s: Calibrate All radio %d failed", __func__, radio->radio_num);
        }

#ifdef USE_FHSS
        /* This is leftover SX127x code */
        set_hop_period(radio, data->hop_period);

        if (data->hop_period != 0) {
            /* Enable FHSS interrupt */
            enable_irq(radio, SX126x_IRQ_FHSS_CHANGE_CHANNEL);
        }
#endif

        /* Set TCXO voltage */
        if (!dev_SetDio3AsTcxoCtrl(radio, 1.6, 5)) {
            ESP_LOGE(TAG, "%s: SetTcxoVoltage failed", __func__);
        }

        if (!dev_ClearDeviceErrors(radio)) {
            ESP_LOGE(TAG, "%s: ClearDeviceErrors radio %d failed", __func__, radio->radio_num);
        }

        /* Configure the unit for receive channel 0; probably overriden by caller */
        int old_channel = set_channel(radio, 0);
        if (old_channel < 0) {
            ESP_LOGE(TAG, "%s: set_channel returned %d", __func__, old_channel);
        }

        if (!dev_SetDio2AsRfSwitch(radio, true)) {
            ESP_LOGE(TAG, "%s: SetDio2AsRfSwitch radio %d failed", __func__, radio->radio_num);
        }

        /* Capture receive/transmit interrupts */
        if (!radio->attach_interrupt(radio, radio->irq_dio_num, GPIO_PIN_INTR_POSEDGE, catch_interrupt)) {
            ESP_LOGE(TAG, "%s: attach dio0 interrupt radio %d failed", __func__, radio->radio_num);
        }

#ifdef GPIO_DIO1
        if (!radio->attach_interrupt(radio, GPIO_DIO1, GPIO_PIN_INTR_POSEDGE, catch_interrupt)) {
            ESP_LOGE(TAG, "%s: attach dio1 interrupt radio %d failed", __func__, radio->radio_num);
        }
#endif

        /* Send initial wakeup call to handler */
        handler_queue_event_t event = {
            .radio = radio,
            .type = HQI_INTERRUPT,
        };

        ok = os_put_queue_with_timeout(global_interrupt_handler_queue, (os_queue_item_t) &event, 0);

        /* Turn off activity led */
        radio->activity_indicator(radio, false);

        release_lock(radio);

        ESP_LOGD(TAG, "SX126x radio started");
    }

ESP_LOGI(TAG, "%s: returned %s", __func__, ok ? "True" : "False");

    return ok;
}


static bool set_standby_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok = dev_SetStandby(radio, SX126x_SetStandby_RC);
        release_lock(radio);
    }

    return ok;
}

static bool set_sleep_mode(radio_t* radio)
{
    bool ok = false;

    if (acquire_lock(radio)) {
        ok = dev_SetSleep(radio, /* retain */true, /* rtc */true);
        if (ok) {
            os_delay(2);
        }
        release_lock(radio);
    }

    return ok;
}

/*
 * Set receive mode requesst
 */
static bool set_receive_mode(radio_t* radio)
{
    return set_handler_state(radio, HS_RECEIVING);
}

/*
 * Set receive mode
 */
static bool activate_receive_mode(radio_t* radio)
{
     bool ok = false;

// ESP_LOGI(TAG, "%s: radio %d", __func__, radio->radio_num);

    if (acquire_lock(radio)) {
        if (radio->inactive) {
            /* Leave in sleep mode */
            ok = radio->set_sleep_mode(radio);
        } else {
            ok = dev_SetStandby(radio, SX126x_SetStandby_XOSC) &&
            //ok = dev_SetStandby(radio, SX126x_SetStandby_XOSC) &&
                 set_dio_irq_mask(radio,
                               SX126x_IrqFlags_RxDone | SX126x_IrqFlags_Timeout | SX126x_IrqFlags_HeaderValid,
                               SX126x_IrqFlags_RxDone | SX126x_IrqFlags_Timeout | SX126x_IrqFlags_HeaderValid,
                               0, 0) &&
                 dev_SetRx(radio, SX_TIMER(RX_TIMEOUT_TIME));
        }

        release_lock(radio);
    }

    return ok;
}

#ifdef ENABLE_CAD_MODE
/*
 * Set receive mode requesst
 */
static bool set_cad_detect_mode(radio_t* radio)
{
    return set_handler_state(radio, HS_CAD_RESTART);
}

static bool activate_cad_detect_mode(radio_t* radio)
{
    bool ok = false;

    /* Just put the unit into continuous receive */
    if (acquire_lock(radio)) {
        ok = set_standby_mode(radio)
          && set_dio_irq_mask(radio,
                SX126x_IrqFlags_CadDetected | SX126x_IrqFlags_CadDone,
                SX126x_IrqFlags_CadDetected | SX126x_IrqFlags_CadDone,
                0, 0)
          && dev_SetCad(radio);

if (!ok) ESP_LOGE(TAG, "%s: failed", __func__);

        release_lock(radio);
    }

    return ok;
}
#endif /* ENABLE_CAD_MODE */

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
#ifdef ENABLE_CAD_MODE
                sx126x_private_data_t* data = (sx126x_private_data_t*) radio->driver_private_data;
                /* Restart wakeup timer */
                os_start_timer(data->handler_wakeup_timer_id);
#endif /* ENABLE_CAD_MODE */
            }
        }
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

ESP_LOGI(TAG, "%s: returning %s", __func__, ok ? "True" : "False");

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

        if (acquire_lock(radio)) {
            old_channel = get_channel(radio);

            set_standby_mode(radio);

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
else ESP_LOGI(TAG, "%s: failed to get lock", __func__);
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
                data->data_rate_bps = (data->spreading_factor * 4 * data->bandwidth_bits) / (data->coding_rate * (1 << data->spreading_factor));

ESP_LOGI(TAG, "%s: datarate %d data_rate_bps %d", __func__, data->datarate, data->data_rate_bps);

                /* Set the width of the transmit window in milliseconds */
                data->window_width = (get_message_time(radio, MAX_PACKET_LEN) * radio->window_width_percent) / 100;

ESP_LOGI(TAG, "%s: window_width %d window_timer_id %p", __func__, data->window_width, data->window_timer_id);

                /* Change the window update rate.  Things may be a bit wonky until the first packet is received */
                os_set_timer(data->window_timer_id, data->window_width);

                /* Update radio with the parameters */
                dev_UpdateModulationParams(radio);

#ifdef ENABLE_CAD_MODE
                /* Update CAD operation details */
                dev_UpdateCadParams(radio);
#endif
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

ESP_LOGI(TAG, "%s: bandwidth code %d bits %d", __func__, bwcode, bw);
    data->bandwidth = bwcode;
    data->bandwidth_bits = bw;

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
        ok = true;
    }

    if (ok) {
        command_reply(context, "D", "Radio %d:", radio->radio_num);
        command_reply(context, "D", "Transmit queue length:  %d", os_items_in_queue(radio->transmit_queue));
        command_reply(context, "D", "Waiting output:         %s", data.current_packet ? "YES" : "NO");
        command_reply(context, "D", "sync_word:              %02x", data.sync_word);
        command_reply(context, "D", "preamble_length:        %d", data.preamble_length);
        command_reply(context, "D", "enable_crc:             %s", data.enable_crc ? "YES" : "NO");
        command_reply(context, "D", "coding_rate:            %d", data.coding_rate);
        command_reply(context, "D", "implicit_header:        %s", data.implicit_header ? "YES" : "NO");
#ifdef USE_FHSS
        command_reply(context, "D", "hop_period:             %d", data.hop_period);
        command_reply(context, "D", "fhss_interrupts:        %d", data.fhss_interrupts);
#endif
        command_reply(context, "D", "channel:                %d", data.channel);
        command_reply(context, "D", "datarate:               %d", data.datarate);
        command_reply(context, "D", "bandwidth_bits:         %d", data.bandwidth_bits);
        command_reply(context, "D", "bandwidth:              %d", data.bandwidth);
        command_reply(context, "D", "spreading_factor:       %d", data.spreading_factor);
        command_reply(context, "D", "tx_power:               %d", data.tx_power);
        command_reply(context, "D", "rx_interrupts:          %d", data.rx_interrupts);
        command_reply(context, "D", "rx_interrupts:          %d", data.rx_interrupts);
        command_reply(context, "D", "rx_timeouts:            %d", data.rx_timeouts);
        command_reply(context, "D", "tx_interrupts:          %d", data.tx_interrupts);
        command_reply(context, "D", "tx_timeouts:            %d", data.tx_timeouts);
        command_reply(context, "D", "hqi_interrupts:         %d", data.hqi_interrupts);
#ifdef ENABLE_CAD_MODE
        command_reply(context, "D", "cad_interrupts:         %d", data.cad_interrupts);
        command_reply(context, "D", "cad_detected:           %d", data.cad_detected);
        command_reply(context, "D", "cad_timeouts:           %d", data.cad_timeouts);
#endif /* ENABLE_CAD_MODE */
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

#ifdef ENABLE_CAD_MODE
        int wakeup_timer_remaining = simpletimer_remaining(&data.handler_wakeup_timer_status);

        if (simpletimer_is_running(&data.handler_wakeup_timer_status)) {
            command_reply(context, "D", "wakeup_timer:           %d", wakeup_timer_remaining);
        } else {
            command_reply(context, "D", "wakeup_timer:           %s", simpletimer_is_expired(&data.handler_wakeup_timer_status) ? "expired" : "stopped");
        }
#endif /* ENABLE_CAD_MODE */
    } else {
        command_reply(context, "E", "Unable to lock data");
    }
}
#endif



#endif /* CONFIG_LASTLINK_RADIO_SX126x_ENABLED */

