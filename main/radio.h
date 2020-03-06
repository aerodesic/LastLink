#ifndef __radio_h_included
#define __radio_h_included

typedef struct linklayer linklayer_t;

/*
 * Functions called from linklayer to radio
 *
 * There can (eventually) be multiple radios on a linklayer, so we
 * need to pass the particular radio to the radio driver
 */
typedef struct radio {

    void* linklayer_data;
    void* radio_data;

    /* Initialize radio and place in standby mode */
    bool (*start)(struct radio* radio, linklayer_t* linklayer);

    /* Stop and disassemble the radio */
    bool (*stop)(struct radio* radio);

    /* Set sleeping mode (low power) */
    bool (*set_sleep_mode)(struct radio* radio);

    /* Set standby mode */
    bool (*set_standby_mode)(struct radio* radio);

    /* Set active receiving mode */
    bool (*set_receive_mode)(struct radio* radio);

    /* Set transmitter power level */
    bool (*set_txpower)(struct radio* radio, int power);

    /* Get current transmit power */
    int (*get_txpower)(struct radio* radio);

    /* Set channel and datarate */
    bool (*set_channel)(struct radio* radio, int channel, int datarate);

    /* Get channel and datarate */
    bool (*get_channel)(struct radio* radio, int* channel, int* datarate);

} radio_t;

#include "driver/gpio.h"

typedef enum {
    DISABLED = GPIO_PIN_INTR_DISABLE,
    RISING = GPIO_PIN_INTR_POSEDGE,
    FALLING = GPIO_PIN_INTR_NEGEDGE,
    ANY = GPIO_PIN_INTR_ANYEDGE,
} dio_edge_t;

/*
 * Functions called from radio to linklayer
 *
 * We pass the radio structure back through the call so linklayer can disambiguate radios if needed.
 */
typedef struct linklayer {
    /* Read a device register */
    int (*read_register)(radio_t* radio, int register);

    /* Write a device register */
    void (*write_register)(radio_t* radio, int register, int value);

    /* Attach an ISR to a GPIO device */
    bool (*attach_interrupt)(radio_t* radio, int dio, dio_edge_t edge, void (*handler)(void* arg));

    /* Callback when a packet has been received */ 
    packet_t* (*on_receive)(radio_t* radio, packet_t* packet, bool crc_ok, int rssi);

    /* Callback on completion of transmission */
    packet_t* (*on_transmit)(radio_t* radio, packet_t* packet);

    /* Write a block of data to a register */
    void (*write_buffer)(radio_t* radio, int register, uint8_t* buffer, int length);

    /* Read a block of data from a register */
    int (*read_buffer)(radio_t* radio, int register, uint8_t* buffer, int bufsize);

} linklayer_t;

/*
 * Define parameters for spi connection
 */
typedef struct spi_connection {
  const char* type;
  int         sck_pin;
  int         mosi_pin;
  int         miso_pin;
  int         ss_pin;
  int         dio_pins[3];
  int         reset_pin;
} spi_connection_t;

#endif /* __radio_h_include */

