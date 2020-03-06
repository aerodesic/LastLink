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

    bool (*start)(struct radio* radio, linklayer_t* linklayer);
    bool (*stop)(struct radio* radio);
    bool (*set_sleep_mode)(struct radio* radio);
    bool (*set_standby_mode)(struct radio* radio);
    bool (*set_receive_mode)(struct radio* radio);
    bool (*set_txpower)(struct radio* radio, int power);
    int (*get_txpower)(struct radio* radio);
    bool (*set_channel)(struct radio* radio, int channel, int datarate);
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
    int (*read_register)(radio_t* radio, int register);
    void (*write_register)(radio_t* radio, int register, int value);
    bool (*attach_interrupt)(radio_t* radio, int dio, dio_edge_t edge, void (*handler)(void* arg));
    packet_t* (*on_receive)(radio_t* radio, packet_t* packet, bool crc_ok, int rssi);
    packet_t* (*on_transmit)(radio_t* radio);
    void (*write_buffer)(radio_t* radio, int register, uint8_t* buffer, int length);
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

