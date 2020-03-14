/*
 * linklayer_io.h
 *
 * I/O layer initialization.
 */
#ifndef __linklayer_io_h_included
#define __linklayer_io_h_included

#define MAX_DIOS     3

#include "radio.h"
#include "linklayer.h"

#ifdef NOTUSED
typedef enum {
    RADIO_IS_SX126x_DEVICE,
    RADIO_IS_SX127x_DEVICE,
} radio_type_t;

typedef struct radio_config {
    const char* type;
    radio_type_t radio_type;
    int crystal;
    int channel;
    int delay;
    int dios[MAX_DIOS];
    int reset;
    union {
       struct {
         int spi_host;
         int spi_sck;
         int spi_mosi;
         int spi_miso;
         int spi_cs;
         int spi_clock;
         void (*spi_pre_xfer_callback)(spi_transaction_t*);
         int spi_dma;
       };
       struct {
         int i2c_blah;
         int i2c_blah2;
       };
       struct {
         const char* dev;
       };
    };
} radio_config_t;
#endif

bool io_init(radio_t* radio, const radio_config_t* config);
bool io_deinit(radio_t* radio);

#endif /* __linklayer_io_h_included */
