/*
 * linklayer_io.c
 *
 * Core io level driver for network.
 *
 * Establishes an interface.
 */

#include <stdbool.h>
#include <string.h>

#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "packets.h"
#include "os_freertos.h"
#include "linklayer_io.h"

#define TAG  "linklayer_io"

static bool spi_init(radio_t* radio, radio_config_t* config);
static bool spi_deinit(radio_t* radio);
static bool spi_write_register(radio_t* radio, int reg, int data);
static bool spi_write_buffer(radio_t* radio, int reg, const uint8_t* buffer, int len);
static int spi_read_register(radio_t* radio, int reg);
static bool spi_read_buffer(radio_t* radio, int reg, uint8_t* bufer, int len);

bool io_init(radio_t* radio, const radio_config_t* config)
{
    bool ret;

    if (strcmp(config->type, "spi") == 0) {
        ret = spi_init(radio, config);
    } else {
        ret = false;
    }

    return ret;
}

bool io_deinit(radio_t* radio)
{
    return radio->stop(radio);
}


static bool spi_init(radio_t* radio, radio_config_t* config)
{
    spi_device_handle_t spi;

    spi_bus_config_t buscfg = {
        .miso_io_num = config->spi_miso,
        .mosi_io_num = config->spi_mosi,
        .sclk_io_num = config->spi_sck,
        .quadwp_io_num = -1,  /* Not used */
        .quadhd_io_num = -1,  /* Not used */
        .max_transfer_sz = config->spi_clock,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock,        /* Clock speed */
        .mode = 0,                                  /* Mode is zero */
        .spics_io_num = config->spi_cs,             /* Chip select */
        .queue_size = 0,                            /* No queued transfers */
        .command_bits = 8,                          /* 8 bit command */
        .address_bits = 0,                          /* No address field */
        .pre_cb = config->spi_pre_xfer_callback,    /* Pre-transfer callback if needed */
    };

    /* Initialize the SPI device */

    esp_err_t ret = spi_bus_initialize(config->spi_host, &buscfg, config->spi_dma);
    if (ret == ESP_OK) {
        spi_device_handle spi;

        ret = spi_bus_add_device(config->spi_host, &devcfg, &spi);
        if (ret == ESP_OK) {
            /* Plug driver entries */
            radio->spi = spi;
            radio->read_register = spi_read_register;
            radio->write_register = spi_write_register;
            radio->read_buffer = spi_read_buffer;
            radio_>write_buffer = spi_write_buffer; 
            radio->bus_deinit = spi_deinit;
        } else if {
            ESP_LOGE(TAG, "%s failed to deinit radio '%s'", __func__, radio->radio_type, esp_err_to_name(err));
        }
    }

    return ret == ESP_OK;
}

static bool spi_deinit(radio_t* radio)
{
   esp_err_t err = spi_bus_remove_device(radio->spi);
   if (err != ESP_OK) {
       ESP_LOGE(TAG, "%s failed to deinit radio '%s'", __func__, radio->radio_type, esp_err_to_name(err));
   }
   return err == ESP_OK;
}

static bool spi_write_register(radio_t* radio, int reg, int data)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.cmd = reg | 0x80;                /* Write to register */
    t.txdata[0] = data;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8+8;                    /* 2 byte transfer */
    return spi_device_transmit(spi, &t) == ESP_OK;
}

static bool spi_write_buffer(radio_t* radio, int reg, const uint8_t* buffer, int len)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.cmd = reg | 0x80;
    t.length = 8 + 8*len;           /* Register + data */
    t.tx_buffer = buffer;

    return spi_device_transmit(spi, &t) == ESP_OK;
}

static int spi_read_register(radio_t* radio, int reg)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.cmd = reg & 0x7F;                /* Read from register */
    t.flags = SPI_TRANS_USE_RXDATA;
    t.length = 8+8;                    /* 2 byte transfer */

    return (spi_device_transmit(spi, &t) == ESP_OK) ? t.rx_data[0] : -1;
}

static bool spi_read_buffer(radio_t* radio, int reg, uint8_t* bufer, int len)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.cmd = reg & 0x7F;
    t.length = 8 + 8*len;           /* Register + data */
    t.rx_buffer = buffer;

    return spi_device_transmit(spi, &t) == ESP_OK;
}

