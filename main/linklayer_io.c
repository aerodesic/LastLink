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
#include "esp_event.h"
#include "esp_log.h"

#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "packets.h"
#include "os_freertos.h"
#include "linklayer_io.h"

#define TAG  "linklayer_io"

static bool spi_init(radio_t* radio, const radio_config_t* config);
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


static bool spi_init(radio_t* radio, const radio_config_t* config)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = config->spi_miso,
        .mosi_io_num = config->spi_mosi,
        .sclk_io_num = config->spi_sck,
        .quadwp_io_num = -1,  /* Not used */
        .quadhd_io_num = -1,  /* Not used */
        .max_transfer_sz = MAX_PACKET_LEN,
    };

    ESP_LOGV(TAG, "%s: miso %d mosi %d sclk %d", __func__, buscfg.miso_io_num, buscfg.mosi_io_num, buscfg.sclk_io_num);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock,        /* Clock speed */
        .mode = 0,                                  /* Mode is zero */
        // .mode = 1,                                  /* Mode zero not working - dropping leading bit */
        .spics_io_num = config->spi_cs,             /* Chip select */
        .queue_size = 1,                            /* No queued transfers */
        .command_bits = 1,                          /* Command Read/Write */
        .address_bits = 7,                          /* 7 bit  address */
        .cs_ena_posttrans = 3,                      /* CS Held a few cycles after transfer */
        .pre_cb = config->spi_pre_xfer_callback,    /* Pre-transfer callback if needed */
    };

    ESP_LOGV(TAG, "%s: clk speed %d mode %d cs %d", __func__, devcfg.clock_speed_hz, devcfg.mode, devcfg.spics_io_num);

    
    /* Initialize the SPI device */
    esp_err_t ret = spi_bus_initialize(config->spi_host, &buscfg, config->dma_chan);
    if (ret == ESP_OK) {
        spi_device_handle_t spi;

        ret = spi_bus_add_device(config->spi_host, &devcfg, &spi);
        if (ret == ESP_OK) {
            /* Plug driver entries */
            radio->spi = spi;
            radio->read_register = spi_read_register;
            radio->write_register = spi_write_register;
            radio->read_buffer = spi_read_buffer;
            radio->write_buffer = spi_write_buffer; 
            radio->bus_deinit = spi_deinit;

            ESP_LOGI(TAG, "%s spi is %p", __func__, spi);

        } else {
            ESP_LOGE(TAG, "%s failed to init radio %d: %s", __func__, radio->radio_num, esp_err_to_name(ret));
        }
    }

    return ret == ESP_OK;
}

static bool spi_deinit(radio_t* radio)
{
   esp_err_t err = spi_bus_remove_device(radio->spi);
   if (err != ESP_OK) {
       ESP_LOGE(TAG, "%s failed to deinit radio %d: %s", __func__, radio->radio_num, esp_err_to_name(err));
   }
   return err == ESP_OK;
}

static bool spi_write_register(radio_t* radio, int reg, int data)
{
    spi_transaction_t t;

 ESP_LOGV(TAG, "%s: %02x with %02x", __func__, reg, data);

    memset(&t, 0, sizeof(t));
    t.cmd = 1;                /* Write to register */
    t.addr = reg;
    t.tx_data[0] = data;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;                    /* 1 byte transfer */
    return spi_device_transmit(radio->spi, &t) == ESP_OK;
}

static bool spi_write_buffer(radio_t* radio, int reg, const uint8_t* buffer, int len)
{
    spi_transaction_t t;

 ESP_LOGV(TAG, "%s: %02x with %d bytes", __func__, reg, len);

    memset(&t, 0, sizeof(t));
    t.cmd = 1;                      /* Write */
    t.addr = reg;
    t.length = 8*len;           /* data */
    t.tx_buffer = buffer;

    return spi_device_transmit(radio->spi, &t) == ESP_OK;
}

static int spi_read_register(radio_t* radio, int reg)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.addr = reg;                      /* Read from register */
    t.flags = SPI_TRANS_USE_RXDATA;
    t.length = 8;                    /* 1 byte transfer */

#if 0
    return (spi_device_transmit(radio->spi, &t) == ESP_OK) ? t.rx_data[0] : -1;
#else
    int v = (spi_device_transmit(radio->spi, &t) == ESP_OK) ? t.rx_data[0] : -1;
 ESP_LOGV(TAG, "%s: %02x returned %02x", __func__, reg, v);

    return v;
#endif
}

static bool spi_read_buffer(radio_t* radio, int reg, uint8_t* buffer, int len)
{
    spi_transaction_t t;

 ESP_LOGV(TAG, "%s: %02x for %d bytes", __func__, reg, len);

    memset(&t, 0, sizeof(t));
    t.cmd = reg;
    t.length = 8*len;           /* data */
    t.rx_buffer = buffer;

    return spi_device_transmit(radio->spi, &t) == ESP_OK;
}

