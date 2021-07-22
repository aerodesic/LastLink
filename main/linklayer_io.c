/* Use byte-at-a-time for read/write buffer */
//#define USE_SIMPLE_SPI_BUFFER_READ
/*
 * linklayer_io.c
 *
 * Core io level driver for network.
 *
 * Establishes an interface.
 */

#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "packets.h"
#include "os_specific.h"
#include "linklayer_io.h"

#define TAG  "linklayer_io"

static bool spi_init(radio_t* radio, const radio_config_t* config);
static bool spi_deinit(radio_t* radio);
static bool spi_write_register(radio_t* radio, int reg, int data);
static bool spi_write_buffer(radio_t* radio, int reg, const uint8_t* buffer, int len);
static int spi_read_register(radio_t* radio, int reg);
static bool spi_read_buffer(radio_t* radio, int reg, uint8_t* buffer, int len);
static bool spi_transact(radio_t* radio, uint8_t command, int address, uint8_t* outbuf, int outlen, uint8_t* inbuf, int inlen);

#define NUM_PER_LINE 32
void dump_buffer(const char* ident, const uint8_t* buffer, int len)
{
    int addr = 0;

    char outbuf[20+NUM_PER_LINE*4+10];

    while (len != 0) {
        int pos = sprintf(outbuf, "%s: %04x:", ident, addr);

        int todump = NUM_PER_LINE;
        if (todump > len) {
            todump = len;
        }

        for (int b = 0; b < todump; ++b) {
            pos += sprintf(outbuf + pos, " %02x", buffer[addr + b]);
        }

        pos += sprintf(outbuf + pos, "  %*.*s", (NUM_PER_LINE - todump)*3, (NUM_PER_LINE - todump)*3, "");

        for (int b = 0; b < todump; ++b) {
            pos += sprintf(outbuf + pos, "%c", isprint(buffer[addr + b]) ? buffer[addr + b] : '.');
        }

        ESP_LOGI(TAG, "%s: %s", __func__, outbuf);

        addr += todump;
        len -= todump;
    }
}

bool io_init(radio_t* radio, const radio_config_t* config)
{
    bool ret;

    /* Copy fields to be accessable by radio */
    radio->transmit_windows = config->transmit_windows;
    radio->window_width_percent = config->window_width_percent;
    radio->cad_restart_delay = config->cad_restart_delay;
    radio->model = config->model;

    /* Do specific I/O initialization */
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
        .max_transfer_sz = 512,
    };

    ESP_LOGV(TAG, "%s: miso %d mosi %d sclk %d", __func__, buscfg.miso_io_num, buscfg.mosi_io_num, buscfg.sclk_io_num);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock,        /* Clock speed */
        .mode = 0,                                  /* Mode is zero */
        .spics_io_num = config->spi_cs,             /* Chip select */
        .queue_size = 1,                            /* No queued transfers */
        .command_bits = 0,                          /* Command Read/Write */
        .address_bits = 8,                          /* 7 bit  address */
        // .flags = SPI_DEVICE_HALFDUPLEX,
        // .cs_ena_posttrans = 3,                      /* CS Held a few cycles after transfer */
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
            radio->io_transact = spi_transact;
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

 //ESP_LOGV(TAG, "%s: %02x with %02x", __func__, reg, data);

    memset(&t, 0, sizeof(t));
    t.addr = reg | 0x80;
    t.tx_data[0] = data;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;                    /* 1 byte transfer */
    return spi_device_transmit(radio->spi, &t) == ESP_OK;
}

static bool spi_write_buffer(radio_t* radio, int reg, const uint8_t* buffer, int len)
{
    bool ok;

#ifdef USE_SIMPLE_SPI_BUFFER_WRITE
    ok = true;

    for (int b = 0; ok && b < len; ++b) {
        ok = spi_write_register(radio, reg, buffer[b]);
    }
#else
    spi_transaction_t t;

 //ESP_LOGV(TAG, "%s: %02x with %d bytes", __func__, reg, len);

    memset(&t, 0, sizeof(t));
    t.addr = reg | 0x80 ;
    t.length = 8*len;           /* data */
    t.tx_buffer = buffer;

    //dump_buffer("Write", buffer, len);

    ok = spi_device_transmit(radio->spi, &t) == ESP_OK;
#endif

#if 0
if (!ok) {
    spi_host_t *host = radio->spi->host;
    ESP_LOGE(TAG, "%s: max_transfer_sz is %d", __func__, host->bus_attr->max_transfer_sz);
}
#endif
    return ok;
}

static int spi_read_register(radio_t* radio, int reg)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.addr = reg & 0x7F;                      /* Read from register */
    t.flags = SPI_TRANS_USE_RXDATA;
    t.length = 8;                    /* 1 byte transfer */

#if 0
    return (spi_device_transmit(radio->spi, &t) == ESP_OK) ? t.rx_data[0] : -1;
#else
    int v = (spi_device_transmit(radio->spi, &t) == ESP_OK) ? t.rx_data[0] : -1;
 //ESP_LOGV(TAG, "%s: %02x returned %02x", __func__, reg, v);

    return v;
#endif
}

static bool spi_read_buffer(radio_t* radio, int reg, uint8_t* buffer, int len)
{
    bool ok;

#ifdef USE_SIMPLE_SPI_BUFFER_READ
    /* Do it the hard way by looping several read register functions. */
    ok = true;

    for (int b = 0; ok &&  b < len; ++b) {
        int ch = spi_read_register(radio, reg);
        if (ch >= 0) {
            buffer[b] = ch;
        } else {
            ok =  false;
        }
    }
#else
    spi_transaction_t t;

ESP_LOGV(TAG, "%s: %02x for %d bytes into %p", __func__, reg, len, buffer);

    memset(&t, 0, sizeof(t));
    t.addr = reg & 0x7F;
    t.length = 8*len;           /* data */
    t.rx_buffer = buffer;

    ok = spi_device_transmit(radio->spi, &t) == ESP_OK;
#endif

    //if (ok) {
    //    dump_buffer("Read", buffer, len);
    //}

    return ok;
}

static bool spi_transact(radio_t* radio, uint8_t command, int address, uint8_t* outbuffer, int outlen, uint8_t* inbuffer, int inlen)
{
    bool ok;

    spi_transaction_ext_t t;

    memset(&t, 0, sizeof(t));
    t.base.addr = command,
    t.base.length = 8*outlen;
    t.base.rxlength = 8*inlen;
    t.base.tx_buffer = outbuffer;
    t.base.rx_buffer = inbuffer;

    if (address >= 0) {
        /* When supplying an extra address, we make it variable mode and change
         * the addr to a 24 bit number, adding the address field as the second 8 bit
         * field and the remaining 8 bits dummy.
         */
        t.base.flags |= SPI_TRANS_VARIABLE_ADDR;
        t.base.addr = t.base.addr << 16 | address << 8;
        t.address_bits = 8*24;
    }

    ok = spi_device_transmit(radio->spi, (spi_transaction_t*) &t) == ESP_OK;

    return ok;
}

