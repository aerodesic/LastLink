#ifndef __radio_h_included
#define __radio_h_included

#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "os_freertos.h"
#include "packets.h"

#ifdef NOTUSED
typedef enum {
    DISABLED = GPIO_PIN_INTR_DISABLE,
    RISING   = GPIO_PIN_INTR_POSEDGE,
    FALLING  = GPIO_PIN_INTR_NEGEDGE,
    HIGH     = GPIO_PIN_INTR_HILEVEL,
    LOW      = GPIO_PIN_INTR_LOLEVEL,
    ANY      = GPIO_PIN_INTR_ANYEDGE,
} dio_edge_t;
#endif

/*
 * This is created by:
 *
 *  1) Set the io pointer to the io function table (set by io_init)
 *  2) Set the linklayer pointer to the linklayer function table (constant table from linklayer.)
 *  3) Set the driver pointer to the driver function table (set by <driver>_init.)
 *  4) Driver can store private data by setting the driver_private_data pointer, if desired.
 *
 *  This structure is allocated and deleted by the linklayer.
 *
 * A call to <radio>_init(<radio pointer>) sets radio private data.
 * A call to <radio>_deinit(<radio pointer>) must be first made to release any radio private data.
 */

/* Forward declaration */
typedef struct radio radio_t;

typedef struct radio {
    /***********************************/
    /* This data is set by the io_init */
    /***********************************/

    /* Add other device handle / pointers here; filled in by io_init; deleted by io_deinit */
    union {
        spi_device_handle_t  spi;
    };

    /* Radio number */
    int radio_num;

    /* transmit delay after receive or transmit */
    int transmit_delay;

#if 0
    /* Deinit specific radio type (call into the radio module)  */
    bool (*radio_deinit)(radio_t* radio);
#endif

    /* Deinit the bus; calls into linklayer_io */
    bool (*bus_deinit)(radio_t* radio);

    os_queue_t* transmit_queue;

    /*
     * This for any private data owned by the <driver>;
     */
    void* driver_private_data;

    /* Filled in by linklayer; owned by radio; removed by linklayer */
    int (*read_register)(radio_t* radio, int reg);
    bool (*write_register)(radio_t* radio, int reg, int value);
    bool (*write_buffer)(radio_t* radio, int reg, const uint8_t* buffer, int length);
    bool (*read_buffer)(radio_t* radio, int reg, uint8_t* buffer, int bufsize);

    /* linklayer functionality */
    bool (*attach_interrupt)(radio_t* radio, int dio, GPIO_INT_TYPE edge, void (*handler)(void* p));
    void (*on_receive)(radio_t* radio, packet_t* packet);
#if 0
    packet_t* (*on_transmit)(radio_t*, bool first_packet);
#endif
    packet_t* (*get_packet)(radio_t*);
    packet_t* (*discard_packet)(radio_t*);

    void (*reset_device)(radio_t* radio);
    void (*activity_indicator)(radio_t* radio, bool active);


    /************************************************/
    /* End of io init area and start of driver init */
    /************************************************/

    /************************************************/
    /* This area set by the radio driver            */
    /************************************************/

    /* Stop and disassemble the radio */
    bool (*stop)(radio_t* radio);

    /* Set sleeping mode (low power) */
    bool (*set_sleep_mode)(radio_t* radio);

    /* Set standby mode */
    bool (*set_standby_mode)(radio_t* radio);

    /* Set active receiving mode */
    bool (*set_receive_mode)(radio_t* radio);

    /* Set transmitter power level */
    bool (*set_txpower)(radio_t* radio, int power);

    /* Get current transmit power */
    int (*get_txpower)(radio_t* radio);

    /* Set channel */
    bool (*set_channel)(radio_t* radio, int channel);

    /* Get channel */
    int (*get_channel)(radio_t* radio);

    /* Set datarate */
    bool (*set_datarate)(radio_t* radio, int datarate);

    /* Get datarate */
    int (*get_datarate)(radio_t* radio);

    //bool (*transmit_packet)(radio_t* radio, packet_t* packet);
    void (*transmit_start)(radio_t* radio);

} radio_t;

#endif /* __radio_h_include */

