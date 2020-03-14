/*
 * sx127x_driver.h
 *
 */
#ifndef __sx127x_driver_h_included
#define __sx127x_driver_h_included

#define SX127x_DEFAULT_PACKET_DELAY	0.05 

/* Register definitions */
#define SX127x_REG_FIFO                 0x00     /* Read/write fifo */
#define SX127x_REG_OP_MODE              0x01     /* Operation mode */
#define SX127x_MODE_LONG_RANGE             0x80
#define SX127x_MODE_MODULATION_FSK         0x00  /* FSK */
#define SX127x_MODE_MODULATION_OOK         0x40  /* OOK */
#define SX127x_MODE_LOW_FREQUENCY          0x08  /* Low frequency mode on */
#define SX127x_MODE_SLEEP                  0x00
#define SX127x_MODE_STANDBY                0x01
#define SX127x_MODE_FS_TX                  0x02
#define SX127x_MODE_TX                     0x03
#define SX127x_MODE_FS_RX                  0x04
#define SX127x_MODE_RX_CONTINUOUS          0x05
#define SX127x_MODE_RX_SINGLE              0x06
/* 0x02 through 0x05 not used */
#define SX127x_REG_FREQ_MSB             0x06     /* Carrier MSB */
#define SX127x_REG_FREQ_MID             0x07     /* Carrier Middle */
#define SX127x_REG_FREQ_LSB             0x08     /* Carrier LSB */
#define SX127x_REG_PA_CONFIG            0x09
#define SX127x_PA_BOOST                    0x80
#define SX127x_REG_PA_RAMP              0x0A     /* Controls ramp time / low phase noise PLL */
#define SX127x_REG_OCP                  0x0B     /* Over-current protection control */
#define SX127x_REG_LNA                  0x0C     /* LNA settings */
#define SX127x_REG_FIFO_PTR             0x0D     /* FIFO SPI Pointer */
#define SX127x_REG_TX_FIFO_BASE         0x0E     /* Start TX data */
#define SX127x_REG_RX_FIFO_BASE         0x0F     /* Start RX data */
#define SX127x_REG_RX_FIFO_CURRENT      0x10     /* Start addr of last packet received */
#define SX127x_REG_IRQ_FLAGS_MASK       0x11     /* Optional IRQ flag mask */
#define SX127x_REG_IRQ_FLAGS            0x12     /* IRQ flags */
#define SX127x_REG_CAD_DETECTED            0x01
#define SX127x_IRQ_FHSS_CHANGE_CHANNEL     0x02
#define SX127x_IRQ_CAD_COMPLETE            0x04
#define SX127x_IRQ_TX_DONE                 0x08
#define SX127x_IRQ_VALID_HEADER            0x10
#define SX127x_IRQ_PAYLOAD_CRC_ERROR       0x20
#define SX127x_IRQ_RX_DONE                 0x40
#define SX127x_IRQ_RX_TIMEOUT              0x80
#define SX127x_REG_RX_NUM_BYTES         0x13     /* Number of received bytes */
#define SX127x_REG_RX_HEADER_CNT_MSB    0x14     /* Number of valid headers MSB */
#define SX127x_REG_RX_HEADER_CNT_LSB    0x15     /* Number of valid headers LSB */
#define SX127x_REG_RX_PACKET_CNT_MSB    0x16     /* Number of packets MSB */
#define SX127x_REG_RX_PACKET_CNT_LSB    0x17     /* Number of packets LSB */
#define SX127x_REG_MODEM_STATUS         0x18     /* Live modem status */
#define SX127x_REG_PACKET_SNR           0x19     /* SNR estimate of last packet */
#define SX127x_REG_PACKET_RSSI          0x1A     /* Last packet RSSI value */
#define SX127x_REG_RSSI_VALUE           0x1B     /* Current SNR value */
#define SX127x_REG_HOP_CHANNEL          0x1C     /* FHSS Start channel */
#define SX127x_REG_MODEM_CONFIG_1       0x1D     /* Modem PHY config 1 */
#define SX127x_REG_MODEM_CONFIG_2       0x1E     /* Modem PHY config 2 */
#define SX127x_REG_SYMBOL_TIMEOUT       0x1F     /* Receiver timeout value */
#define SX127x_REG_PREAMBLE_MSB         0x20     /* Size of preamble MSB */
#define SX127x_REG_PREAMBLE_LSB         0x21     /* Size of preamble LSB */
#define SX127x_REG_PAYLOAD_LENGTH       0x22     /* Payload length */
#define SX127x_REG_MAX_PAYLOAD_LENGTH   0x23     /* Max payload length */
#define SX127x_REG_HOP_PERIOD           0x24     /* FHSS Hop period */
#define SX127x_REG_RX_FIFO_BYTE         0x25     /* Address of last byte written to FIFO */
#define SX127x_REG_MODEM_CONFIG_3       0x26     /* Modem PHY config 3 */
/* 0x27 reserved */
#define SX127x_REG_FEI_MSB              0x28     /* Estimated frequency error MSB */
#define SX127x_REG_FEI_MID              0x29     /* Estimated frequency error MID */
#define SX127x_REG_FEI_LSB              0x2A     /* Estimated frequency error LSB */
/* 0x2B reserved */
#define SX127x_REG_RSSI_WIDEBAND        0x2c     /* Wideband RSSI measurement */
/* 0x2D to 02x30 reserved */
#define SX127x_REG_DETECTION_OPTIMIZE   0x31     /* Detection optimize for SF6 */
/* 0x32 reserved */
#define SX127x_REG_INVERT_IQ            0x33     /* Invert I and Q signals */
/* 0x34 through 0x36 reserved */
#define SX127x_REG_DETECTION_THRESHOLD  0x37     /* Detection threshold for SF6 */
/* 0x38 reserved */
#define SX127x_REG_SYNC_WORD            0x39     /* Sync word */
/* 0x3A through 0x3F reserved */
#define SX127x_REG_DIO_MAPPING_1        0x40     /* Mapping of DIO 0, 1, 2 and 3 */
#define SX127x_REG_DIO_MAPPING_2        0x41     /* Mapping of DIO 4, 5 and ClkOut */
#define SX127x_REG_VERSION              0x42     /* Returns SEMTECH IC Version */
/* 0x43 through 0x4A reserved */
#define SX127x_REG_TCXO                 0x4B     /* TCXO or Crystal input setting */
/* 0x4C reserved */
#define SX127x_REG_PA_DAC               0x4D     /* Higher power settings of PA */
/* 0x4E through 0x60 not used */
#define SX127x_REG_AGC_REG              0x61     /* Adjustments ... */
#define SX127x_REG_AGC_THRESHOLD_1      0x62     /* ... of ... */
#define SX127x_REG_AGC_THRESHOLD_2      0x63     /* ... AGC ... */
#define SX127x_REG_AGC_THRESHOLD_3      0x64     /* ... Thresholds */
/* 0x65 through 0x6F not used */
#define SX127x_REG_PLL                  0x70     /* Constrols PLL bandwidth */
/* 0x71 through 0x7F test mode - not used */

/* Other consts */
#define SX127x_MAX_PACKET_LENGTH        255

#define TX_FIFO_BASE		0x00
#define RX_FIFO_BASE		0x00

#endif /* __sx127x_driver_h_included */
