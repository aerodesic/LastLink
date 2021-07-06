#ifndef MAIN_SSD1306_H_
#define MAIN_SSD1306_H_

#define SSD1306_NUM_PAGE(h)                ((h) / 8)

// Following definitions are bollowed from 
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

// Control byte
#define SSD1306_CONTROL_BYTE_CMD_SINGLE    0x80
#define SSD1306_CONTROL_BYTE_CMD_STREAM    0x00
#define SSD1306_CONTROL_BYTE_DATA_STREAM   0x40

// Column addressing in page mode
#define SSD1306_CMD_SET_LOWER_COLUMN_ADDR  0x00
#define SSD1306_CMD_SET_UPPER_COLUMN_ADDR  0x10

// Addressing Command Table (pg.30)
#define SSD1306_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define   SSD1306_PARAM_MEMORY_ADDR_MODE_HORIZONTAL 0x00   // Column += 1 after a write; at limit column reset; page += 1
#define   SSD1306_PARAM_MEMORY_ADDR_MODE_VERTICAL   0x01   // Page += 1 after a write; at limit page resete; column += 1
#define   SSD1306_PARAM_MEMORY_ADDR_MODE_PAGE       0x02
#define   SSD1306_PARAM_MEMORY_ADDR_MODE_invalid    0x03

#define SSD1306_CMD_SET_COLUMN_RANGE       0x21    // Starting / ending column address for a region
#define SSD1306_CMD_SET_PAGE_RANGE         0x22    // Starting / ending page address for a region

#define SSD1306_CMD_SET_
// Fundamental commands (pg.28)
#define SSD1306_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define SSD1306_CMD_DISPLAY_RAM            0xA4
#define SSD1306_CMD_DISPLAY_ALLON          0xA5
#define SSD1306_CMD_DISPLAY_NORMAL         0xA6
#define SSD1306_CMD_DISPLAY_INVERTED       0xA7
#define SSD1306_CMD_DISPLAY_OFF            0xAE
#define SSD1306_CMD_DISPLAY_ON             0xAF

// Hardware Config (pg.31)
#define SSD1306_CMD_SET_DISPLAY_START_LINE 0x40
#define SSD1306_CMD_SET_SEGMENT_NORMAL     0xA0    
#define SSD1306_CMD_SET_SEGMENT_REMAP      0xA1    
#define SSD1306_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define SSD1306_CMD_SET_PAGE_START         0xB0    // + page num 0..n-1
#define SSD1306_CMD_SET_COM_SCAN_NORMAL    0xC0    
#define SSD1306_CMD_SET_COM_SCAN_REMAP     0xC8    
#define SSD1306_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define SSD1306_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define SSD1306_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define SSD1306_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define SSD1306_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define SSD1306_CMD_SET_VCOMH_DESELECT     0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define SSD1306_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

#include "ssd1306_i2c.h"

typedef struct {
    int                  i2c_num;
    int                  reset_pin;

    /* Place to save the original display close */
    void                 (*close)(display_t*);
} ssd1306_i2c_driver_info;

#endif /* MAIN_SSD1306_H_ */
