/*
 * ssd1306_i2c.h
 *
 * User access to the ssd1306 i2c driver.
 */
#include "display.h"

#ifndef __ssd1306_i2c_h_included
#define __ssd1306_i2c_h_included

display_t *ssd1306_i2c_create(uint8_t flags);
display_t *ssd1306_i2c_create_raw(int i2c_num, int scl_pin, int sda_pin, int reset_pin, int clk_speed, int width, int height, uint8_t flags);

#endif /* __ssd1306_i2c_h_included */

