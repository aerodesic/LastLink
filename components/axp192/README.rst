Sample code for driving 128x64 OLED display with SSD1306 driver via ESP-IDF's I2C master driver
====================

This code implements a component for adding to the esp-idf environment.  It implements a simple frame buffer manager supporting, font, pixel, line and rectangle operations.  This frame buffer is expanded via a simple inheritance mechanism to include a physical driver supporting frame blit, contrast (brightness) and enable/disable.  I elected to produce my own frame buffer mechanism to avoid the extra overhead of multi-bit and color pixels.  If expansion is anticipated, it would likely prove better to incorporate one of the existing open-source frame buffer managers and replace this driver.

Fixed-width fonts are now supported, with some hooks added to favor implemention variable pitch fonts at a later date.
Simple one-bit-pixel bitmaps are supported to help implement variable height fonts (a font glyph is simply a small bitmap.)



----------
About
----------

This sample code implement procedures to read values from 128x64 OLED display with SSD1306 driver via ESP-IDF's I2C master driver. It supports all features decribed in `Solomon Systech's SSD1306 datasheet`_.

----------
For local setup
----------

For your local setup, connect SDI pin to GPIO 15 pin and the SCK to GPIO 2 pin as they are default ports (I2C_SDA, I2C_SCL) for I2C master according to `ESP32 datasheet`_, C.4. IO_MUX, Page 49.

Be aware about there are serveal models on 128x64 OLED display with SSD1306. Like one model with more pins works with both SPI/I2C, and another model with lesser pins works I2C only.  This cample code is confirmed with a OLED model which have (GND, VDD, SCK, SDA) pins and it supports I2C only (no SPI available). Perhaps your model has slightly different pins, but it should works.

In case it does not work, please check your circit. Consider insert 10k ohm pull-up registors on between 3.3v power supply and (SDA, SCK) pins respectively, as OLED display consumes larger current comparing with other tiny I2C sensors. In my case, 10k ohm pull-up registors stabilized voltage level for clock and SDA and the code worked correctly.

.. _main.c: https://github.com/yanbe/ssd1306-esp-idf-i2c/blob/master/main/main.c
.. _ESP32 datasheet: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
.. _Solomon Systech's SSD1306 datasheet: https://www.robot-r-us.com/e/986-ssd1306-datasheet-for-096-oled.html
