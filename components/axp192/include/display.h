/*
 * ssd1336.h
 *
 * User header for SSD1306/SSD1336 driver.
 */
#ifndef __SSD1306_h_included
#define __SSD1306_h_included

#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "font.h"
#include "bitmap.h"

typedef enum {
    draw_flag_fill   = 0x01,
    draw_flag_border = 0x02,
    draw_flag_clear  = 0x04,
} draw_flags_t;

typedef struct __display__ display_t;

typedef struct __display__ {
    void               *driver_info;
    SemaphoreHandle_t  mutex;

    uint8_t            flags;

    uint8_t*           frame_buf;
    size_t             frame_len;

    /* Overall size */
    int                width;
    int                height;

    int                hold_count;

    /* Currently selected font */
    const font_t       *font;
    int                font_height;

    /* Private - not meant for user calls */
    void               (*_lock)(display_t *display);
    void               (*_unlock)(display_t *display);
    void               (*_show)(display_t *display);

    /* User entry points */
    void               (*close)(display_t *display);
    void               (*clear)(display_t *display);
    void               (*hold)(display_t *display);
    void               (*show)(display_t *display);
    void               (*contrast)(display_t *display, int setting);
    void               (*draw_text)(display_t *display, int x, int y, const char* text);
    void               (*enable)(display_t *display, bool enable);
    void               (*set_font)(display_t *display, const font_t *font);
    const font_t*      (*get_font)(display_t *display);
    void               (*draw_bitmap)(display_t *display, bitmap_t* bitmap, int x, int y, int width, int height, bitmap_method_t method);
#if CONFIG_DISPLAY_RECTANGLE_ENABLED
    void               (*draw_rectangle)(display_t *display, int x, int y, int width, int height, draw_flags_t flags);
#endif
#if CONFIG_DISPLAY_LINE_ENABLED
    void               (*draw_line)(display_t *display, int x1, int y1, int x2, int y2, bool set);
#endif
#if CONFIG_DISPLAY_PIXEL_ENABLED
    void               (*draw_pixel)(display_t *display, int x, int y, bool set);
#endif
#if CONFIG_DISPLAY_PROGRESS_BAR_ENABLED
    void               (*draw_progress_bar)(display_t *display, int x, int y, int width, int height, int total, int progress, const char* text);
#endif
} display_t;

#define DISPLAY_FLAGS_MIRROR_X  0x01
#define DISPLAY_FLAGS_MIRROR_Y  0x02
#define DISPLAY_FLAGS_DEFAULT   0x00

display_t *display_create(int width, int height, uint8_t flags);

#endif /* __SSD1336_h_included */
