/*
 * Generic part of the display handler.
 */
#include "sdkconfig.h" // generated by "make menuconfig"

#if CONFIG_SSD1306_I2C_ENABLED

#include <string.h>
#include <stdarg.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"


#include "display.h"

#define TAG "display"

static void display_lock(display_t* display)
{
    xSemaphoreTakeRecursive(display->mutex, portMAX_DELAY);
}

static void display_unlock(display_t* display)
{
    xSemaphoreGiveRecursive(display->mutex);
}

static void display_clear(display_t *display)
{
    memset(display->frame_buf, 0, display->frame_len);
}

static void display_hold(display_t *display)
{
    display->hold_count++;
}

static void display_show(display_t *display)
{
    if (display->hold_count > 0) {
        display->hold_count--;
    }
    if (display->hold_count == 0) {
        display->_show(display);
    }
}

/*
 * Put the bitmap into the frame buffer at x, y.  x,y is the top left corner
 * (x, y, width, height) are the dimensions of the region to be overlayed with the bitmap.
 * Extra space is ignore.  Insufficient space causes truncation.
 */
static void display_draw_bitmap(display_t *display, bitmap_t *bitmap, int x, int y, int width, int height, bitmap_method_t method)
{
    //static uint8_t masks[] = { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, };
    static uint8_t masks[] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, };

    int bitmap_row = 0;
    int target_row = y;

    int end_x = x + width - 1;
    int end_y = y + height - 1;

    bool odd = true;

    display->_lock(display);

    display->hold(display);

    int bitmap_height = bitmap->height;

    while (bitmap_height > 0) {
        /* Compute byte shift and mask */
        int shift = (bitmap_row + target_row) % 8;  /* Starting row at this y interval */

//ESP_LOGI(TAG, "%s: bitmap_height %d height %d bitmap_row %d target_row %d odd %s shift %d", __func__, bitmap_height, height, bitmap_row, target_row, odd ? "ODD left" : "EVEN right", shift);

        /* Do one row */
        for (int bx = 0; bx < bitmap->width && (bx + x) < end_x; ++bx) {
            uint8_t value = bitmap->bits != NULL ? bitmap->bits[bitmap->width * (bitmap_row/8) + bx] : 0xFF;
            uint8_t mask = ((end_y - target_row + 1) < 8) ? mask = masks[end_y - target_row + 1] : 0xFF;

            if (odd) {
                value = (value & mask) << shift;
            } else {
                value = (value >> shift) & mask;
            }

            uint8_t *byte = &display->frame_buf[(target_row / 8) * display->width + x + bx];

            if (method == bitmap_method_XOR) {
                *byte ^= value;
            } else if (method == bitmap_method_NAND) {
                *byte &= ~value;
            } else {
                *byte |= value;
            }
        }

        odd = !odd;

        /* Move to the  next bitmap row */
        bitmap_row += (8 - shift);
        target_row += (8 - shift);

        /* Remove from the height required */
        bitmap_height -= (8 - shift);
    }

    display->show(display);

    display->_unlock(display);
}

#if CONFIG_DISPLAY_PIXEL_ENABLED
static void display_draw_pixel(display_t *display, int x, int y, bool set)
{
    display->_lock(display);

    display->hold(display);

//ESP_LOGI(TAG, "%s: %d,%d %s", __func__, x, y, set ? "SET" : "CLEAR");

    if (x >= 0 && x < display->width && y >= 0 && y < display->height) {
        uint8_t *byte = &display->frame_buf[(y/8) * display->width + x];

        if (set) {
            *byte |= 1 << (y % 8);
        } else {
            *byte &= ~(1 << (y % 8));
        }
    }

    display->show(display);

    display->_unlock(display);
}
#endif

#if CONFIG_DISPLAY_LINE_ENABLED
static void display_draw_line(display_t *display, int x1, int y1, int x2, int y2, bool set)
{
//ESP_LOGI(TAG, "%s: %d,%d to %d,%d  dx %d dy %d d %d", __func__, x1, y1, x2, y2, dx, dy, d);

    display->_lock(display);

    display->hold(display);

    int dx =  abs(x2-x1);
    int sx = x1<x2 ? 1 : -1;
    int dy = -abs(y2-y1);
    int sy = y1<y2 ? 1 : -1;
    int err = dx+dy;

    do {
        display_draw_pixel(display, x1, y1, set);

        if (x1 != x2 || y1 != y2) {
            int e2 = 2*err;
            if (e2 >= dy) {
                err += dy;
                x1 += sx;
            }

            if (e2 <= dx) {
                err += dx;
                y1 += sy;
            }
        }
    } while (x1 != x2 || y1 != y2);

    display->show(display);

    display->_unlock(display);
}
#endif

#if CONFIG_DISPLAY_RECTANGLE_ENABLED
static void display_draw_rectangle(display_t *display, int x, int y, int width, int height, draw_flags_t flags)
{
//ESP_LOGI(TAG, "%s: x1 %d y1 %d x2 %d y2 %d flags %02x", __func__, x1, y1, x2, y2, flags);

    int x1 = x;
    int x2 = x + width - 1;
    int y1 = y;
    int y2 = y + height - 1;

    if (x1 < 0) {
        x1 = 0;
    } else if (x1 >= display->width) {
        x1 = display->width - 1;
    }

    if (x2 < 0) {
        x2 = 0;
    } else if (x2 >= display->width) {
        x2 = display->width - 1;
    }

    if (y1 < 0) {
        y1 = 0;
    } else if (y1 >= display->height) {
        y1 = display->height - 1;
    }

    if (y2 < 0) {
        y2 = 0;
    } else if (y2 >= display->height) {
        y2 = display->height - 1;
    }

    display->_lock(display);
    
    display->hold(display);

    if (flags & draw_flag_border) {
        display_draw_line(display, x1, y1, x2, y1, !(flags & draw_flag_clear));
        display_draw_line(display, x2, y1, x2, y2, !(flags & draw_flag_clear));
        display_draw_line(display, x2, y2, x1, y2, !(flags & draw_flag_clear));
        display_draw_line(display, x1, y2, x1, y1, !(flags & draw_flag_clear));
        x1++;
        y1++;
        x2--;
        y2--;
    }
        
//    if (flags & (draw_flag_fill | draw_flag_clear)) {
//        for (int y = y1; y <= y2; ++y) {
//            display_draw_line(display, x1, y, x2, y, !(flags & draw_flag_clear));
//        } 
//    }

    if (flags & (draw_flag_fill | draw_flag_clear)) {
        /* When  the bits pointer is 0, the bitmap is full of ones */
        bitmap_t bitmap = {
            .bits = NULL,
            .width =  width,
            .height = height,
        };

        bitmap_method_t method = (flags & draw_flag_clear) ? bitmap_method_NAND : bitmap_method_OR;

        display->draw_bitmap(display, &bitmap, x, y, width, height, method);
//        int y = y1;
//        while (y <= y2) {
//            int x = x1;
//
//            while (x <= x2) {
//               display->draw_bitmap(display, &bitmap, x, y, width - (x - x1), height - (y - y1), method);
//               x += bitmap.width;
//            } 
//
//            y += bitmap.height;
//        } 
    }

    display->show(display);

    display->_unlock(display);
}
#endif

/*
 * Draw text in rectangle at x, y
 */
static void display_draw_text(display_t *display, int x, int y, const char* text)
{
    display->_lock(display);

    display->hold(display);

    int textx = x;
    int texty = y;

    while (*text != '\0' && textx < display->width - 1) {
        bitmap_t bitmap;

        if (char_to_bitmap(&bitmap, display->font, *text)) {

            if (textx + bitmap.width >= display->width || *text == '\n') {
                /* Advance a line */
                texty += display->font_height;

                /* Reset X */
                textx = x;

                if (*text == '\n') {
                    ++text;
                }
            } else if (texty + bitmap.height <= display->height) {
                display_draw_bitmap(display, &bitmap, textx, texty, bitmap.width, bitmap.height, bitmap_method_XOR);
                textx += bitmap.width;
                ++text;
            }
        }
    }

    display->show(display);

    display->_unlock(display);
}

#if CONFIG_DISPLAY_PROGRESS_BAR_ENABLED
void display_draw_progress_bar(display_t *display, int x, int y, int width, int height, int range, int value, const char* text)
{
    /* Draw surrounding border */
    display->draw_rectangle(display, x, y, width, height, draw_flag_border);

    /* Make smaller rectangle for the moving bar */
    x += 1;
    y += 1;
    width -= 2;
    height -= 2;

    int bar = ((width - 1) * value) / range;

    display->hold(display);

    /* Paint the Progress part */
    display->draw_rectangle(display, x, y, bar, height, draw_flag_fill);

    /* Paint the non-progress part */
    if (range != value) {
        display->draw_rectangle(display, x + bar, y, width - bar, height, draw_flag_clear);
    }

    if (text != NULL) {
        int cwidth, cheight;
        text_metrics(display->font, text, &cwidth, &cheight);
        display->draw_text(display, x + width/2 - cwidth/2, y + height/2 - cheight/2, text);
    }

    display->show(display);
}
#endif

static void display_set_font(display_t *display, const font_t* font)
{
    display->_lock(display);

    display->font = font;

    /* Get a representative height */
    display->font_height = display->font->height;

    display->_unlock(display);
}

static const font_t *display_get_font(display_t *display)
{
    return display->font;
}

static void display_close(display_t* display)
{
    vSemaphoreDelete(display->mutex); 

    free((void*) display);
}

static void display_init(display_t* display, int width, int height, uint8_t flags)
{
    /* Create the frame buffer */
    display->frame_len            = (width * height) / 8;
    display->frame_buf            = (uint8_t *) malloc(display->frame_len);

ESP_LOGI(TAG, "%s: frame_buf is %p", __func__, display->frame_buf);

    memset(display->frame_buf, 0, display->frame_len);

    display->width                = width;
    display->height               = height;
    display->flags                = flags;

    display->_lock                = display_lock;
    display->_unlock              = display_unlock;

    display->hold                 = display_hold;
    display->show                 = display_show;
    display->close                = display_close;
    display->set_font             = display_set_font;
    display->get_font             = display_get_font;
    display->clear                = display_clear;
    display->draw_text            = display_draw_text;
    display->draw_bitmap          = display_draw_bitmap;

    /* Optional items below */

#if CONFIG_DISPLAY_RECTANGLE_ENABLED
    display->draw_rectangle       = display_draw_rectangle;
#endif
#if CONFIG_DISPLAY_LINE_ENABLED
    display->draw_line            = display_draw_line;
#endif
#if CONFIG_DISPLAY_PIXEL_ENABLED
    display->draw_pixel           = display_draw_pixel;
#endif
#if CONFIG_DISPLAY_PROGRESS_BAR_ENABLED
    display->draw_progress_bar    = display_draw_progress_bar;
#endif

    display->mutex = xSemaphoreCreateRecursiveMutex();
}

display_t *display_create(int width, int height, uint8_t flags)
{
    //ESP_LOGI(TAG, "%s: display_create %d,%d flags %02x", __func__, width, height, flags);

    display_t *display = (display_t*) malloc(sizeof(display_t));

ESP_LOGI(TAG, "%s: display is %p", __func__, display);

    if (display != NULL) {
        memset(display, 0, sizeof(*display));
        display_init(display, width, height, flags);
    }

    //ESP_LOGI(TAG, "%s: returning %p", __func__, display);

    return display;
}
#endif /* CONFIG_SSD1306_I2C_DISPLAY_ENABLED */
