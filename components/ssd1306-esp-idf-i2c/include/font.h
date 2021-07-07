/*
 * font.h
 *
 * Defines a font entry
 */
#ifndef __fonts_h_included
#define __fonts_h_included

#include <stdbool.h>
#include <sys/types.h>
#include "bitmap.h"

typedef enum {
    font_flag_pitch    = 0x01,
    font_flag_variable = font_flag_pitch,
    font_flag_fixed    = 0x00,
} font_flags_t;
     
typedef struct {
    uint8_t      *base;
    font_flags_t flags;
    int          first_ch;
    int          last_ch;
    int          space_ch;
    int          width;
    int          height;
} font_t;

/*
 * Return address of the row of font glyph bytes
 */
uint8_t *font_char(const font_t *font, int ch, int *pwidth, int *pheight);
uint8_t *char_to_bitmap(bitmap_t *bitmap, const font_t *font, int ch);
void text_metrics(const font_t* font, const char* text, int *pwidth, int *pheight);

#endif /* __fonts_h_included */

