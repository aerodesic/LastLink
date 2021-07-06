/*
 * Return address of the row of font glyph bytes
 */
#include <sys/types.h>
#include "font.h"

uint8_t *char_to_bitmap(bitmap_t *bitmap, const font_t *font, int ch)
{
    uint8_t *glyph = NULL;

    if (ch >= font->first_ch && ch <= font->last_ch) {
        if ((font->flags & font_flag_pitch) == font_flag_fixed) {
            bitmap->bits = (uint8_t*) font->base + ((ch - font->first_ch) * font->width);

            /* These need to change for variable pitch fonts */
            bitmap->width = font->width;
            bitmap->height = font->height;

            /* Return the bit array for the glyph */
            glyph = bitmap->bits;
        } else {
            /* Need to do something for variable pitch fonts */
        }
    }
        
    return glyph;
}

void text_metrics(const font_t *font, const char* text, int *pwidth, int *pheight)
{
    if (pwidth != NULL) {
        *pwidth = 0;
    }

    if (pheight != NULL) {
        *pheight = 0;
    }

    while (*text != 0) {
        /* Need to change for variable pitch fonts */
        int cwidth = font->width;
        int cheight = font->height;

        if (pwidth != NULL) {
            *pwidth += cwidth;
        }

        if (pheight != NULL && cheight > *pheight) {
            *pheight = cheight;
        }
        ++text;
    }
}

