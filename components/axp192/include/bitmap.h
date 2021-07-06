/*
 * bitmap.h
 *
 * Super simple bitmap for glyphs and simple bit images.
 */
#ifndef __bitmap_h_included
#define __bitmap_h_included

typedef struct {
    int     width;
    int     height;
    uint8_t *bits;
} bitmap_t;

/* Blending method for bitmap images */
typedef enum {
    bitmap_method_OR,
    bitmap_method_XOR,
    bitmap_method_NAND,
} bitmap_method_t;

#endif /* __bitmap_h_included */

