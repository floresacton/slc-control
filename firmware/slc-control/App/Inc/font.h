#ifndef INC_FONT_H_
#define INC_FONT_H_

#include "stdint.h"

#define FONT_CHARS 95
#define FONT_START 32

struct Font {
    const uint16_t width;
    const uint16_t height;
    const uint8_t scale; // width (bytes)
    const uint8_t *data;
};

extern const struct Font Font_7x10;
extern const struct Font Font_6x8;

#endif
