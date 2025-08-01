#include "oled.h"
#include "string.h"
#include "stdlib.h"

void Oled_Reset(struct Oled_Handle* handle) {
    if (handle->hspi) {
        HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);

        if (handle->rsPort) {
            HAL_GPIO_WritePin(handle->rsPort, handle->rsPin, 0);
            HAL_Delay(10);
            HAL_GPIO_WritePin(handle->rsPort, handle->rsPin, 1);
            HAL_Delay(10);
        }
    }
}

void Oled_WriteCommand(struct Oled_Handle* handle, uint8_t command) {
    if (handle->hspi) {
        HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
        HAL_GPIO_WritePin(handle->dcPort, handle->dcPin, 0); // command
        HAL_SPI_Transmit(handle->hspi, &command, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
    } else {
        HAL_I2C_Mem_Write(handle->hi2c, handle->address, 0x00, 1, &command, 1, HAL_MAX_DELAY);
    }
}

void Oled_WriteData(struct Oled_Handle* handle, uint8_t *buf, uint32_t len) {
    if (handle->hspi) {
        HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
        HAL_GPIO_WritePin(handle->dcPort, handle->dcPin, 1); // data
        HAL_SPI_Transmit(handle->hspi, buf, len, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
    } else {
        HAL_I2C_Mem_Write(handle->hi2c, handle->address, 0x40, 1, buf, len, HAL_MAX_DELAY);
    }
}

void Oled_WriteBuffer(struct Oled_Handle* handle, uint8_t *buf, uint32_t len) {
    if (len <= handle->bufSize) {
        memcpy(handle->buf, buf, len);
    }
}

void Oled_Init(struct Oled_Handle* handle) {
    handle->bufSize = handle->width*handle->height;
    handle->buf = malloc(handle->bufSize);

    handle->offsetLower = handle->offset & 0x0F;
    handle->offsetUpper = (handle->offset >> 4) & 0x07;

    handle->cursorX = 0;
    handle->cursorY = 0;

    Oled_SetPower(handle, 0);

    Oled_WriteCommand(handle, 0x20); //Set Memory Addressing Mode
    Oled_WriteCommand(handle, 0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                // 10b,Page Addressing Mode (RESET); 11b,Invalid

    Oled_WriteCommand(handle, 0xB0); //Set Page Start Address for Page Addressing Mode,0-7

    if (handle->mirrorVertical) {
        Oled_WriteCommand(handle, 0xC0); // Mirror vertically
    }else{
        Oled_WriteCommand(handle, 0xC8); //Set COM Output Scan Direction
    }

    Oled_WriteCommand(handle, 0x00); //---set low column address
    Oled_WriteCommand(handle, 0x10); //---set high column address

    Oled_WriteCommand(handle, 0x40); //--set start line address - CHECK

    Oled_SetContrast(handle, 0xFF);

    if (handle->mirrorHorizontal) {
        Oled_WriteCommand(handle, 0xA0); // Mirror horizontally
    }else{
        Oled_WriteCommand(handle, 0xA1); //--set segment re-map 0 to 127 - CHECK
    }

    if (handle->invertColor) {
        Oled_WriteCommand(handle, 0xA7); //--set inverse color
    }else{
        Oled_WriteCommand(handle, 0xA6); //--set normal color
    }

    // Set multiplex ratio.
    // Found in the Luma Python lib for SH1106.
    if (handle->height == 128) {
        Oled_WriteCommand(handle, 0xFF);
    }else{
        Oled_WriteCommand(handle, 0xA8); //--set multiplex ratio(1 to 64) - CHECK
    }

    if (handle->height == 32) {
        Oled_WriteCommand(handle, 0x1F); //
    }else if (handle->height == 64) {
        Oled_WriteCommand(handle, 0x3F); //
    }else if (handle->height == 128) {
        Oled_WriteCommand(handle, 0x3F); // Seems to work for 128px high displays too.
    }else{
        Error_Handler(); // Only 32, 64, or 128 lines of height are supported
    }

    Oled_WriteCommand(handle, 0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    Oled_WriteCommand(handle, 0xD3); //-set display offset - CHECK
    Oled_WriteCommand(handle, 0x00); //-not offset

    Oled_WriteCommand(handle, 0xD5); //--set display clock divide ratio/oscillator frequency
    Oled_WriteCommand(handle, 0xF0); //--set divide ratio

    Oled_WriteCommand(handle, 0xD9); //--set pre-charge period
    Oled_WriteCommand(handle, 0x22); //

    Oled_WriteCommand(handle, 0xDA); //--set com pins hardware configuration - CHECK

    if (handle->height == 32) {
        Oled_WriteCommand(handle, 0x02);
    } else if (handle->height == 64) {
        Oled_WriteCommand(handle, 0x12);
    } else if (handle->height == 128) {
        Oled_WriteCommand(handle, 0x12);
    } else {
        Error_Handler(); // Only 32, 64, or 128 lines of height are supported
    }

    Oled_WriteCommand(handle, 0xDB); //--set vcomh
    Oled_WriteCommand(handle, 0x20); //0x20,0.77xVcc

    Oled_WriteCommand(handle, 0x8D); //--set DC-DC enable
    Oled_WriteCommand(handle, 0x14); //
    Oled_SetPower(handle, 1); //--turn on SSD1306 panel

    Oled_Fill(handle, Oled_ColorBlack);
}

void Oled_Update(struct Oled_Handle* handle) {
    for(uint8_t i = 0; i < handle->height/8; i++) {
        Oled_WriteCommand(handle, 0xB0 + i);
        Oled_WriteCommand(handle, 0x00 + handle->offsetLower);
        Oled_WriteCommand(handle, 0x10 + handle->offsetUpper);
        Oled_WriteData(handle, handle->buf+(handle->width*i), handle->width);
    }
}

void Oled_Fill(struct Oled_Handle* handle, enum Oled_Color color) {
    memset(handle->buf, color, handle->bufSize);
}

void Oled_ClearPixel(struct Oled_Handle* handle, uint16_t x, uint16_t y) {
    if (x >= handle->width || y >= handle->height) {
        return;
    }
    handle->buf[x + (y / 8) * handle->width] &= ~(1 << (y % 8));
}

void Oled_DrawPixel(struct Oled_Handle* handle, uint16_t x, uint16_t y) {
    if (x >= handle->width || y >= handle->height) {
    return;
    }
    handle->buf[x + (y / 8) * handle->width] |= 1 << (y % 8);
}

void Oled_DrawLine(struct Oled_Handle* handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    int16_t dx = abs((int16_t)x2 - (int16_t)x1);
    int16_t sx = x1 < x2 ? 1 : -1;
    int16_t dy = -abs((int16_t)y2 - (int16_t)y1);
    int16_t sy = y1 < y2 ? 1 : -1;
    int16_t err = dx + dy;  // error value

    while (1) {
        Oled_DrawPixel(handle, x1, y1);

        if (x1 == x2 && y1 == y2)
            break;

        int16_t e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x1 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y1 += sy;
        }
    }
}

void Oled_DrawHorizontalLine(struct Oled_Handle* handle, uint8_t x1, uint8_t x2, uint8_t y) {
    const uint8_t mask = 1 << (y % 8);
    const uint16_t offset = (y / 8) * handle->width;
    for (uint8_t x = x1; x < x2; x++) {
        handle->buf[x + offset] |= mask;
    }
}

void Oled_ClearRectangle(struct Oled_Handle* handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    if (y2 > handle->height){
        y2 = handle->height;
    }
    if (x2 > handle->width){
        x2 = handle->width;
    }
    for (uint16_t y = y1; y < y2; y++) {
        for (uint16_t x = x1; x < x2; x++) {
            Oled_ClearPixel(handle, x, y);
    }
    }
}

void Oled_FillRectangle(struct Oled_Handle* handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    if (y2 > handle->height){
        y2 = handle->height;
    }
    if (x2 > handle->width){
        x2 = handle->width;
    }
    for (uint16_t y = y1; y < y2; y++) {
        for (uint16_t x = x1; x < x2; x++) {
            Oled_DrawPixel(handle, x, y);
        }
    }
}

void Oled_SetCursor(struct Oled_Handle* handle, uint16_t x, uint16_t y) {
    handle->cursorX = x;
    handle->cursorY = y;
}

void Oled_DrawChar(struct Oled_Handle* handle, char chr, const struct Font* font) {
    if (chr < FONT_START || chr >= FONT_CHARS + FONT_START) {
        return;
    }

    if (handle->width < (handle->cursorX + font->width) || handle->height < (handle->cursorY + font->height)) {
        return;
    }

    for (uint16_t y = 0; y < font->height; y++) {
        const uint8_t data = font->data[(chr - FONT_START) * font->height + y];
    for (uint16_t x = 0; x < font->width; x++) {
            if ((data << x) & 0x80) {
                Oled_DrawPixel(handle, handle->cursorX + x, handle->cursorY + y);
            }
        }
    }

    handle->cursorX += font->width;
}

void Oled_DrawString(struct Oled_Handle* handle, const char *str, const struct Font* font) {
    while (*str) {
        Oled_DrawChar(handle, *str, font);
        str++;
    }
}

void Oled_DrawBitmap(struct Oled_Handle* handle, uint16_t x, uint16_t y, const uint8_t* bitmap, uint16_t w, uint16_t h) {
    int16_t byteWidth = (w + 7) / 8;
    uint8_t byte = 0;

    if (x >= handle->width || y >= handle->height) {
        return;
    }

    for (uint8_t j = 0; j < h; j++, y++) {
        for (uint8_t i = 0; i < w; i++) {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }

            if (byte & 0x80) {
                Oled_DrawPixel(handle, x + i, y);
            }
        }
    }
}

void Oled_SetContrast(struct Oled_Handle* handle, uint8_t val) {
    Oled_WriteCommand(handle, 0x81);
    Oled_WriteCommand(handle, val);
}

void Oled_SetPower(struct Oled_Handle* handle, uint8_t state) {
    Oled_WriteCommand(handle, 0xAE | state);
}
