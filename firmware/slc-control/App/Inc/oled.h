#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "stdint.h"
#include "font.h"
#include "main.h"

enum Oled_Color {
    Oled_ColorBlack = 0x00u,
    Oled_ColorWhite = 0xFFu,
};

struct Oled_Handle {
    I2C_HandleTypeDef* hi2c;
    uint8_t address;

    SPI_HandleTypeDef* hspi; // 10MHz max

    GPIO_TypeDef* csPort;
    uint16_t csPin;
    //command
    GPIO_TypeDef* dcPort;
    uint16_t dcPin;
    //reset
    GPIO_TypeDef* rsPort;
    uint16_t rsPin;

    uint8_t width;
    uint8_t height;

    uint8_t offset; // x offset

    // booleans
    uint8_t invertColor;
    uint8_t mirrorVertical;
    uint8_t mirrorHorizontal;

    // internal
    uint8_t* buf;
    uint16_t bufSize;

    uint8_t offsetLower;
    uint8_t offsetUpper;

    uint8_t cursorX;
    uint8_t cursorY;

};

void Oled_Init(struct Oled_Handle* oled);
void Oled_Update(struct Oled_Handle* oled);

void Oled_Reset(struct Oled_Handle* oled);

void Oled_WriteCommand(struct Oled_Handle* oled, uint8_t command);
void Oled_WriteData(struct Oled_Handle* oled, uint8_t *buf, uint32_t len);
void Oled_WriteBuffer(struct Oled_Handle* oled, uint8_t *buf, uint32_t len);

void Oled_Fill(struct Oled_Handle* oled, enum Oled_Color color);
void Oled_DrawPixel(struct Oled_Handle* oled, uint16_t x, uint16_t y);
void Oled_DrawLine(struct Oled_Handle* oled, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void Oled_DrawHorizontalLine(struct Oled_Handle* oled, uint8_t x1, uint8_t x2, uint8_t y);
void Oled_FillRectangle(struct Oled_Handle* oled, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void Oled_SetCursor(struct Oled_Handle* oled, uint16_t x, uint16_t y);
void Oled_DrawChar(struct Oled_Handle* oled, char chr, const struct Font* font);
void Oled_DrawString(struct Oled_Handle* oled, const char *str, const struct Font* font);
void Oled_DrawBitmap(struct Oled_Handle* oled, uint16_t x, uint16_t y, const uint8_t* bitmap,
        uint16_t w, uint16_t h);

void Oled_ClearPixel(struct Oled_Handle* oled, uint16_t x, uint16_t y);
void Oled_ClearRectangle(struct Oled_Handle* oled, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void Oled_SetContrast(struct Oled_Handle* oled, uint8_t val);
void Oled_SetPower(struct Oled_Handle* oled, uint8_t state);

#endif
