#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "stdint.h"
#include "font.h"
#include "main.h"

/* BEGIN CONFIG */
//#define OLED_I2C
//#define OLED_HANDLE hi2c2
//#define OLED_ADDRESS (0x3C << 1)

#define OLED_SPI
#define OLED_HANDLE hspi2 //10MHz max
#define OLED_CS_PORT CSD_GPIO_Port
#define OLED_CS_PIN CSD_Pin
#define OLED_DC_PORT DC_GPIO_Port
#define OLED_DC_PIN DC_Pin
//#define OLED_RES_PORT RES_GPIO_Port
//#define OLED_RES_PIN RES_Pin

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_BUFFER_SIZE OLED_WIDTH * OLED_HEIGHT / 8

#define OLED_X_OFFSET 0
#define OLED_X_OFFSET_LOWER (OLED_X_OFFSET & 0x0F)
#define OLED_X_OFFSET_UPPER ((OLED_X_OFFSET >> 4) & 0x07)

//#define OLED_INVERT_COLOR
//#define OLED_MIRROR_VERTICAL
//#define OLED_MIRROR_HORIZONTAL
/* END CONFIG */

#define OLED_REG_CONTRAST 0x81

enum OledColor {
    OledBlack = 0x00u,
    OledWhite = 0xFFu,
};

void Oled_Init(void);
void Oled_Update(void);

void Oled_Reset(void);

void Oled_WriteCommand(uint8_t byte);
void Oled_WriteData(uint8_t *buf, uint32_t buff_size);
void Oled_FillBuffer(uint8_t *buf, uint32_t len);

void Oled_Fill(enum OledColor color);
void Oled_DrawPixel(uint16_t x, uint16_t y);
void Oled_DrawHorizontalLine(uint8_t x1, uint8_t x2, uint8_t y);
void Oled_FillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void Oled_SetCursor(uint16_t x, uint16_t y);
void Oled_DrawChar(char chr, const struct Font* font);
void Oled_DrawString(const char *str, const struct Font* font);
void Oled_DrawBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap,
		uint16_t w, uint16_t h);

void Oled_ClearPixel(uint16_t x, uint16_t y);
void Oled_ClearRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void Oled_SetContrast(uint8_t val);
void Oled_SetPower(uint8_t on);

#endif
