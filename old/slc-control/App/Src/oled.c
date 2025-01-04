#include "oled.h"
#include "stm32g4xx_hal.h"
#include "string.h"

static uint8_t oled_buffer[OLED_BUFFER_SIZE] = { 0 };
static uint16_t cursorX = 0;
static uint16_t cursorY = 0;

#ifdef OLED_I2C
extern I2C_HandleTypeDef OLED_HANDLE;

void Oled_Reset(void) {
	//do nothing
}

void Oled_WriteCommand(uint8_t byte) {
	HAL_I2C_Mem_Write(&OLED_HANDLE, OLED_ADDRESS, 0x00, 1, &byte, 1,
	HAL_MAX_DELAY);
}

void Oled_WriteData(uint8_t *buffer, size_t buff_size) {
	HAL_I2C_Mem_Write(&OLED_HANDLE, OLED_ADDRESS, 0x40, 1, buffer, buff_size,
	HAL_MAX_DELAY);
}

#elif defined(OLED_SPI)
extern SPI_HandleTypeDef OLED_HANDLE;

void Oled_Reset(void) {
	HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, 1);

	// Reset the OLED
#ifdef OLED_RES_PORT
	    HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, 0);
	    HAL_Delay(10);
	    HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, 1);
	    HAL_Delay(10);
#endif
}

void Oled_WriteCommand(uint8_t byte) {
	HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, 0);
	HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, 0); // command
	HAL_SPI_Transmit(&OLED_HANDLE, (uint8_t *) &byte, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, 1);
}

void Oled_WriteData(uint8_t *buf, uint32_t len) {
	HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, 0);
	HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, 1); // data
	HAL_SPI_Transmit(&OLED_HANDLE, buf, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, 1);
}

#endif

void Oled_FillBuffer(uint8_t *buf, uint32_t len) {
	if (len <= OLED_BUFFER_SIZE) {
		memcpy(oled_buffer, buf, len);
	}
}

void Oled_Init(void) {
	Oled_Reset();
	HAL_Delay(100);
	Oled_SetPower(0);

    Oled_WriteCommand(0x20); //Set Memory Addressing Mode
    Oled_WriteCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                // 10b,Page Addressing Mode (RESET); 11b,Invalid

    Oled_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

#ifdef OLED_MIRROR_VERT
    Oled_WriteCommand(0xC0); // Mirror vertically
#else
    Oled_WriteCommand(0xC8); //Set COM Output Scan Direction
#endif

    Oled_WriteCommand(0x00); //---set low column address
    Oled_WriteCommand(0x10); //---set high column address

    Oled_WriteCommand(0x40); //--set start line address - CHECK

    Oled_SetContrast(0xFF);

#ifdef OLED_MIRROR_HORIZ
    Oled_WriteCommand(0xA0); // Mirror horizontally
#else
    Oled_WriteCommand(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef OLED_INVERT_COLOR
    Oled_WriteCommand(0xA7); //--set inverse color
#else
    Oled_WriteCommand(0xA6); //--set normal color
#endif

// Set multiplex ratio.
#if (OLED_HEIGHT == 128)
    // Found in the Luma Python lib for SH1106.
    Oled_WriteCommand(0xFF);
#else
    Oled_WriteCommand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
#endif

#if (OLED_HEIGHT == 32)
    Oled_WriteCommand(0x1F); //
#elif (OLED_HEIGHT == 64)
    Oled_WriteCommand(0x3F); //
#elif (OLED_HEIGHT == 128)
    Oled_WriteCommand(0x3F); // Seems to work for 128px high displays too.
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    Oled_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    Oled_WriteCommand(0xD3); //-set display offset - CHECK
    Oled_WriteCommand(0x00); //-not offset

    Oled_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
    Oled_WriteCommand(0xF0); //--set divide ratio

    Oled_WriteCommand(0xD9); //--set pre-charge period
    Oled_WriteCommand(0x22); //

    Oled_WriteCommand(0xDA); //--set com pins hardware configuration - CHECK
#if (OLED_HEIGHT == 32)
    Oled_WriteCommand(0x02);
#elif (OLED_HEIGHT == 64)
    Oled_WriteCommand(0x12);
#elif (OLED_HEIGHT == 128)
    Oled_WriteCommand(0x12);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    Oled_WriteCommand(0xDB); //--set vcomh
    Oled_WriteCommand(0x20); //0x20,0.77xVcc

    Oled_WriteCommand(0x8D); //--set DC-DC enable
    Oled_WriteCommand(0x14); //
    Oled_SetPower(1); //--turn on SSD1306 panel

    Oled_Fill(OledBlack);
}

void Oled_Update(void) {
	for(uint8_t i = 0; i < OLED_HEIGHT/8; i++) {
		Oled_WriteCommand(0xB0 + i);
		Oled_WriteCommand(0x00 + OLED_X_OFFSET_LOWER);
		Oled_WriteCommand(0x10 + OLED_X_OFFSET_UPPER);
		Oled_WriteData(&oled_buffer[OLED_WIDTH*i], OLED_WIDTH);
	}
}

void Oled_Fill(enum OledColor color) {
	memset(oled_buffer, color, OLED_BUFFER_SIZE);
}

void Oled_ClearPixel(uint16_t x, uint16_t y) {
	if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
		return;
	}
	oled_buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
}

void Oled_DrawPixel(uint16_t x, uint16_t y) {
	if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
		return;
	}
	oled_buffer[x + (y / 8) * OLED_WIDTH] |= 1 << (y % 8);
}

void Oled_DrawHorizontalLine(uint8_t x1, uint8_t x2, uint8_t y) {
	const uint8_t mask = 1 << (y % 8);
	const uint16_t offset = (y / 8) * OLED_WIDTH;
	for (uint8_t x = x1; x < x2; x++) {
		oled_buffer[x + offset] |= mask;
	}
}

void Oled_ClearRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	if (y2 > OLED_HEIGHT){
		y2 = OLED_HEIGHT;
	}
	if (x2 > OLED_WIDTH){
		x2 = OLED_WIDTH;
	}
	for (uint16_t y = y1; y < y2; y++) {
		for (uint16_t x = x1; x < x2; x++) {
			Oled_ClearPixel(x, y);
		}
	}
}

void Oled_FillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	if (y2 > OLED_HEIGHT){
		y2 = OLED_HEIGHT;
	}
	if (x2 > OLED_WIDTH){
		x2 = OLED_WIDTH;
	}
	for (uint16_t y = y1; y < y2; y++) {
		for (uint16_t x = x1; x < x2; x++) {
			Oled_DrawPixel(x, y);
		}
	}
}

void Oled_SetCursor(uint16_t x, uint16_t y) {
	cursorX = x;
	cursorY = y;
}

void Oled_DrawChar(char chr, const struct Font* font) {
	if (chr < FONT_START || chr >= FONT_CHARS + FONT_START) {
		return;
	}

	if (OLED_WIDTH < (cursorX + font->width) ||
	OLED_HEIGHT < (cursorY + font->height)) {
		return;
	}

	for (uint16_t y = 0; y < font->height; y++) {
		const uint8_t data = font->data[(chr - FONT_START) * font->height + y];
		for (uint16_t x = 0; x < font->width; x++) {
			if ((data << x) & 0x80) {
				Oled_DrawPixel(cursorX + x, cursorY + y);
			}
		}
	}

	cursorX += font->width;
}

void Oled_DrawString(const char *str, const struct Font* font) {
	while (*str) {
		Oled_DrawChar(*str, font);
		str++;
	}
}

void Oled_DrawBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap,
		uint16_t w, uint16_t h) {
	int16_t byteWidth = (w + 7) / 8;
	    uint8_t byte = 0;

	    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
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
	                Oled_DrawPixel(x + i, y);
	            }
	        }
	    }
}

void Oled_SetContrast(const uint8_t val) {
	Oled_WriteCommand(OLED_REG_CONTRAST);
	Oled_WriteCommand(val);
}

void Oled_SetPower(const uint8_t on) {
	Oled_WriteCommand(0xAE | on);
}
