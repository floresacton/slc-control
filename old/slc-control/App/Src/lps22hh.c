#include "lps22hh.h"
#include "string.h"
#include "main.h"
#include "usbd_cdc_if.h"

extern SPI_HandleTypeDef LPS22HH_HANDLE;

#define T(name) name##_GPIO_Port,
static GPIO_TypeDef *csPorts[LPS22HH_COUNT] = { LPS22HH_LIST };
#undef T
#define T(name) name##_Pin,
static uint16_t csPins[LPS22HH_COUNT] = { LPS22HH_LIST };
#undef T

static uint8_t data[LPS22HH_COUNT*LPS22HH_DATA_SIZE] = {0};
static uint32_t* pressures[LPS22HH_COUNT];
static int16_t* temperatures[LPS22HH_COUNT];

void lps22hh_write_register(enum Lps22hhId lps, uint8_t reg, uint8_t data) {
	HAL_GPIO_WritePin(csPorts[lps], csPins[lps], 0);
	HAL_SPI_Transmit(&LPS22HH_HANDLE, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&LPS22HH_HANDLE, &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(csPorts[lps], csPins[lps], 1);
}

uint8_t lps22hh_read_register(enum Lps22hhId lps, uint8_t reg) {
	uint8_t data = 0;
	uint8_t regRW = 0x80 | reg;
	HAL_Delay(1);
	HAL_GPIO_WritePin(csPorts[lps], csPins[lps], 0);
	HAL_SPI_Transmit(&LPS22HH_HANDLE, &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&LPS22HH_HANDLE, &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(csPorts[lps], csPins[lps], 1);
	HAL_Delay(1);
	return data;
}

void lps22hh_read_registers(enum Lps22hhId lps, uint8_t reg, uint8_t *output, uint16_t len) {
	uint8_t regRW = 0x80 | reg;
	HAL_GPIO_WritePin(csPorts[lps], csPins[lps], 0);
	HAL_SPI_Transmit(&LPS22HH_HANDLE, &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&LPS22HH_HANDLE, output, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(csPorts[lps], csPins[lps], 1);
}

void Lps22hh_Init(void) {
	for (uint8_t i = 0; i < LPS22HH_COUNT; i++) {
		const uint8_t whoami = lps22hh_read_register(i, LPS22HH_REG_WHO_AM_I);
		if (whoami != LPS22HH_ID) {
			Error_Handler();
		}

		//configure ODR 200hz
		//enable low pass filter at bandwidth odr/9
//		lps22hh_write_register(cs, LPS22HH_REG_CTRL_REG1, 0b01110000);

		//configure ODR 200hz ODR/20 filter
		Lps22hh_SetODR(i, Lps22hhODR10Hz);
		lps22hh_write_register(i, LPS22HH_REG_CTRL_REG1, 0b00111100);
	}
}

void Lps22hh_Update(void) {
	for (uint8_t i = 0; i < LPS22HH_COUNT; i++) {
		lps22hh_read_registers(i, LPS22HH_REG_PRESS_XL, data+i*LPS22HH_DATA_SIZE, LPS22HH_DATA_SIZE);
		pressures[i] = (uint32_t*)(data+i*LPS22HH_DATA_SIZE);
		temperatures[i] = (int16_t*)(data+i*LPS22HH_DATA_SIZE+3);
	}
}

uint8_t* Lps22hh_GetData(void) {
	return data;
}

uint32_t Lps22hh_GetPressure(enum Lps22hhId lps) {
	return (*pressures[lps])&0x00FFFFFF;
}

int16_t Lps22hh_GetTemperature(enum Lps22hhId lps) {
	return *temperatures[lps];
}

void Lps22hh_SetODR(enum Lps22hhId lps, enum Lps22hhODR odr) {
	const uint8_t val = lps22hh_read_register(lps, LPS22HH_REG_CTRL_REG1);
	lps22hh_write_register(lps, LPS22HH_REG_CTRL_REG1, (val&0x0F)|(odr<<4));
}
void Lps22hh_SetFilter(enum Lps22hhFilter filter) {

}

#ifdef LPS22HH_INT_PIN
uint8_t Lps22hh_ExtFlag(uint16_t pin) {
	return pin == LPS22HH_INT_PIN;
}

void Lps22hh_ExtHandler(void) {
	Lps22hh_Update();
}
#endif
