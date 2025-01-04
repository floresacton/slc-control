#include "qmc5883.h"
#include "main.h"
#include "string.h"

extern I2C_HandleTypeDef QMC5883_HANDLE;

static uint8_t data[6];

void qmc5883_write_register(uint8_t reg, uint8_t data) {
	HAL_I2C_Mem_Write(&QMC5883_HANDLE, QMC5883_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

uint8_t qmc5883_read_register(uint8_t reg) {
	uint8_t data;
	HAL_I2C_Mem_Read(&QMC5883_HANDLE, QMC5883_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	return data;
}

void qmc5883_read_registers(uint8_t reg, uint8_t *data, uint8_t len) {
	HAL_I2C_Mem_Read(&QMC5883_HANDLE, QMC5883_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

void Qmc5883_Init(void) {
	if (qmc5883_read_register(QMC5883_REG_CHIP_ID) != QMC5883_ID){
		Error_Handler();
	}
//	set set/reset frequency (default 1)
	qmc5883_write_register(QMC5883_REG_FBR, 0b00000001);
	//set full oversampling
	//fsr of +- 2G
	//odr 200hz continuous
	qmc5883_write_register(QMC5883_REG_MODE, 0b00001101);
	//enable roll over and interrupt
	//disable soft reset
	qmc5883_write_register(QMC5883_REG_CONTROL, 0b01000000);
}

uint8_t Qmc5883_ExtFlag(uint16_t pin) {
	return pin == QMC5883_INT_PIN;
}

void Qmc5883_ExtHandler(void) {
	qmc5883_read_registers(QMC5883_REG_X_OUT_L, data, 6);
}

int16_t Qmc5883_GetX(void) {
	int16_t val;
	memcpy(&val, data, 2);
	return val;
}

int16_t Qmc5883_GetY(void) {
	int16_t val;
	memcpy(&val, data+2, 2);
	return val;
}

int16_t Qmc5883_GetZ(void) {
	int16_t val;
	memcpy(&val, data+4, 2);
	return val;
}

uint8_t* Qmc5883_GetData(void) {
	return data;
}

int16_t Qmc5883_GetTemperature(void) {
	uint8_t data[2];
	qmc5883_read_registers(QMC5883_REG_TEMP_OUT_L, data, 2);
	int16_t val;
	memcpy(&val, data, 2);
	return val;
}
