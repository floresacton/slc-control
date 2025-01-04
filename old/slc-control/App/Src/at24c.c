#include "at24c.h"
#include "main.h"

extern I2C_HandleTypeDef AT24C_HANDLE;

void At24c_Write(uint16_t address, uint8_t *buf, uint16_t len) {
	HAL_I2C_Mem_Write(&AT24C_HANDLE, AT24C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, buf, len, HAL_MAX_DELAY);
}

void At24c_ReadNext(uint8_t *buf, uint8_t len) {
	HAL_I2C_Master_Receive(&AT24C_HANDLE, AT24C_ADDRESS, buf, len, HAL_MAX_DELAY);
}

void At24c_Read(uint16_t address, uint8_t *buf, uint16_t len) {
	HAL_I2C_Mem_Read(&AT24C_HANDLE, AT24C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, buf, len, HAL_MAX_DELAY);
}
