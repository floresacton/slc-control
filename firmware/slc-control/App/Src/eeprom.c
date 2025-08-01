#include "eeprom.h"

void Eeprom_Read(struct Eeprom_Handle* handle, uint16_t address, uint8_t *buf, uint16_t len) {
    HAL_I2C_Mem_Read(handle->hi2c, handle->address, address, I2C_MEMADD_SIZE_16BIT, buf, len, HAL_MAX_DELAY);
}
uint8_t Eeprom_ReadByte(struct Eeprom_Handle* handle, uint16_t address) {
    uint8_t val;
    HAL_I2C_Mem_Read(handle->hi2c, handle->address, address, I2C_MEMADD_SIZE_16BIT, &val, 1, HAL_MAX_DELAY);
    return val;
}

void Eeprom_Write(struct Eeprom_Handle* handle, uint16_t address, uint8_t *buf, uint16_t len) {
    HAL_I2C_Mem_Write(handle->hi2c, handle->address, address, I2C_MEMADD_SIZE_16BIT, buf, len, HAL_MAX_DELAY);
}
void Eeprom_WriteByte(struct Eeprom_Handle* handle, uint16_t address, uint8_t val) {
    HAL_I2C_Mem_Write(handle->hi2c, handle->address, address, I2C_MEMADD_SIZE_16BIT, &val, 1, HAL_MAX_DELAY);
}

void Eeprom_ReadNext(struct Eeprom_Handle* handle, uint8_t *buf, uint8_t len) {
    HAL_I2C_Master_Receive(handle->hi2c, handle->address, buf, len, HAL_MAX_DELAY);
}
