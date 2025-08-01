#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stdint.h"
#include "main.h"

struct Eeprom_Handle {
    // configuration
    I2C_HandleTypeDef* hi2c;
    uint8_t address;

    uint16_t pages;
    uint8_t pageSize;
};

void Eeprom_Read(struct Eeprom_Handle* handle, uint16_t address, uint8_t *buf, uint16_t len);
uint8_t Eeprom_ReadByte(struct Eeprom_Handle* handle, uint16_t address);

//wraps on page
void Eeprom_Write(struct Eeprom_Handle* handle, uint16_t address, uint8_t *buf, uint16_t len);
void Eeprom_WriteByte(struct Eeprom_Handle* handle, uint16_t address, uint8_t val);

//wraps on memory
void Eeprom_ReadNext(struct Eeprom_Handle* handle, uint8_t *buf, uint8_t len);

#endif
