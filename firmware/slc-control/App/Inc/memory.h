#ifndef INC_MEMORY_H_
#define INC_MEMORY_H_

#include "stdint.h"
#include "eeprom.h"

struct Memory_Variable {
    // configuration
    float reset;

    float min;
    float max;

    int8_t decimals; // can be negative

    // internal
    int8_t maxDigit;

    float value;
    uint16_t address;
};

struct Memory_Handle {
    // configuration
    struct Eeprom_Handle* eeprom;

    float hash;

    struct Memory_Variable** vars;
    uint8_t count;
};

void Memory_Init(struct Memory_Handle* handle);

void Memory_Reset(struct Memory_Handle* handle);
void Memory_Save(struct Memory_Handle* handle, struct Memory_Variable* var);

uint8_t Memory_Print(char* buf, struct Memory_Variable* var);

#endif
