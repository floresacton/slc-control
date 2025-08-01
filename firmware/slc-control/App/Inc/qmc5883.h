#ifndef INC_QMC5883_H_
#define INC_QMC5883_H_

#include "stdint.h"
#include "main.h"

#define QMC5883_ADDRESS 0x1A
#define QMC5883_ID 0xFF

#define QMC5883_REG_X_OUT_L 0x00 //r
#define QMC5883_REG_X_OUT_H 0x01 //r
#define QMC5883_REG_Y_OUT_L 0x02 //r
#define QMC5883_REG_Y_OUT_H 0x03 //r
#define QMC5883_REG_Z_OUT_L 0x04 //r
#define QMC5883_REG_Z_OUT_H 0x05 //r

#define QMC5883_REG_STATUS 0x06 //r

#define QMC5883_REG_TEMP_OUT_L 0x07 //r
#define QMC5883_REG_TEMP_OUT_H 0x08 //r

#define QMC5883_REG_MODE 0x09 //r-w

#define QMC5883_REG_CONTROL 0x0A //r-w

#define QMC5883_REG_FBR 0x0B //r-w

#define QMC5883_REG_CHIP_ID 0x0D //r

enum Qmc5883_Mode {
    Qmc5883_ModeStandby = 0u,
    Qmc5883_ModeCont = 1u,
};

enum Qmc5883_Odr {
    Qmc5883_Odr10 = 0u,
    Qmc5883_Odr50 = 1u,
    Qmc5883_Odr100 = 2u,
    Qmc5883_Odr200 = 3u,
};

enum Qmc5883_Scale {
    Qmc5883_Scale2G = 0u,
    Qmc5883_Scale8G = 1u,
};

enum Qmc5883_Osr {
    Qmc5883_Osr512 = 0u,
    Qmc5883_Osr256 = 1u,
    Qmc5883_Osr128 = 2u,
    Qmc5883_Osr64 = 3u,
};

struct Qmc5883_Handle {
    // configuration
    I2C_HandleTypeDef* hi2c;
    uint16_t intPin;

    // temperature is only relative
    uint8_t readTemp;

    // internal
    uint8_t init;

    uint8_t* data;

    float temperature;
    float x, y, z;
    float angle;
};

void Qmc5883_Init(struct Qmc5883_Handle* handle);

uint8_t Qmc5883_ExtFlag(struct Qmc5883_Handle* handle, uint16_t pin);
void Qmc5883_ExtHandler(struct Qmc5883_Handle* handle);

void Qmc5583_Reset(struct Qmc5883_Handle* handle);

void Qmc5583_SetMode(struct Qmc5883_Handle* handle, enum Qmc5883_Mode mode);
void Qmc5583_SetODR(struct Qmc5883_Handle* handle, enum Qmc5883_Odr odr);
void Qmc5583_SetScale(struct Qmc5883_Handle* handle, enum Qmc5883_Scale scale);
void Qmc5583_SetOSR(struct Qmc5883_Handle* handle, enum Qmc5883_Osr osr);

void Qmc5883_SetInterrupt(struct Qmc5883_Handle* handle, uint8_t state);
void Qmc5883_SetRollover(struct Qmc5883_Handle* handle, uint8_t state);

void Qmc5883_SetFBR(struct Qmc5883_Handle* handle, uint8_t value);

#endif
