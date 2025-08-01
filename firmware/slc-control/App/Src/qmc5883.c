#include "qmc5883.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

static void qmc5883_write_register(struct Qmc5883_Handle* handle, uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(handle->hi2c, QMC5883_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

static uint8_t qmc5883_read_register(struct Qmc5883_Handle* handle, uint8_t reg) {
    uint8_t data;
    HAL_I2C_Mem_Read(handle->hi2c, QMC5883_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

static void qmc5883_read_registers(struct Qmc5883_Handle* handle, uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_I2C_Mem_Read(handle->hi2c, QMC5883_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

void Qmc5883_Init(struct Qmc5883_Handle* handle) {
    if (handle->readTemp) {
        handle->data = malloc(8);
    }else{
        handle->data = malloc(6);
    }

    const uint8_t whoami = qmc5883_read_register(handle, QMC5883_REG_CHIP_ID);
    if (whoami != QMC5883_ID) {
        Error_Handler();
    }
    Qmc5583_Reset(handle);

    Qmc5583_SetMode(handle, Qmc5883_ModeCont);
    Qmc5583_SetODR(handle, Qmc5883_Odr100);
    Qmc5583_SetScale(handle, Qmc5883_Scale2G);
    Qmc5583_SetOSR(handle, Qmc5883_Osr512);
    Qmc5883_SetFBR(handle, 1); // datasheet recommended

    Qmc5883_SetRollover(handle, 1);
    Qmc5883_SetInterrupt(handle, 1);

    handle->init = 1;

    Qmc5883_ExtHandler(handle);
}

uint8_t Qmc5883_ExtFlag(struct Qmc5883_Handle* handle, uint16_t pin) {
    return handle->intPin == pin;
}

void Qmc5883_ExtHandler(struct Qmc5883_Handle* handle) {
    if (handle->init) {
        qmc5883_read_registers(handle, QMC5883_REG_X_OUT_L, handle->data, 6);
        if (handle->readTemp) {
            qmc5883_read_registers(handle, QMC5883_REG_TEMP_OUT_L, handle->data+6, 2);
            
            int16_t temp;
            memcpy(&temp, handle->data+6, 2);
            handle->temperature = (float)temp / 500.0f * 9.0f + 32.0f; // F
        }
        int16_t x, y, z;
        memcpy(&x, handle->data, 2);
        memcpy(&y, handle->data+2, 2);
        memcpy(&z, handle->data+4, 2);

        handle->x = (float)x / 12000.0f;
        handle->y = (float)y / 12000.0f;
        handle->z = (float)z / 12000.0f;

        handle->angle = atan2(handle->y, handle->z) - 3.1415927f / 2.0f;

//      int16_t data[3];
//      qmc5883_read_registers(handle, QMC5883_REG_X_OUT_L, (uint8_t*)data, 6);
//      struct Vector3f vec = {.x = data[0], .y = data[1], .z = data[2]};
//      Algebra_Add3f(&vec, handle->bias);
//      Algebra_Transform3f(&vec, handle->transform);
//      memcpy(handle->vec, &vec, sizeof(struct Vector3f));
    }
}

void Qmc5583_Reset(struct Qmc5883_Handle* handle) {
    const uint8_t val = qmc5883_read_register(handle, QMC5883_REG_CONTROL);
    qmc5883_write_register(handle, QMC5883_REG_CONTROL, val | 0x80);
    while (1) {
        HAL_Delay(1);
        if (!(qmc5883_read_register(handle, QMC5883_REG_CONTROL) >> 7)) {
            break;
        }
    }
}

void Qmc5583_SetMode(struct Qmc5883_Handle* handle, enum Qmc5883_Mode mode) {
    uint8_t val = qmc5883_read_register(handle, QMC5883_REG_MODE);
    val = (val & 0xfc) | mode;
    qmc5883_write_register(handle, QMC5883_REG_MODE, val);
}

void Qmc5583_SetODR(struct Qmc5883_Handle* handle, enum Qmc5883_Odr odr) {
    uint8_t val = qmc5883_read_register(handle, QMC5883_REG_MODE);
    val = (val & 0xf3) | (odr << 2);
    qmc5883_write_register(handle, QMC5883_REG_MODE, val);
}

void Qmc5583_SetScale(struct Qmc5883_Handle* handle, enum Qmc5883_Scale scale) {
    uint8_t val = qmc5883_read_register(handle, QMC5883_REG_MODE);
    val = (val & 0xcf) | (scale << 4);
    qmc5883_write_register(handle, QMC5883_REG_MODE, val);
}

void Qmc5583_SetOSR(struct Qmc5883_Handle* handle, enum Qmc5883_Osr osr) {
    uint8_t val = qmc5883_read_register(handle, QMC5883_REG_MODE);
    val = (val & 0x3f) | (osr << 6);
    qmc5883_write_register(handle, QMC5883_REG_MODE, val);
}

void Qmc5883_SetInterrupt(struct Qmc5883_Handle* handle, uint8_t state) {
    uint8_t val = qmc5883_read_register(handle, QMC5883_REG_CONTROL);
    val = (val & 0xc0) | (!state);
    qmc5883_write_register(handle, QMC5883_REG_CONTROL, val);
}

void Qmc5883_SetRollover(struct Qmc5883_Handle* handle, uint8_t state) {
    uint8_t val = qmc5883_read_register(handle, QMC5883_REG_CONTROL);
    val = (val & 0x81) | (state << 6);
    qmc5883_write_register(handle, QMC5883_REG_CONTROL, val);
}

void Qmc5883_SetFBR(struct Qmc5883_Handle* handle, uint8_t value) {
    qmc5883_write_register(handle, QMC5883_REG_FBR, value);
}
