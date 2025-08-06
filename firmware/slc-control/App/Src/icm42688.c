#include "icm42688.h"
#include "stdlib.h"
#include "string.h"

static void icm42688_write_register(struct Icm42688_Handle* handle, uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
    HAL_SPI_Transmit(handle->hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(handle->hspi, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
}

static void icm42688_write_registers(struct Icm42688_Handle* handle, uint8_t reg, uint8_t* data, uint8_t len) {
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
    HAL_SPI_Transmit(handle->hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(handle->hspi, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
}

static uint8_t icm42688_read_register(struct Icm42688_Handle* handle, uint8_t reg) {
    uint8_t data = 0;
    uint8_t regRW = 0x80 | reg;
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
    HAL_SPI_Transmit(handle->hspi, &regRW, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(handle->hspi, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
    return data;
}

static void icm42688_read_registers(struct Icm42688_Handle* handle, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t regRW = 0x80 | reg;
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
    HAL_SPI_Transmit(handle->hspi, &regRW, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(handle->hspi, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
}

static void icm42688_write_bit(struct Icm42688_Handle* handle, uint8_t reg, uint8_t bit, uint8_t value) {
    const uint8_t val = icm42688_read_register(handle, reg);
    icm42688_write_register(handle, reg, (val & ~(1 << bit)) | (value << bit));
}

static void icm42688_write_bits(struct Icm42688_Handle* handle, uint8_t reg, uint8_t startBit, uint8_t value, uint8_t len) {
    const uint8_t val = icm42688_read_register(handle, reg);
    const uint8_t mask = ((1 << len) - 1) << startBit;
    icm42688_write_register(handle, reg, (val & ~mask) | (value << startBit));
}

static uint8_t icm42688_read_bit(struct Icm42688_Handle* handle, uint8_t reg, uint8_t bit) {
    return (icm42688_read_register(handle, reg) >> bit) & 1;
}

void Icm42688_Init(struct Icm42688_Handle* handle) {
    handle->data = malloc(20);

    const uint8_t whoami = icm42688_read_register(handle, ICM42688_REG_WHO_AM_I);
    if (whoami != ICM42688_ID) {
        Error_Handler();
    }
    Icm42688_Reset(handle);
//
    Icm42688_SetInt(handle, Icm42688_Int1, Icm42688_IntModePulse, Icm42688_IntDrivePush, Icm42688_IntPolarityHigh);
    Icm42688_SetFifo(handle, Icm42688_FifoModeStream);
    Icm42688_SetFifoHoldData(handle, 0);
    Icm42688_SetFifoCountMode(handle, Icm42688_FifoCountModeRecord);
    Icm42688_SetEndian(handle, Icm42688_EndianLittle);
//
    // defaults to this anyways in fifo hires mode
    Icm42688_SetGyroFsr(handle, Icm42688_GyroFsr2000);
    Icm42688_SetAccelFsr(handle, Icm42688_AccelFsr16);

    Icm42688_SetGyroOdr(handle, Icm42688_Odr50);
    Icm42688_SetAccelOdr(handle, Icm42688_Odr50);

    Icm42688_SetTempFilt(handle, Icm42688_TempFilt4k);

    Icm42688_SetGyroFiltOrder(handle, Icm42688_FiltOrder3);
    Icm42688_SetGyroFilt(handle, Icm42688_Filt2);
    Icm42688_SetAccelFiltOrder(handle, Icm42688_FiltOrder3);
    Icm42688_SetAccelFilt(handle, Icm42688_Filt2);

    Icm42688_SetTimeRes(handle, Icm42688_TimeRes1);
    Icm42688_SetTimeDelta(handle, 1);
    Icm42688_SetTimeFsync(handle, 0);

    Icm42688_SetFifoHires(handle, 1);
    Icm42688_SetFifoTemp(handle, 1);
    Icm42688_SetFifoGyro(handle, 1);
    Icm42688_SetFifoAccel(handle, 1);

    Icm42688_SetFifoIntThresh(handle, 1);
    Icm42688_SetIntDuration(handle, Icm42688_IntDuration8);
    Icm42688_SetIntDeassert(handle, 0);
    Icm42688_SetIntAsync(handle, 0);

    Icm42688_SetIntSource(handle, Icm42688_Int1, Icm42688_IntSourceThresh);
    Icm42688_SetIntSource(handle, Icm42688_Int2, Icm42688_IntSourceNone);

    Icm42688_SetGyroMode(handle, Icm42688_GyroModeLowNoise);
    Icm42688_SetAccelMode(handle, Icm42688_AccelModeLowNoise);

    Icm42688_ResetSignalPath(handle);
    Icm42688_FlushFifo(handle);

    handle->init = 1;
    handle->skip = 0;
}

uint8_t Icm42688_ExtFlag(struct Icm42688_Handle* handle, uint16_t pin) {
    return handle->intPin == pin;
}

void Icm42688_ExtHandler(struct Icm42688_Handle* handle) {
    if (handle->init) {
        icm42688_read_registers(handle, ICM42688_REG_FIFO_DATA, handle->data, 20);
        Icm42688_FlushFifo(handle);
        int16_t ax, ay, az, temp;
        memcpy(&ax, handle->data+1, 2);
        memcpy(&ay, handle->data+3, 2);
        memcpy(&az, handle->data+5, 2);
        memcpy(&temp, handle->data+13, 2);
        handle->accelx = (float)ax / 2048.0f;
        handle->accely = (float)ay / 2048.0f;
        handle->accelz = (float)az / 2048.0f;
        //handle->temperature = (float)temp / 132.48f + 25.0f;
    }
}

void Icm42688_Reset(struct Icm42688_Handle* handle) {
    icm42688_write_bit(handle, ICM42688_REG_DEVICE_CONFIG, 0, 1);
    while (1) {
        HAL_Delay(1);
        if (!icm42688_read_bit(handle, ICM42688_REG_DEVICE_CONFIG, 0)) {
            break;
        }
    }
}
void Icm42688_ResetSignalPath(struct Icm42688_Handle* handle) {
    icm42688_write_bit(handle, ICM42688_REG_SIGNAL_PATH_RESET, 3, 1);
}
void Icm42688_FlushFifo(struct Icm42688_Handle* handle) {
    icm42688_write_bit(handle, ICM42688_REG_SIGNAL_PATH_RESET, 1, 1);
}

uint8_t Icm42688_GetStatus(struct Icm42688_Handle* handle) {
    return icm42688_read_register(handle, ICM42688_REG_INT_STATUS);
}
void Icm42688_SetBank(struct Icm42688_Handle* handle, enum Icm42688_Bank bank) {
    icm42688_write_register(handle, ICM42688_REG_BANK_SEL, bank);
}

void Icm42688_SetInt(struct Icm42688_Handle* handle, enum Icm42688_Int interrupt, enum Icm42688_IntMode mode, enum Icm42688_IntDrive drive, enum Icm42688_IntPolarity polarity) {
    uint8_t val = icm42688_read_register(handle, ICM42688_REG_INT_CONFIG);
    val = (val & ~(0x07 << interrupt)) | (((mode << 2) | (drive << 1) | polarity) << interrupt);
    icm42688_write_register(handle, ICM42688_REG_INT_CONFIG, val);
}
void Icm42688_SetFifo(struct Icm42688_Handle* handle, enum Icm42688_FifoMode mode) {
    icm42688_write_register(handle, ICM42688_REG_FIFO_CONFIG, mode << 6);
}
void Icm42688_SetFifoHoldData(struct Icm42688_Handle* handle, uint8_t value) {
    icm42688_write_bit(handle, ICM42688_REG_INTF_CONFIG0, 7, value);
}
void Icm42688_SetFifoCountMode(struct Icm42688_Handle* handle, enum Icm42688_FifoCountMode mode) {
    icm42688_write_bit(handle, ICM42688_REG_INTF_CONFIG0, 6, mode);
}
void Icm42688_SetEndian(struct Icm42688_Handle* handle, enum Icm42688_Endian endian) {
    icm42688_write_bit(handle, ICM42688_REG_INTF_CONFIG0, 5, endian);
    icm42688_write_bit(handle, ICM42688_REG_INTF_CONFIG0, 4, endian);
}

void Icm42688_SetGyroMode(struct Icm42688_Handle* handle, enum Icm42688_GyroMode mode) {
    icm42688_write_bits(handle, ICM42688_REG_PWR_MGMT0, 2, mode, 2);
}
void Icm42688_SetAccelMode(struct Icm42688_Handle* handle, enum Icm42688_AccelMode mode) {
    icm42688_write_bits(handle, ICM42688_REG_PWR_MGMT0, 0, mode, 2);
}
void Icm42688_SetGyroFsr(struct Icm42688_Handle* handle, enum Icm42688_GyroFsr fsr) {
    icm42688_write_bits(handle, ICM42688_REG_GYRO_CONFIG0, 5, fsr, 3);
}
void Icm42688_SetGyroOdr(struct Icm42688_Handle* handle, enum Icm42688_Odr odr) {
    icm42688_write_bits(handle, ICM42688_REG_GYRO_CONFIG0, 0, odr, 4);
}
void Icm42688_SetAccelFsr(struct Icm42688_Handle* handle, enum Icm42688_AccelFsr fsr) {
    icm42688_write_bits(handle, ICM42688_REG_ACCEL_CONFIG0, 5, fsr, 3);
}
void Icm42688_SetAccelOdr(struct Icm42688_Handle* handle, enum Icm42688_Odr odr) {
    icm42688_write_bits(handle, ICM42688_REG_ACCEL_CONFIG0, 0, odr, 4);
}

void Icm42688_SetTempFilt(struct Icm42688_Handle* handle, enum Icm42688_TempFilt filt) {
    icm42688_write_bits(handle, ICM42688_REG_GYRO_CONFIG1, 5, filt, 3);
}

void Icm42688_SetGyroFiltOrder(struct Icm42688_Handle* handle, enum Icm42688_FiltOrder filt) {
    icm42688_write_bits(handle, ICM42688_REG_GYRO_CONFIG1, 2, filt, 2);
}
void Icm42688_SetAccelFilt(struct Icm42688_Handle* handle, enum Icm42688_Filt filt) {
    icm42688_write_bits(handle, ICM42688_REG_GYRO_ACCEL_CONFIG0, 4, filt, 4);
}
void Icm42688_SetGyroFilt(struct Icm42688_Handle* handle, enum Icm42688_Filt filt) {
    icm42688_write_bits(handle, ICM42688_REG_GYRO_ACCEL_CONFIG0, 0, filt, 4);
}
void Icm42688_SetAccelFiltOrder(struct Icm42688_Handle* handle, enum Icm42688_FiltOrder filt) {
    icm42688_write_bits(handle, ICM42688_REG_ACCEL_CONFIG1, 3, filt, 2);
}

void Icm42688_SetTimeRes(struct Icm42688_Handle* handle, enum Icm42688_TimeRes res) {
    icm42688_write_bit(handle, ICM42688_REG_TMST_CONFIG, 3, res);
}
void Icm42688_SetTimeDelta(struct Icm42688_Handle* handle, uint8_t state) {
    icm42688_write_bit(handle, ICM42688_REG_TMST_CONFIG, 2, state);
}
void Icm42688_SetTimeFsync(struct Icm42688_Handle* handle, uint8_t state) {
    icm42688_write_bit(handle, ICM42688_REG_TMST_CONFIG, 1, state);
}
void Icm42688_SetFifoHires(struct Icm42688_Handle* handle, uint8_t state) {
    icm42688_write_bit(handle, ICM42688_REG_FIFO_CONFIG1, 4, state);
}
void Icm42688_SetFifoTemp(struct Icm42688_Handle* handle, uint8_t state) {
    icm42688_write_bit(handle, ICM42688_REG_FIFO_CONFIG1, 2, state);
}
void Icm42688_SetFifoGyro(struct Icm42688_Handle* handle, uint8_t state) {
    icm42688_write_bit(handle, ICM42688_REG_FIFO_CONFIG1, 1, state);
}
void Icm42688_SetFifoAccel(struct Icm42688_Handle* handle, uint8_t state) {
    icm42688_write_bit(handle, ICM42688_REG_FIFO_CONFIG1, 0, state);
}
void Icm42688_SetFifoIntThresh(struct Icm42688_Handle* handle, uint16_t thresh) {
    icm42688_write_registers(handle, ICM42688_REG_FIFO_CONFIG2, (uint8_t*)&thresh, 2);
}

void Icm42688_SetIntDuration(struct Icm42688_Handle* handle, enum Icm42688_IntDuration duration) {
    icm42688_write_bit(handle, ICM42688_REG_INT_CONFIG1, 6, duration);
}
void Icm42688_SetIntDeassert(struct Icm42688_Handle* handle, uint8_t state) {
    icm42688_write_bit(handle, ICM42688_REG_INT_CONFIG1, 5, !state);
}
void Icm42688_SetIntAsync(struct Icm42688_Handle* handle, uint8_t state) {
    icm42688_write_bit(handle, ICM42688_REG_INT_CONFIG1, 4, state);
}

void Icm42688_SetIntSource(struct Icm42688_Handle* handle, enum Icm42688_Int interrupt, enum Icm42688_IntSource source) {
    icm42688_write_register(handle, ICM42688_REG_INT_SOURCE0 + (interrupt & 0x02), source);
}
