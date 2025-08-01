#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_

#include "stdint.h"
#include "main.h"

#define ICM42688_ID 0x47 // 0b01000111

//bank 0
#define ICM42688_REG_DEVICE_CONFIG 0x11 //r-w

#define ICM42688_REG_DRIVE_CONFIG 0x13 //r-w
#define ICM42688_REG_INT_CONFIG 0x14

#define ICM42688_REG_FIFO_CONFIG 0x16 //r-w

#define ICM42688_REG_TEMP_DATA1 0x1D //sync
#define ICM42688_REG_TEMP_DATA0 0x1E //sync

#define ICM42688_REG_ACCEL_DATA_X1 0x1F //sync
#define ICM42688_REG_ACCEL_DATA_X0 0x20 //sync
#define ICM42688_REG_ACCEL_DATA_Y1 0x21 //sync
#define ICM42688_REG_ACCEL_DATA_Y0 0x22 //sync
#define ICM42688_REG_ACCEL_DATA_Z1 0x23 //sync
#define ICM42688_REG_ACCEL_DATA_Z0 0x24 //sync

#define ICM42688_REG_GYRO_DATA_X1 0x25 //sync
#define ICM42688_REG_GYRO_DATA_X0 0x26 //sync
#define ICM42688_REG_GYRO_DATA_Y1 0x27 //sync
#define ICM42688_REG_GYRO_DATA_Y0 0x28 //sync
#define ICM42688_REG_GYRO_DATA_Z1 0x29 //sync
#define ICM42688_REG_GYRO_DATA_Z0 0x2A //sync

#define ICM42688_REG_TMST_FSYNCH 0x2B //sync
#define ICM42688_REG_TMST_FSYNCL 0x2C //sync

#define ICM42688_REG_INT_STATUS 0x2D //r-clr

#define ICM42688_REG_FIFO_COUNTH 0x2E //r
#define ICM42688_REG_FIFO_COUNTL 0x2F //r

#define ICM42688_REG_FIFO_DATA 0x30 //r

#define ICM42688_REG_APEX_DATA0 0x31 //sync
#define ICM42688_REG_APEX_DATA1 0x32 //sync
#define ICM42688_REG_APEX_DATA2 0x33 //r
#define ICM42688_REG_APEX_DATA3 0x34 //r
#define ICM42688_REG_APEX_DATA4 0x35 //r
#define ICM42688_REG_APEX_DATA5 0x36 //r

#define ICM42688_REG_INT_STATUS2 0x37 //r-clr
#define ICM42688_REG_INT_STATUS3 0x38 //r-clr

#define ICM42688_REG_SIGNAL_PATH_RESET 0x4B //w-clr

#define ICM42688_REG_INTF_CONFIG0 0x4C //r-w
#define ICM42688_REG_INTF_CONFIG1 0x4D //r-w

#define ICM42688_REG_PWR_MGMT0 0x4E //r-w

#define ICM42688_REG_GYRO_CONFIG0 0x4F //r-w
#define ICM42688_REG_GYRO_CONFIG1 0x51 //r-w

#define ICM42688_REG_ACCEL_CONFIG0 0x50 //r-w
#define ICM42688_REG_ACCEL_CONFIG1 0x53 //r-w

#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52 //r-w

#define ICM42688_REG_TMST_CONFIG 0x54 //r-w

#define ICM42688_REG_APEX_CONFIG0 0x56 //r-w

#define ICM42688_REG_SMD_CONFIG 0x57 //r-w

#define ICM42688_REG_FIFO_CONFIG1 0x5F //r-w
#define ICM42688_REG_FIFO_CONFIG2 0x60 //r-w
#define ICM42688_REG_FIFO_CONFIG3 0x61 //r-w

#define ICM42688_REG_FSYNC_CONFIG 0x62 //r-w

#define ICM42688_REG_INT_CONFIG0 0x63 //r-w
#define ICM42688_REG_INT_CONFIG1 0x64 //r-w

#define ICM42688_REG_INT_SOURCE0 0x65 //r-w
#define ICM42688_REG_INT_SOURCE1 0x66 //r-w
#define ICM42688_REG_INT_SOURCE3 0x67 //r-w
#define ICM42688_REG_INT_SOURCE4 0x68 //r-w

#define ICM42688_REG_FIFO_LOST_PKT0 0x6C //r
#define ICM42688_REG_FIFO_LOST_PKT1 0x6D //r

#define ICM42688_REG_SELF_TEST_CONFIG 0x70 //r-w

#define ICM42688_REG_WHO_AM_I 0x75 //r

#define ICM42688_REG_BANK_SEL 0x76 //r-w

//bank 1
#define ICM42688_REG_SENSOR_CONFIG0 0x03 //r-w

#define ICM42688_REG_GYRO_CONFIG_STATIC2  0x0B //r-w
#define ICM42688_REG_GYRO_CONFIG_STATIC3  0x0C //r-w
#define ICM42688_REG_GYRO_CONFIG_STATIC4  0x0D //r-w
#define ICM42688_REG_GYRO_CONFIG_STATIC5  0x0E //r-w
#define ICM42688_REG_GYRO_CONFIG_STATIC6  0x0F //r-w
#define ICM42688_REG_GYRO_CONFIG_STATIC7  0x10 //r-w
#define ICM42688_REG_GYRO_CONFIG_STATIC8  0x11 //r-w
#define ICM42688_REG_GYRO_CONFIG_STATIC9  0x12 //r-w
#define ICM42688_REG_GYRO_CONFIG_STATIC10 0x13 //r-w

#define ICM42688_REG_XG_ST_DATA 0x5F //r-w
#define ICM42688_REG_YG_ST_DATA 0x60 //r-w
#define ICM42688_REG_ZG_ST_DATA 0x61 //r-w

#define ICM42688_REG_TMSTVAL0 0x62 //r
#define ICM42688_REG_TMSTVAL1 0x63 //r
#define ICM42688_REG_TMSTVAL2 0x64 //r

#define ICM42688_REG_INTF_CONFIG4 0x7A //r-w
#define ICM42688_REG_INTF_CONFIG5 0x7B //r-w
#define ICM42688_REG_INTF_CONFIG6 0x7C //r-w

//bank 2
#define ICM42688_REG_ACCEL_CONFIG_STATIC2 0x03 //r-w
#define ICM42688_REG_ACCEL_CONFIG_STATIC3 0x04 //r-w
#define ICM42688_REG_ACCEL_CONFIG_STATIC4 0x05 //r-w

#define ICM42688_REG_XA_ST_DATA 0x3B //r-w
#define ICM42688_REG_YA_ST_DATA 0x3C //r-w
#define ICM42688_REG_ZA_ST_DATA 0x3D //r-w

//bank 4
#define ICM42688_REG_APEX_CONFIG1 0x40 //r-w
#define ICM42688_REG_APEX_CONFIG2 0x41 //r-w
#define ICM42688_REG_APEX_CONFIG3 0x42 //r-w
#define ICM42688_REG_APEX_CONFIG4 0x43 //r-w
#define ICM42688_REG_APEX_CONFIG5 0x44 //r-w
#define ICM42688_REG_APEX_CONFIG6 0x45 //r-w
#define ICM42688_REG_APEX_CONFIG7 0x46 //r-w
#define ICM42688_REG_APEX_CONFIG8 0x47 //r-w
#define ICM42688_REG_APEX_CONFIG9 0x48 //r-w

#define ICM42688_REG_ACCEL_WOM_X_THR 0x4A //r-w
#define ICM42688_REG_ACCEL_WOM_Y_THR 0x4B //r-w
#define ICM42688_REG_ACCEL_WOM_Z_THR 0x4C //r-w

#define ICM42688_REG_INT_SOURCE6  0x4D //r-w
#define ICM42688_REG_INT_SOURCE7  0x4E //r-w
#define ICM42688_REG_INT_SOURCE8  0x4F //r-w
#define ICM42688_REG_INT_SOURCE9  0x50 //r-w
#define ICM42688_REG_INT_SOURCE10 0x51 //r-w

#define ICM42688_REG_OFFSET_USER0 0x77 //r-w
#define ICM42688_REG_OFFSET_USER1 0x78 //r-w
#define ICM42688_REG_OFFSET_USER2 0x79 //r-w
#define ICM42688_REG_OFFSET_USER3 0x7A //r-w
#define ICM42688_REG_OFFSET_USER4 0x7B //r-w
#define ICM42688_REG_OFFSET_USER5 0x7C //r-w
#define ICM42688_REG_OFFSET_USER6 0x7D //r-w
#define ICM42688_REG_OFFSET_USER7 0x7E //r-w
#define ICM42688_REG_OFFSET_USER8 0x7F //r-w

enum Icm42688_Bank {
    Icm42688_Bank0 = 0u,
    Icm42688_Bank1 = 1u,
    Icm42688_Bank2 = 2u,
    Icm42688_Bank3 = 3u,
    Icm42688_Bank4 = 4u,
};

enum Icm42688_Int {
    Icm42688_Int1 = 0u,
    Icm42688_Int2 = 3u,
};
enum Icm42688_IntMode {
    Icm42688_IntModePulse = 0u,
    Icm42688_IntModeLatch = 1u,
};
enum Icm42688_IntDrive {
    Icm42688_IntDriveOpen = 0u,
    Icm42688_IntDrivePush = 1u,
};
enum Icm42688_IntPolarity {
    Icm42688_IntPolarityLow = 0u,
    Icm42688_IntPolarityHigh = 1u,
};

enum Icm42688_FifoMode {
    Icm42688_FifoModeBypass = 0u,
    Icm42688_FifoModeStream = 1u,
    Icm42688_FifoModeFill = 2u,
};

enum Icm42688_FifoCountMode {
    Icm42688_FifoCountModeByte = 0u,
    Icm42688_FifoCountModeRecord = 1u,
};

enum Icm42688_Endian {
    Icm42688_EndianLittle = 0u,
    Icm42688_EndianBig = 1u,
};


enum Icm42688_Odr {
    Icm42688_Odr32k = 1u,
    Icm42688_Odr16k = 2u,
    Icm42688_Odr8k = 3u,
    Icm42688_Odr4k = 4u,
    Icm42688_Odr2k = 5u,
    Icm42688_Odr1k = 6u,
    Icm42688_Odr500 = 15u,
    Icm42688_Odr200 = 7u,
    Icm42688_Odr100 = 8u,
    Icm42688_Odr50 = 9u,
    Icm42688_Odr25 = 10u,
    Icm42688_Odr12 = 11u, // actually 12.5

    // not available for gyroscope
    Icm42688_Odr6 = 12u, // actually 6.25
    Icm42688_Odr3 = 13u, // actually 3.125
    Icm42688_Odr1 = 14u, // actually 1.5625
};

// in degrees per second
enum Icm42688_GyroFsr {
    Icm42688_GyroFsr2000 = 0u,
    Icm42688_GyroFsr1000 = 1u,
    Icm42688_GyroFsr500 = 2u,
    Icm42688_GyroFsr250 = 3u,
    Icm42688_GyroFsr125 = 4u,
    Icm42688_GyroFsr62 = 5u, // actually 62.5
    Icm42688_GyroFsr31 = 6u, // actually 31.25
    Icm42688_GyroFsr15 = 7u, // actually 15.625
};

// in g
enum Icm42688_AccelFsr {
    Icm42688_AccelFsr16 = 0u,
    Icm42688_AccelFsr8 = 1u,
    Icm42688_AccelFsr4 = 2u,
    Icm42688_AccelFsr2 = 3u,
};

enum Icm42688_GyroMode {
    Icm42688_GyroModeOff = 0u,
    Icm42688_GyroModeStandby = 1u,
    Icm42688_GyroModeLowNoise = 3u,
};

enum Icm42688_AccelMode {
    Icm42688_AccelModeOff = 0u,
    Icm42688_AccelModeLowPower = 2u,
    Icm42688_AccelModeLowNoise = 3u,
};

// in bandwidth Hz
enum Icm42688_TempFilt {
    Icm42688_TempFilt4k = 0u, // 0.125ms latency
    Icm42688_TempFilt170 = 1u, // 1ms latency
    Icm42688_TempFilt82 = 2u, // 2ms latency
    Icm42688_TempFilt40 = 3u, // 4ms latency
    Icm42688_TempFilt20 = 4u, // 8ms latency
    Icm42688_TempFilt10 = 5u, // 16ms latency
    Icm42688_TempFilt5 = 6u, // 32ms latency
};

enum Icm42688_FiltOrder {
    Icm42688_FiltOrder1 = 0u,
    Icm42688_FiltOrder2 = 1u,
    Icm42688_FiltOrder3 = 2u,
};

// in ODR division
enum Icm42688_Filt {
    Icm42688_Filt2 = 0u,
    Icm42688_Filt4 = 1u,
    Icm42688_Filt5 = 2u,
    Icm42688_Filt8 = 3u,
    Icm42688_Filt10 = 4u,
    Icm42688_Filt16 = 5u,
    Icm42688_Filt20 = 6u,
    Icm42688_Filt40 = 7u,

    // low latency options
    Icm42688_FiltDec = 14u, // basic decimation
    Icm42688_FiltDec8 = 15u, // basic decimation/8
};

// in microseconds
enum Icm42688_TimeRes {
    Icm42688_TimeRes1 = 0u,
    Icm42688_TimeRes16 = 1u,
    Icm42688_TimeResRtc = 1u,
};

// in microseconds
enum Icm42688_IntDuration {
    Icm42688_IntDuration100 = 0u,
    Icm42688_IntDuration8 = 1u,
};

// interrupt mask
enum Icm42688_IntSource {
    Icm42688_IntSourceNone = 0x00u,
    Icm42688_IntSourceFsync = 0x40u,
    Icm42688_IntSourcePll = 0x20u,
    Icm42688_IntSourceReset = 0x10u,
    Icm42688_IntSourceDrdy = 0x08u,
    Icm42688_IntSourceThresh = 0x04u,
    Icm42688_IntSourceFull = 0x02u,
    Icm42688_IntSourceAgc = 0x01u,
};

struct Icm42688_Handle {
    // configuration
    SPI_HandleTypeDef* hspi; // 24Mhz max
    GPIO_TypeDef* csPort;
    uint16_t csPin;

    uint16_t intPin;

    // internal
    uint8_t init;

    uint8_t* data;
};

void Icm42688_Init(struct Icm42688_Handle* handle);

uint8_t Icm42688_ExtFlag(struct Icm42688_Handle* handle, uint16_t pin);
void Icm42688_ExtHandler(struct Icm42688_Handle* handle);

void Icm42688_Reset(struct Icm42688_Handle* handle);
void Icm42688_ResetSignalPath(struct Icm42688_Handle* handle);
void Icm42688_FlushFifo(struct Icm42688_Handle* handle);

uint8_t Icm42688_GetStatus(struct Icm42688_Handle* handle);

void Icm42688_SetBank(struct Icm42688_Handle* handle, enum Icm42688_Bank bank);

void Icm42688_SetInt(struct Icm42688_Handle* handle, enum Icm42688_Int interrupt,
        enum Icm42688_IntMode mode, enum Icm42688_IntDrive drive, enum Icm42688_IntPolarity polarity);
void Icm42688_SetFifo(struct Icm42688_Handle* handle, enum Icm42688_FifoMode mode);
void Icm42688_SetFifoHoldData(struct Icm42688_Handle* handle, uint8_t value);
void Icm42688_SetFifoCountMode(struct Icm42688_Handle* handle, enum Icm42688_FifoCountMode mode);
void Icm42688_SetEndian(struct Icm42688_Handle* handle, enum Icm42688_Endian endian);
void Icm42688_SetGyroMode(struct Icm42688_Handle* handle, enum Icm42688_GyroMode mode);
void Icm42688_SetAccelMode(struct Icm42688_Handle* handle, enum Icm42688_AccelMode mode);
void Icm42688_SetGyroFsr(struct Icm42688_Handle* handle, enum Icm42688_GyroFsr fsr);
void Icm42688_SetGyroOdr(struct Icm42688_Handle* handle, enum Icm42688_Odr odr);
void Icm42688_SetAccelFsr(struct Icm42688_Handle* handle, enum Icm42688_AccelFsr fsr);
void Icm42688_SetAccelOdr(struct Icm42688_Handle* handle, enum Icm42688_Odr odr);
void Icm42688_SetTempFilt(struct Icm42688_Handle* handle, enum Icm42688_TempFilt filt);
void Icm42688_SetGyroFiltOrder(struct Icm42688_Handle* handle, enum Icm42688_FiltOrder filt);
void Icm42688_SetAccelFilt(struct Icm42688_Handle* handle, enum Icm42688_Filt filt);
void Icm42688_SetGyroFilt(struct Icm42688_Handle* handle, enum Icm42688_Filt filt);
void Icm42688_SetAccelFiltOrder(struct Icm42688_Handle* handle, enum Icm42688_FiltOrder filt);
void Icm42688_SetTimeRes(struct Icm42688_Handle* handle, enum Icm42688_TimeRes res);
void Icm42688_SetTimeDelta(struct Icm42688_Handle* handle, uint8_t state);
void Icm42688_SetTimeFsync(struct Icm42688_Handle* handle, uint8_t state);
void Icm42688_SetFifoHires(struct Icm42688_Handle* handle, uint8_t state);
void Icm42688_SetFifoTemp(struct Icm42688_Handle* handle, uint8_t state);
void Icm42688_SetFifoGyro(struct Icm42688_Handle* handle, uint8_t state);
void Icm42688_SetFifoAccel(struct Icm42688_Handle* handle, uint8_t state);
void Icm42688_SetFifoIntThresh(struct Icm42688_Handle* handle, uint16_t thresh);
void Icm42688_SetIntDuration(struct Icm42688_Handle* handle, enum Icm42688_IntDuration duration);
void Icm42688_SetIntDeassert(struct Icm42688_Handle* handle, uint8_t state);
void Icm42688_SetIntAsync(struct Icm42688_Handle* handle, uint8_t state);
void Icm42688_SetIntSource(struct Icm42688_Handle* handle, enum Icm42688_Int interrupt, enum Icm42688_IntSource source);

#endif
