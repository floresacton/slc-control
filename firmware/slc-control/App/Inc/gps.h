#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stdint.h"
#include "nmea.h"
#include "main.h"

struct Gps_Handle {
    // configuration
    UART_HandleTypeDef* huart;
    DMA_HandleTypeDef* hdma;

    uint16_t rxBufSize; // 1000 typical
    uint16_t txBufSize; // 1000 typical

    // internal
    uint8_t* rxBuf;
    uint8_t* txBuf;

    uint8_t* readBuf;
};

void Gps_Init(struct Gps_Handle* handle);

uint8_t Gps_UartFlag(struct Gps_Handle* handle, UART_HandleTypeDef* huart);
void Gps_UartHandler(struct Gps_Handle* handle, uint16_t len);

void Gps_Transmit(struct Gps_Handle* handle, uint8_t* data, uint16_t len);

#endif
