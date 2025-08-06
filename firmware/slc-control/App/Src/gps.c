#include "gps.h"
#include "usbd_cdc_if.h"
#include "stdlib.h"
#include "string.h"
#include "nmea.h"

static void gps_start_dma(struct Gps_Handle* handle) {
    HAL_UARTEx_ReceiveToIdle_DMA(handle->huart, handle->rxBuf, handle->rxBufSize);
    __HAL_DMA_DISABLE_IT(handle->hdma, DMA_IT_HT);
}

void Gps_Init(struct Gps_Handle* handle) {
    handle->rxBuf = malloc(handle->rxBufSize);
    handle->readBuf = malloc(handle->rxBufSize);

    handle->txBuf = malloc(handle->txBufSize);

    gps_start_dma(handle);
    //handle->available = 1;
    handle->init = 0;
}

uint8_t Gps_UartFlag(struct Gps_Handle* handle, UART_HandleTypeDef* huart) {
    return handle->huart == huart;
}

void Gps_UartHandler(struct Gps_Handle* handle, uint16_t len) {
    //if (handle->available) {
    //    handle->available = 0;
        memcpy(handle->readBuf, handle->rxBuf, len);
        gps_start_dma(handle);
        //handle->available = 1;
    //}
}

void Gps_Transmit(struct Gps_Handle* handle, uint8_t* data, uint16_t len) {
    memcpy(handle->txBuf, data, len);
    HAL_UART_Transmit_DMA(handle->huart, handle->txBuf, len);
}
