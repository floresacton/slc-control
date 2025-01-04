#include "gps.h"
#include "memory.h"
#include "nmea.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "display.h"
#include "usbd_cdc_if.h"

extern UART_HandleTypeDef GPS_HANDLE;
extern DMA_HandleTypeDef GPS_DMA_HANDLE;

static uint8_t gps_usb_buf[GPS_RX_BUF_SIZE] = {0};

static uint8_t gps_read_buf[GPS_RX_BUF_SIZE] = {0};

static uint8_t gps_rx_buf[GPS_RX_BUF_SIZE] = {0};
static uint8_t gps_tx_buf[GPS_TX_BUF_SIZE] = {0};

static uint8_t usb_receive = 0;
static uint8_t usb_transmit = 0;

void Gps_Init(void) {
	HAL_UARTEx_ReceiveToIdle_DMA(&GPS_HANDLE, gps_rx_buf, GPS_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&GPS_DMA_HANDLE, DMA_IT_HT);
}

uint8_t Gps_UartFlag(UART_HandleTypeDef* huart) {
	return huart == &GPS_HANDLE;
}

void Gps_UartHandler(uint16_t size) {
	memcpy(gps_read_buf, gps_rx_buf, GPS_RX_BUF_SIZE);
	Nmea_Parse((char*)gps_read_buf);

	if (usb_transmit) {
		memcpy(gps_usb_buf, gps_rx_buf, GPS_RX_BUF_SIZE);
		CDC_Transmit_FS(gps_usb_buf, size);
	}

	memset(gps_rx_buf, 0, GPS_RX_BUF_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&GPS_HANDLE, gps_rx_buf, GPS_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&GPS_DMA_HANDLE, DMA_IT_HT);
}

void Gps_UsbHandler(uint8_t* data, uint32_t len) {
	if (usb_receive) {
		memcpy(gps_tx_buf, data, len);
		HAL_UART_Transmit_DMA(&GPS_HANDLE, gps_tx_buf, len);
	}
}

uint8_t Gps_ExtFlag(uint16_t pin) {
	return pin == GPS_INT_PIN;
}

void Gps_ExtHandler(void) {
	Nmea_TpHandler();
}

void Gps_UsbTransmit(uint8_t mode) {
	usb_transmit = mode;
}

void Gps_UsbReceive(uint8_t mode) {
	usb_receive = mode;
}
