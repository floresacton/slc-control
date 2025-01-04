#include "app.h"
#include "memory.h"

#include "icm42688.h"
#include "lps22hh.h"
#include "qmc5883.h"

#include "dac.h"
#include "tach.h"
#include "gps.h"

#include "display.h"
#include "oled.h"
#include "button.h"

#include "control.h"

//app N
//bitmaps ok
//control (including replicate) N
//dac N
//display ok
//eeprom ok
//fonts ok
//gps N
//input ok
//memory ok
//oled ok
//tach N
//gyro N
//magnometer ok
//pressure sensor ok
//usb ok

extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp3;

void App_Init(void) {
	Memory_Init();

	Icm42688_Init();
	Lps22hh_Init();
	Qmc5883_Init();

	Dac_Set(Dac1, Memory_ReadShort(MemTach1Thresh));
	Dac_Set(Dac2, Memory_ReadShort(MemTach2Thresh));
	Dac_Set(Dac3, Memory_ReadShort(MemTach3Thresh));

	Gps_Init();
	Tach_Init();
	HAL_COMP_Start(&hcomp1);
	HAL_COMP_Start(&hcomp2);
	HAL_COMP_Start(&hcomp3);

	Oled_Init();
	Display_Init();
	Button_Init();

	//temporary
	Gps_UsbTransmit(1);
	Gps_UsbReceive(1);
}

void App_Update(void) {
	Control_Update();
	Display_Update();
	Lps22hh_Update();

	HAL_Delay(10);
}

void App_UsbHandler(uint8_t* data, uint32_t len) {
	Gps_UsbHandler(data, len);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (Tach_TimFlagRef(htim)) {
		Tach_TimHandlerRef();
	}else if (Tach_TimFlagCalc(htim)) {
		Tach_TimHandlerCalc();
	}else if (Button_TimFlag(htim)){
		Button_TimHandler();
	}
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef* hcomp) {
	if (hcomp == &hcomp2) {
		Tach_Trigger(Tach1);
	}else if (hcomp == &hcomp1) {
		Tach_Trigger(Tach2);
	}else if (hcomp == &hcomp3) {
		Tach_Trigger(Tach3);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (Icm42688_ExtFlag(GPIO_Pin)) {
		Icm42688_ExtHandler();
	}else if (Qmc5883_ExtFlag(GPIO_Pin)) {
		Qmc5883_ExtHandler();
	}else if (Lps22hh_ExtFlag(GPIO_Pin)) {
		Lps22hh_ExtHandler();
	}else if (Gps_ExtFlag(GPIO_Pin)) {
		Gps_ExtHandler();
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	if (Gps_UartFlag(huart)) {
		Gps_UartHandler(size);
	}
}
