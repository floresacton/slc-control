#ifndef INC_TACH_H_
#define INC_TACH_H_

#include "stdint.h"
#include "main.h"

/* BEGIN CONFIG */
#define TACH_COUNT 3

#define TACH_REF_HANDLE htim1 //50kHz
#define TACH_REF_FREQ 50000

#define TACH_CALC_HANDLE htim2 //50Hz
#define TACH_CALC_FREQ 50

enum TachId {
	Tach1,
	Tach2,
	Tach3,
};
/* END CONFIG */

void Tach_Init(void);

uint8_t Tach_TimFlagCalc(TIM_HandleTypeDef* htim);
void Tach_TimHandlerCalc(void);

uint8_t Tach_TimFlagRef(TIM_HandleTypeDef* htim);
void Tach_TimHandlerRef(void);

void Tach_Trigger(enum TachId tach);

uint16_t Tach_GetRpm(enum TachId chan);

#endif
