#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "stdint.h"
#include "main.h"

/* BEGIN CONFIG */
#define BUTTON_HANDLE htim3 //50hz

#define BUTTON_COUNT 4

#define BUTTON_LIST T(BTN1) T(BTN2) T(BTN3) T(BTN4)

enum ButtonId {
	ButtonUp,
	ButtonDown,
	ButtonEnter,
	ButtonBack,
};
/* END CONFIG */

void Button_Init(void);

uint8_t Button_GetPressed(enum ButtonId button);

void Button_TimHandler(void);
uint8_t Button_TimFlag(TIM_HandleTypeDef *htim);

#endif
