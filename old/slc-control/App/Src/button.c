#include "button.h"
#include "main.h"

extern TIM_HandleTypeDef BUTTON_HANDLE;

#define T(btn) btn##_GPIO_Port,
static GPIO_TypeDef *ports[BUTTON_COUNT] = { BUTTON_LIST };
#undef T

#define T(btn) btn##_Pin,
static uint16_t pins[BUTTON_COUNT] = { BUTTON_LIST };
#undef T

static uint8_t lastState[BUTTON_COUNT];
static uint8_t pressed[BUTTON_COUNT];

void Button_Init(void) {
	HAL_TIM_Base_Start_IT(&BUTTON_HANDLE);
}

uint8_t Button_GetPressed(enum ButtonId button) {
	const uint8_t val = pressed[button];
	pressed[button] = 0;
	return val;
}

void Button_TimHandler(void) {
	for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
		uint8_t state = !HAL_GPIO_ReadPin(ports[i], pins[i]);
		if (state && !lastState[i]) {
			pressed[i] = 1;
		}
		lastState[i] = state;
	}
}

uint8_t Button_TimFlag(TIM_HandleTypeDef *htim) {
	return htim == &BUTTON_HANDLE;
}
