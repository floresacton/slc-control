#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "stdint.h"
#include "stdbool.h"
#include "main.h"

struct Button_Handle {
    // configuration
    GPIO_TypeDef* port;
    uint16_t pin;

    // internal
    bool lastState;
    bool pressed;
};

void Button_Init(struct Button_Handle* handle);

// 50Hz typical
void Button_Update(struct Button_Handle* handle);

#endif
