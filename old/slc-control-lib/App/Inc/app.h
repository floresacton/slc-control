#ifndef INC_APP_H_
#define INC_APP_H_

#include "stdint.h"

void App_Init(void);
void App_Update(void);

void App_UsbHandler(uint8_t* data, uint32_t len);

#endif
