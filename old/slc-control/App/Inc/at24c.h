#ifndef INC_AT24C_H_
#define INC_AT24C_H_

#include "stdint.h"

/* BEGIN CONFIG */
#define AT24C_HANDLE hi2c1
#define AT24C_ADDRESS 0xA0

#define AT24C_PAGE_SIZE  64
#define AT24C_PAGE_COUNT 512
/* END CONFIG */

//wrap page
void At24c_Write(uint16_t address, uint8_t *buf, uint16_t len);

//wrap memory
void At24c_ReadNext(uint8_t *buf, uint8_t len);

void At24c_Read(uint16_t address, uint8_t *buf, uint16_t len);

#endif
