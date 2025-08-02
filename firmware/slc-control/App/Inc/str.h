#ifndef INC_STR_H_
#define INC_STR_H_

#include "stdint.h"

//returns if strings match to len characters
uint8_t Str_Equal(char* str1, char* str2, const uint8_t len);

//returns the char* at found character
//or end if not found
uint8_t Str_To(char* str, uint8_t len, char delim);

uint8_t Str_ParseByte(char* str, uint8_t len);
float Str_ParseFloat(char* str, uint8_t len);

void Str_PrintFloat(char* buf, uint8_t len, uint8_t decimals, uint8_t zeros, float val);

#endif
