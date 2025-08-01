#ifndef INC_STR_H_
#define INC_STR_H_

#include "stdint.h"

//returns if strings match to len characters
uint8_t Str_Equal(char* str1, char* str2, const uint8_t len);

//returns the char* at found character
//or end if not found
char* Str_To(char* msg, char* end, char delim);

uint8_t Str_ParseByte(char* msg, char* end);
float Str_ParseFloat(char* msg, char* end);

uint8_t Str_PrintFloat(char* buf, float val, uint8_t decimals);

#endif
