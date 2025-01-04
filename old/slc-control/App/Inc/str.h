#ifndef INC_STR_H_
#define INC_STR_H_

#include "stdint.h"

//returns if string is longer than len in length
uint8_t Str_Longer(char* str, uint8_t len);

//returns if strings match to len characters
uint8_t Str_Comp(char* str1, char* str2, const uint8_t len);

//returns the char* at nth found delim
char* Str_Count(char* str, char delim, uint8_t len);

//returns the char* at found character
char* Str_To(char* str, char delim);

uint8_t Str_ParseByte(char* str);
float Str_ParseFloat(char* str);

#endif
