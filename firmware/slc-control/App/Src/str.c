#include "str.h"
#include "stdint.h"

uint8_t Str_Equal(char* str1, char* str2, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (*str1 != *str2) {
            return 0;
        }
        str1++;
        str2++;
    }
    return 1;
}

uint8_t Str_To(char* str, uint8_t len, char delim) {
    for (uint8_t i = 0; i < len; i++) {
        if (str[i] == delim) {
            return i;
        }
    }
    return len;
}

uint8_t Str_ParseByte(char* str, uint8_t len) {
    uint8_t val = 0;
    for (uint8_t i = 0; i < len; i++) {
        val *= 10;
        val += str[i] - '0';
    }
    return val;
}

float Str_ParseFloat(char* str, uint8_t len) {
    if (!len) return 0;
    uint8_t neg = 0;
    if (str[0] == '-') neg = 1;
    
    const uint8_t decimal = Str_To(str, len, '.');
    
    float val = 0;
    float mult = 1;
    for (uint8_t i = neg; i < len; i++) {
        if (i < decimal) {
            val *= 10;
            val += str[i] - '0';
        }else if (i > decimal){
            mult /= 10;
            val += mult * (str[i] - '0');
        }
    }
    if (neg) val = -val;

    return val;
}

/*
void Str_PrintInt(char* buf, uint8_t len, uint8_t zeros, int32_t val) {
    const uint8_t neg = val < 0;
    if (neg) {
        buf[0] = '-';
        val = -val;
    }

    for (uint8_t i = 0; i < len-neg; i++) {
        if (!zeros && !val && i > 0) return;
        buf[len-1-i] = '0' + (val % 10);
        val /= 10;
    }
}
*/

void Str_PrintFloat(char* buf, uint8_t len, uint8_t decimals, uint8_t zeros, float val) {
    const uint8_t neg = val < 0;
    if (neg) {
        buf[0] = '-';
        val = -val;
    }

    for (uint8_t i = 0; i < decimals; i++) {
        val *= 10;
    }

    uint32_t collect = (uint32_t)val;
    
    for (uint8_t i = 0; i < len-neg; i++) {
        if ((!zeros) && (!collect) && (i > decimals)) {
            buf[len-1-i] = ' ';
            continue;
        }
        buf[len-1-i] = '0' + (collect % 10);
        collect /= 10;
    }
    buf[len] = 0;
}
