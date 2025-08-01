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

char* Str_To(char* msg, char* end, char delim) {
    for (; msg < end; msg++) {
        if (*msg == delim) {
            return msg;
        }
    }
    return msg;
}

uint8_t Str_ParseByte(char* msg, char* end) {
    uint8_t val = 0;
    for (; msg < end; msg++) {
        val *= 10;
        val += *msg - '0';
    }
    return val;
}

float Str_ParseFloat(char* msg, char* end) {
    uint8_t neg = 0;
    if (*msg == '-') {
        neg = 1;
        msg++;
    }
    char* decimal = Str_To(msg, end, '.');
    float val = 0;
    float mult = 0.1;
    for (; msg < end; msg++) {
        if (msg < decimal) {
            val *= 10;
            val += *msg-'0';
        }else if (msg > decimal){
            val += mult * (*msg-'0');
            mult /= 10;
        }
    }
    if (neg) {
        val = -val;
    }
    return val;
}

uint8_t Str_PrintFloat(char* buf, float val, uint8_t decimals) {
    const uint8_t neg = val < 0;
    if (neg) {
        buf[0] = '-';
        val = -val;
    }

    for (uint8_t i = 0; i < decimals; i++) {
        val *= 10;
    }

    uint32_t collect = (uint32_t)val;
    char tbuf[10];
    uint8_t i = 0;
    while ((i <= decimals) || collect) {
        tbuf[i] = '0' + (collect % 10);
        collect /= 10;
        i++;
    }

    const uint8_t iter = i + (decimals > 0);
    const uint8_t end = iter + neg;
    uint8_t j = 0;
    for (i = 0; i < iter; i++) {
        buf[end-1-i] = tbuf[j];
        
        if (i+1 == decimals) {
            i++;
            buf[end-i-1] = '.';
        }
        j++;
    }

    buf[end] = 0;
    return end;
}
