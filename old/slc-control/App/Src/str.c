#include "str.h"
#include "stdint.h"

uint8_t Str_Longer(char* str, uint8_t len) {
	while (*str) {
		if (!len) {
			return 1;
		}
		len--;
		str++;
	}
	return 0;
}

uint8_t Str_Comp(char* str1, char* str2, const uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		if (*str1 != *str2) {
			return 0;
		}
		str1++;
		str2++;
	}
	return 1;
}

char* Str_Count(char* str, char delim, uint8_t len) {
    if (!len) {
        return str;
    }
	while (*str) {
		if (*str == delim) {
			len--;
			if (!len) {
				return str;
			}
		}
		str++;
	}
	return str;
}

char* Str_To(char* str, char delim) {
	while (*str && *str != delim) {
		str++;
	}
	return str;
}

uint8_t Str_ParseByte(char* str) {
	uint8_t val = 0;
	while (*str) {
		val *= 10;
		val += *str - '0';
		str++;
	}
	return val;
}

float Str_ParseFloat(char* str) {
	uint8_t neg = 0;
	if (*str == '-') {
		neg = 1;
		str++;
	}
	const char* decimal = Str_To(str, '.');
	float val = 0;
	float mult = 0.1;
	while (*str) {
		if (str < decimal) {
			val *= 10;
			val += *str-'0';
		}else if (str > decimal){
			val += mult*(*str-'0');
			mult /= 10;
		}
		str++;
	}
	if (neg) {
		val = -val;
	}
	return val;
}
