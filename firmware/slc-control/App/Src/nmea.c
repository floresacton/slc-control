#include "nmea.h"
#include "stdint.h"
#include "stdlib.h"
#include "str.h"

static char* messages[NMEA_MESSAGE_COUNT] = {"GGA", "GLL", "GSA", "GSV", "MSS", "RMC", "VTG", "ZDA"};

static char nmea_parse_hem(char* str, uint8_t len) {
    if (!len) return '-';
    return str[0];
}

static float nmea_parse_coord(char* str, uint8_t len, uint8_t digits) {
    if (len < digits) return 0;

    const uint8_t deg = Str_ParseByte(str, digits);

    const float min = Str_ParseFloat(str+digits, len-digits);
    return min/60.0 + (float)deg;
}

static void nmea_parse_utc(struct Nmea_Handle* handle, char* str, uint8_t len) {
    if (len < 4) return;

    uint8_t hour = 0;
    for (uint8_t i = 0; i < 2; i++) {
        hour *= 10;
        hour += str[i] - '0';
    }
    uint8_t minute = 0;
    for (uint8_t i = 2; i < 4; i++) {
        minute *= 10;
        minute += str[i] - '0';
    }
    const float second = Str_ParseFloat(str+4, len-4);

    handle->hour = hour;
    handle->minute = minute;
    handle->second = second;

    // utc to central time
    if (handle->hour < 6) {
        handle->hour += 19;
    }else{
        handle->hour -= 5;
    }
}

static void nmea_parse_date(struct Nmea_Handle* handle, char* str, uint8_t len) {
    if (len < 6) return;

    uint8_t day = 0;
    for (uint8_t i = 0; i < 2; i++) {
        day *= 10;
        day += str[i] - '0';
    }
    uint8_t month = 0;
    for (uint8_t i = 2; i < 4; i++) {
        month *= 10;
        month += str[i] - '0';
    }
    uint8_t year = 0;
    for (uint8_t i = 4; i < 6; i++) {
        year *= 10;
        year += str[i] - '0';
    }
    handle->day = day;
    handle->month = month;
    handle->year = year;
}

static void nmea_parse_param(struct Nmea_Handle* handle, uint8_t type, uint8_t index, char* str, uint8_t len) {
    switch (type) {
    case Nmea_MessageGGA:
        switch (index) {
        case 6:
            handle->satCount = Str_ParseByte(str, len);
            return;
        case 8:
            handle->altitude = Str_ParseFloat(str, len);
            return;
        }
        return;
    case Nmea_MessageRMC:
        switch (index) {
        case 0:
            nmea_parse_utc(handle, str, len);
            return;
        case 1:
            handle->fix = str[0] == 'A';
            return;
        case 2:
            handle->latitude = nmea_parse_coord(str, len, 2);
            return;
        case 3:
            handle->latHem = nmea_parse_hem(str, len);
            return;
        case 4:
            handle->longitude = nmea_parse_coord(str, len, 3);
            return;
        case 5:
            handle->lonHem = nmea_parse_hem(str, len);
            return;
        case 8:
            nmea_parse_date(handle, str, len);
            return;
        }
        return;
    case Nmea_MessageVTG:
        switch (index) {
        case 6:
            handle->speed = Str_ParseFloat(str, len) * 0.621371;
            return;
        }
        return;
    default:
        return;
    }
}

static void nmea_parse_line(struct Nmea_Handle* handle, char* str, uint8_t len) {
    if (len < NMEA_MIN_LENGTH) {
        return;
    }
    
    uint8_t type = 0;
    for (; type < NMEA_MESSAGE_COUNT; type++) {
        //$GP GLL, (ski first three)
        if (Str_Equal(str+3, messages[type], 3)) {
            goto comma_parser;
        }
    }
    return;

comma_parser: ;
    
    uint8_t count = 0;

    uint8_t valid = 0;
    uint16_t begin = 0;
    uint16_t i = 6;
    for (; i < len; i++) {
        if (str[i] == ',') {
            if (valid) {
                nmea_parse_param(handle, type, count, str + begin, i - begin);
                count++;
            }
            begin = i + 1;
            valid = 1;
        }
    }
    if (valid) {
        nmea_parse_param(handle, type, count, str + begin, i - begin);
    }
}

void Nmea_Init(struct Nmea_Handle* handle) {
    /*
    uint8_t fix;
    uint8_t satCount;

    uint8_t hour;
    uint8_t minute;
    float second;

    uint8_t year;
    uint8_t month;
    uint8_t day;

    float latitude;
    char latHem;
    float longitude;
    char lonHem;
    float altitude;

    float speed;
    */

    handle->latHem = '-';
    handle->lonHem = '-';
}

void Nmea_Parse(struct Nmea_Handle* handle, char* str, uint16_t len) {
    uint8_t valid = 0;
    uint16_t begin = 0;
    uint16_t i = 0;
    for (; i < len; i++) {
        if (str[i] == '$') {
            if (valid) {
                nmea_parse_line(handle, str + begin, i - begin);
            }
            begin = i;
            valid = 1;
        }
    }
    if (valid) {
        nmea_parse_line(handle, str + begin, i - begin);
    }
}

uint8_t Nmea_ExtFlag(struct Nmea_Handle* handle, uint16_t pin) {
    return handle->intPin == pin;
}
void Nmea_ExtHandler(struct Nmea_Handle* handle) {
    handle->second += handle->timepulse;
}
