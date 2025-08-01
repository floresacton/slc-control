#include "nmea.h"
#include "stdint.h"
#include "stdlib.h"
#include "str.h"

static char* messages[NMEA_MESSAGE_COUNT] = {"GGA", "GLL", "GSA", "GSV", "MSS", "RMC", "VTG", "ZDA"};

static char nmea_parse_hem(char* msg, char* end) {
    if (msg == end) {
        return '-';
    }
    return *msg;
}

static float nmea_parse_coord(char* msg, char* end, uint8_t digits) {
    uint8_t deg = 0;
    for (uint8_t i = 0; i < digits && (msg < end); i++) {
        deg *= 10;
        deg += *msg-'0';
        msg++;
    }
    const float min = Str_ParseFloat(msg, end);
    return min/60.0 + (float)deg;
}

static void nmea_parse_utc(struct Nmea_Handle* handle, char* msg, char* end) {
    uint8_t hour = 0;
    for (uint8_t i = 0; i < 2 && (msg < end); i++) {
        hour *= 10;
        hour += *msg-'0';
        msg++;
    }
    uint8_t minute = 0;
    for (uint8_t i = 0; i < 2 && (msg < end); i++) {
        minute *= 10;
        minute += *msg-'0';
        msg++;
    }
    const float second = Str_ParseFloat(msg, end);

    handle->hour = hour;
    handle->minute = minute;
    handle->second = second;
}

static void nmea_parse_date(struct Nmea_Handle* handle, char* msg, char* end) {
    uint8_t day = 0;
    for (uint8_t i = 0; i < 2 && (msg < end); i++) {
        day *= 10;
        day += *msg-'0';
        msg++;
    }
    uint8_t month = 0;
    for (uint8_t i = 0; i < 2 && (msg < end); i++) {
        month *= 10;
        month += *msg-'0';
        msg++;
    }
    uint8_t year = 0;
    for (uint8_t i = 0; i < 2 && (msg < end); i++) {
        year *= 10;
        year += *msg-'0';
        msg++;
    }
    handle->day = day;
    handle->month = month;
    handle->year = year;
}

static void nmea_parse_param(struct Nmea_Handle* handle, uint8_t message, uint8_t param, char* start, char* end) {
    switch (message) {
    case Nmea_MessageGGA:
        switch (param) {
        case 6:
            handle->satCount = Str_ParseByte(start, end);
            return;
        case 8:
            handle->altitude = Str_ParseFloat(start, end);
            return;
        }
        return;
    case Nmea_MessageRMC:
        switch (param) {
        case 0:
            nmea_parse_utc(handle, start, end);
            return;
        case 1:
            handle->fix = *start == 'A';
            return;
        case 2:
            handle->latitude = nmea_parse_coord(start, end, 2);
            return;
        case 3:
            handle->latHem = nmea_parse_hem(start, end);
            return;
        case 4:
            handle->longitude = nmea_parse_coord(start, end, 3);
            return;
        case 5:
            handle->lonHem = nmea_parse_hem(start, end);
            return;
        case 8:
            nmea_parse_date(handle, start, end);
            return;
        }
        return;
    case Nmea_MessageVTG:
        switch(param){
        case 7:
            handle->speed = Str_ParseFloat(start, end) * 0.621371;
            return;
        }
        return;
    default:
        return;
    }
}

static void nmea_parse_msg(struct Nmea_Handle* handle, char* start, char* end) {
    const uint16_t len = end-start;
    if (len < NMEA_MIN_LENGTH) {
        return;
    }
    //$GP GLL, (skip first three characters)
    start += 3;

    uint8_t count = 0;
    char* msg = NULL;
    for (uint8_t m = 0; m < NMEA_MESSAGE_COUNT; m++) {
        if (Str_Equal(start, messages[m], 3)) {
            start += 3;
            for (; start < end; start++) {
                if (*start == ',') {
                    if (msg) {
                        nmea_parse_param(handle, m, count, msg+1, start);
                        count++;
                    }
                    msg = start;
                }
            }
            if (msg) {
                nmea_parse_param(handle, m, count, msg+1, start);
            }
            return;
        }
    }
}
//  if (Str_Comp(start, "GGA", 3)) {
//      start += 3;
//      char* msg_begin
//      for (; start < end; start++) {
//
//      }
//  }else if (Str_Comp(start, "RMC", 3)) {
//      start += 3;
//
//  }else if (Str_Comp(start, "VTG", 3)) {
//      start += 3;
//
//  }

//  start += 2;
//  if (Str_Comp(start, "GGA", 3)) {
//      start += 3;
//      uint8_t count = 0;
//      char last = ' ';
//      while (last) {
//          start++;
//          char* end = Str_To(start, ',');
//          last = *end;
//          *end = '\0';
//          //start is a readable string now
//          switch (count) {
//          case 6:
//              handle->satCount = Str_ParseByte(start);
//              break;
//          case 8:
//              handle->altitude = Str_ParseFloat(start);
//              break;
//          }
//          start = end;
//          count++;
//      }
//      return;
//  }
//  if (Str_Comp(start, "RMC", 3)) {
//      start += 3;
//      uint8_t count = 0;
//      char last = ' ';
//      while (last) {
//          start++;
//          char* end = Str_To(start, ',');
//          last = *end;
//          *end = '\0';
//          //start is a readable string now
//          switch (count) {
//          case 0:
//              nmea_parse_utc(handle, start);
//              break;
//          case 1:
//              handle->fix = *start == 'A';
//              break;
//          case 2:
//              handle->latitude = nmea_parse_coord(start, 2);
//              break;
//          case 3:
//              handle->latHem = nmea_parse_hem(start);
//              break;
//          case 4:
//              handle->longitude = nmea_parse_coord(start, 3);
//              break;
//          case 5:
//              handle->lonHem = nmea_parse_hem(start);
//              break;
//          case 8:
//              nmea_parse_date(handle, start);
//              break;
//          }
//          start = end;
//          count++;
//      }
//      return;
//  }
//  if (Str_Comp(start, "VTG", 3)) {
//      start += 3;
//      start = Str_Count(start, ',', 7);
//      start++;
//      char* speed_end = Str_To(start, ',');
//      *speed_end = '\0';
//      handle->speed = Str_ParseFloat(start) * 0.621371;
//      return;
//  }

void Nmea_Init(struct Nmea_Handle* handle) {
    handle->latHem = '-';
    handle->lonHem = '-';
}

void Nmea_Parse(struct Nmea_Handle* handle, uint8_t* data, uint16_t len) {
    uint8_t* end = data + len;
    uint8_t* begin = NULL;
    for (; (data < end); data++) {
        if (*data == '$') {
            if (begin) {
                nmea_parse_msg(handle, (char*)begin, (char*)data);
            }
            begin = data;
        }
    }
    if (begin) {
        nmea_parse_msg(handle, (char*)begin, (char*)data);
    }
}

uint8_t Nmea_ExtFlag(struct Nmea_Handle* handle, uint16_t pin) {
    return handle->intPin == pin;
}
void Nmea_ExtHandler(struct Nmea_Handle* handle) {
    handle->second += handle->timepulse;
}
