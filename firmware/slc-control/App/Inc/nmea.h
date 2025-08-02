#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "stdint.h"

#define NMEA_BUF_SIZE 90
#define NMEA_MIN_LENGTH 6

#define NMEA_MESSAGE_COUNT 8

enum Nmea_Message {
    Nmea_MessageGGA,
    Nmea_MessageGLL,
    Nmea_MessageGSA,
    Nmea_MessageGSV,
    Nmea_MessageMSS,
    Nmea_MessageRMC,
    Nmea_MessageVTG,
    Nmea_MessageZDA,
};

struct Nmea_Handle {
    uint16_t intPin;
    float timepulse;

    // internal
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
};

void Nmea_Init(struct Nmea_Handle* handle);

void Nmea_Parse(struct Nmea_Handle* handle, char* data, uint16_t len);

uint8_t Nmea_ExtFlag(struct Nmea_Handle* handle, uint16_t pin);
void Nmea_ExtHandler(struct Nmea_Handle* handle);

#endif
