#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "stdint.h"

/* BEGIN CONFIG */
/* END CONFIG */

#define NMEA_BUF_SIZE 90
#define NMEA_MIN_LENGTH 5

struct Time {
	uint8_t hour;
	uint8_t min;
	float second;

	uint8_t day;
	uint8_t month;
	uint8_t year;
};

struct Position {
	float latitude;
	char latHem;
	float longitude;
	char lonHem;
	float altitude;
};

void Nmea_Parse(char* data);
void Nmea_TpHandler(void);

float Nmea_GetSpeed(void);

uint8_t Nmea_GetSatelliteCount(void);

struct Time* Nmea_GetTime(void);
struct Position* Nmea_GetPosition(void);

#endif
