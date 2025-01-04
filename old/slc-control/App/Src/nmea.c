#include "nmea.h"
#include "stdint.h"
#include "str.h"

static struct Time time = {0};
static struct Position position = {0};

static uint8_t sat_count = 0;

static float fix = 0;

static float speed = 0;

static char nmea_parse_hem(char* str) {
	if (*str) {
		return *str;
	}
	return '-';
}

static float nmea_parse_coord(char* str, uint8_t degree_digits) {
	uint8_t deg = 0;
	for (uint8_t i = 0; i < degree_digits && *str; i++) {
		deg *= 10;
		deg += *str-'0';
		str++;
	}
	const float min = Str_ParseFloat(str);
	return min/60.0 + (float)deg;
}

static void nmea_parse_utc(char* str) {
	uint8_t hour = 0;
	for (uint8_t i = 0; i < 2 && *str; i++) {
		hour *= 10;
		hour += *str-'0';
		str++;
	}
	uint8_t min = 0;
	for (uint8_t i = 0; i < 2 && *str; i++) {
		min *= 10;
		min += *str-'0';
		str++;
	}
	const float second = Str_ParseFloat(str);

	time.hour = hour;
	time.min = min;
	time.second = second;
}

static void nmea_parse_date(char* str) {
	uint8_t day = 0;
	for (uint8_t i = 0; i < 2 && *str; i++) {
		day *= 10;
		day += *str-'0';
		str++;
	}
	uint8_t month = 0;
	for (uint8_t i = 0; i < 2 && *str; i++) {
		month *= 10;
		month += *str-'0';
		str++;
	}
	uint8_t year = 0;
	for (uint8_t i = 0; i < 2 && *str; i++) {
		year *= 10;
		year += *str-'0';
		str++;
	}
	time.day = day;
	time.month = month;
	time.year = year;
}

static void nmea_parse_msg(char* start) {
	if (!Str_Longer(start, NMEA_MIN_LENGTH)) {
		return;
	}
	start += 2;
	if (Str_Comp(start, "GGA", 3)) {
		start += 3;
		uint8_t count = 0;
		char last = ' ';
		while (last) {
			start++;
			char* end = Str_To(start, ',');
			last = *end;
			*end = '\0';
			//start is a readable string now
			switch (count) {
			case 6:
				sat_count = Str_ParseByte(start);
				break;
			case 8:
				position.altitude = Str_ParseFloat(start);
				break;
			}
			start = end;
			count++;
		}
		return;
	}
	if (Str_Comp(start, "RMC", 3)) {
		start += 3;
		uint8_t count = 0;
		char last = ' ';
		while (last) {
			start++;
			char* end = Str_To(start, ',');
			last = *end;
			*end = '\0';
			//start is a readable string now
			switch (count) {
			case 0:
				nmea_parse_utc(start);
				break;
			case 1:
				fix = *start == 'A';
				break;
			case 2:
				position.latitude = nmea_parse_coord(start, 2);
				break;
			case 3:
				position.latHem = nmea_parse_hem(start);
				break;
			case 4:
				position.longitude = nmea_parse_coord(start, 3);
				break;
			case 5:
				position.lonHem = nmea_parse_hem(start);
				break;
			case 8:
				nmea_parse_date(start);
				break;
			}
			start = end;
			count++;
		}
		return;
	}
	if (Str_Comp(start, "VTG", 3)) {
		start += 3;
		start = Str_Count(start, ',', 7);
		start++;
		char* speed_end = Str_To(start, ',');
		*speed_end = '\0';
		speed = Str_ParseFloat(start) * 0.621371;
		return;
	}
}

void Nmea_Parse(char* data) {
	char* start = Str_To(data, '$');
	char last = ' ';
	while (last) {
		start++;
		char* end = Str_To(start, '$');
		last = *end;
		*end = '\0';
		nmea_parse_msg(start);
		start = end;
	}
}

void Nmea_TpHandler(void) {
//	time.second += 0.1;
}

float Nmea_GetSpeed(void) {
	return speed;
}

uint8_t Nmea_GetSatelliteCount(void) {
	return sat_count;
}

struct Time* Nmea_GetTime(void) {
	return &time;
}
struct Position* Nmea_GetPosition(void) {
	return &position;
}
