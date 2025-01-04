#include "icm42688.h"

static int16_t vel[3] = {0};
static int16_t accel[3] = {0};

void Icm42688_Init(void) {

}

uint8_t Icm42688_ExtFlag(uint16_t pin) {
	return pin == ICM42688_INT_PIN;
}

void Icm42688_ExtHandler(void) {

}

int16_t Icm4288_GetVelX(void) {
	return vel[0];
}
int16_t Icm4288_GetVelY(void) {
	return vel[1];
}
int16_t Icm4288_GetVelZ(void) {
	return vel[2];
}

int16_t Icm4288_GetAccelX(void) {
	return accel[0];
}
int16_t Icm4288_GetAccelY(void) {
	return accel[1];
}
int16_t Icm4288_GetAccelZ(void) {
	return accel[2];
}
