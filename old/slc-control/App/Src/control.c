#include "control.h"
#include "main.h"
#include "memory.h"
#include "gps.h"
#include "tach.h"

static uint16_t tire_rpm = 0;
static uint16_t vehicle_rpm = 0;
static float slip = 0;

static uint16_t rpm_max(uint16_t a, uint16_t b) {
	if (a > b) {
		return a;
	}
	return b;
}

void Control_Update(void) {
	tire_rpm = rpm_max(Tach_GetRpm(Tach2), Tach_GetRpm(Tach3));

	const uint8_t second = Memory_ReadFloat(MemGpsTire)*Tach_GetRpm(Tach1)*0.00094697 < Memory_ReadShort(MemAuxThresh);
	HAL_GPIO_WritePin(DRV2_GPIO_Port, DRV2_Pin, second);

	vehicle_rpm = rpm_max(Tach_GetRpm(Tach1), Memory_ReadShort(MemMinRpm));

	if (vehicle_rpm < tire_rpm) {
		slip = 100.0*((float)tire_rpm-(float)vehicle_rpm/(float)vehicle_rpm);
	} else {
		slip = 0;
	}

	HAL_GPIO_WritePin(DRV1_GPIO_Port, DRV1_Pin, slip > Memory_ReadFloat(MemSlipThresh));
}

float Control_GetSlip(void) {
	return slip;
}

float Control_GetRatio(void) {
	if (vehicle_rpm == 0) {
		return 0;
	}
	return (float)tire_rpm/(float)vehicle_rpm;
}
