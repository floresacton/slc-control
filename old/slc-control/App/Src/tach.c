#include "tach.h"
#include "stm32g4xx_hal.h"
#include "memory.h"
#include "dac.h"

extern TIM_HandleTypeDef TACH_REF_HANDLE;
extern TIM_HandleTypeDef TACH_CALC_HANDLE;

static volatile uint32_t count[TACH_COUNT] = {0};

static volatile uint32_t countAccum[TACH_COUNT] = {0};
static volatile uint32_t passes[TACH_COUNT] = {0};

static volatile uint32_t checkpointAccum[TACH_COUNT] = {0};
static volatile uint32_t checkpointPasses[TACH_COUNT] = {0};

static uint16_t rpm[TACH_COUNT] = {0};


void Tach_Init(void) {
	HAL_TIM_Base_Start_IT(&TACH_REF_HANDLE);
	HAL_TIM_Base_Start_IT(&TACH_CALC_HANDLE);
}

uint8_t Tach_TimFlagCalc(TIM_HandleTypeDef* htim) {
	return htim == &TACH_CALC_HANDLE;
}

void Tach_TimHandlerCalc(void) {
	for (uint8_t i = 0; i < TACH_COUNT; i++) {
		if (checkpointPasses[i]) {
			rpm[i] = (float)TACH_REF_FREQ*60.0*(float)checkpointPasses[i]/((float)checkpointAccum[i]*(float)Memory_ReadByte(MemTach1Spokes+i));
			countAccum[i] = 0;
			checkpointAccum[i] = 0;
		}else if(passes[i]){
			rpm[i] = (float)TACH_REF_FREQ*60.0*(float)passes[i]/((float)countAccum[i]*(float)Memory_ReadByte(MemTach1Spokes+i));
			countAccum[i] = 0;
			checkpointAccum[i] = 0;
		}else {
			const uint16_t new_rpm = (float)TACH_REF_FREQ*60.0/((float)Memory_ReadByte(MemTach1Spokes+i)*(float)count[i]);
			if (new_rpm < rpm[i]) {
				rpm[i] = new_rpm;
			}
		}

		passes[i] = 0;
		checkpointPasses[i] = 0;
	}
}

uint8_t Tach_TimFlagRef(TIM_HandleTypeDef* htim) {
	return htim == &TACH_REF_HANDLE;
}

void Tach_TimHandlerRef(void) {
	count[0]++;
	count[1]++;
	count[2]++;
}

void Tach_Trigger(enum TachId tach) {
	if (count[tach] >= 60.0*(float)TACH_REF_FREQ/(Memory_ReadByte(MemTach1Spokes+tach)*Memory_ReadShort(MemTach1Max+tach))) {
		passes[tach]++;
		countAccum[tach]+=count[tach];
		if (passes[tach] % Memory_ReadByte(MemTach1Spokes+tach) == 0) {
			checkpointPasses[tach] = passes[tach];
			checkpointAccum[tach] = countAccum[tach];
		}
		count[tach] = 0;
	}
}

uint16_t Tach_GetRpm(uint8_t chan) {
	return rpm[chan];
}
