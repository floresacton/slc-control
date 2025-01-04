#include "dac.h"
#include "main.h"

#define T(dac, chan) extern DAC_HandleTypeDef dac;
DAC_LIST
#undef T

#define T(dac, chan) &dac,
static DAC_HandleTypeDef *dacs[DAC_COUNT] = { DAC_LIST };
#undef T

#define T(dac, chan) chan,
static uint32_t channels[DAC_COUNT] = { DAC_LIST };
#undef T

void Dac_Set(enum DacId dac, uint16_t value) {
	HAL_DAC_SetValue(dacs[dac], channels[dac], DAC_ALIGN_12B_R, value);
}
