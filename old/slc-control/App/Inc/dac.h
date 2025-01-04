#ifndef INC_DAC_H_
#define INC_DAC_H_

#include "stdint.h"

/* BEGIN CONFIG */
#define DAC_COUNT 3

#define DAC_LIST T(hdac1, DAC_CHANNEL_2) T(hdac1, DAC_CHANNEL_1) T(hdac3, DAC_CHANNEL_1)

enum DacId {
	Dac1,
	Dac2,
	Dac3,
};
/* END CONFIG */

void Dac_Set(enum DacId dac, uint16_t value);

#endif
