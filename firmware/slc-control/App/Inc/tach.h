#ifndef INC_TACH_H_
#define INC_TACH_H_

#include "stdint.h"
#include "main.h"

struct Tach_Handle {
    // configuration
    uint32_t tick_freq; // freq of Tach_Count

    uint8_t ppr;
    uint16_t max_rpm;

    // calculated
    uint32_t ticks_per_min;
    uint32_t ticks_per_pulse; 
    
    // internal
    uint16_t rpm;

    uint32_t ticks;

    uint32_t tick_total;
    uint32_t pulses;

    uint32_t tick_checkpoint;
    uint32_t rotations;
};

void Tach_Init(struct Tach_Handle* handle);

void Tach_Config(struct Tach_Handle* handle);

// 10-50Hz typical
void Tach_Update(struct Tach_Handle* handle);

// 50kHz typical
void Tach_Tick(struct Tach_Handle* handle);

void Tach_Pulse(struct Tach_Handle* handle);

#endif
