#include "tach.h"

void Tach_Init(struct Tach_Handle* handle) {
    handle->rpm = 0;

    handle->ticks = 0;

    handle->tick_total = 0;
    handle->pulses = 0;
    
    handle->tick_checkpoint = 0;
    handle->rotations = 0;

    Tach_Config(handle);
}

void Tach_Config(struct Tach_Handle* handle) {
    handle->ticks_per_min = handle->tick_freq * 60;

    const uint32_t pulses_per_min = handle->ppr * handle->max_rpm;
    handle->ticks_per_pulse = handle->ticks_per_min / pulses_per_min;
}

void Tach_Update(struct Tach_Handle* handle) {
    if (handle->rotations) {
        const uint32_t ticks_rotations_per_min = handle->ticks_per_min * handle->rotations;

        handle->rpm = ticks_rotations_per_min / handle->tick_checkpoint;
        
        handle->rotations = 0;
        handle->tick_checkpoint = 0;
        
        handle->pulses = 0;
        handle->tick_total = 0;
    }else if (handle->pulses) {
        const uint32_t ticks_pulses_per_min = handle->ticks_per_min * handle->pulses; // up to 90,000,000
        const uint32_t ticks_pulses_per_rotation = handle->tick_total * handle->ppr;

        handle->rpm = ticks_pulses_per_min / ticks_pulses_per_rotation;
        
        handle->pulses = 0;
        handle->tick_total = 0;
    }else {
        const uint32_t ticks_pulses_per_rotation = handle->ticks * handle->ppr;
        const uint32_t rotations_per_min_pulse = handle->ticks_per_min / ticks_pulses_per_rotation;
        
        // pulse implicitly = 1
        const uint32_t new_rpm = rotations_per_min_pulse;
        if (new_rpm < handle->rpm) {
            handle->rpm = new_rpm;
        }
    }
}

void Tach_Tick(struct Tach_Handle* handle) {
    handle->ticks++;
}

void Tach_Pulse(struct Tach_Handle* handle) {
    // pulse implicity = 1
    if (handle->ticks >= handle->ticks_per_pulse) {
        handle->tick_total += handle->ticks;
        handle->pulses++;
        if (handle->pulses % handle->ppr == 0) {
            handle->tick_checkpoint = handle->tick_total;
            handle->rotations++;
        }
        handle->ticks = 0;
    }
}
