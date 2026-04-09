#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "fsm.h"

// Configuration constants (ideally loaded from config, but hardcoded here for simplicity)
#define ELEVATOR_MAX_DEFLECTION_DEG 15.0f
#define ELEVATOR_MAX_SLEW_RATE_DEG_PER_SEC 10.0f

// Initialize the module state
void elevator_init(void);

// Called by the FSM to request a control level (Target calculation)
void elevator_apply_cmd(fsm_state_t current_state, float calculated_aoa, float limit_high);

// Called by main.c every control cycle to step the physical actuator (Slew rate enforcement)
void elevator_tick(uint32_t dt_ms);

// Getter for the logger
float elevator_get_current_deflection(void);

#endif // ELEVATOR_H