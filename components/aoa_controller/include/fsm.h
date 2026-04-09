#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef struct {
    fsm_state_t current_state;
    float aoa_limit_low;
    float aoa_limit_high;
    uint32_t state_entry_time;
} fsm_context_t;

typedef struct {
    fsm_state_t state;
    float aoa_value;
    bool led_on;
    uint32_t led_blink_period;
    char status_str[32];
} fsm_output_t;

void fsm_init(void);
void fsm_set_thresholds(const char *aircraft_type, const char *flight_mode);
fsm_output_t fsm_run(float calculated_aoa);
fsm_context_t *fsm_get_context(void);

#endif
