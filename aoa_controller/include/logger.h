#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "fsm.h"

typedef struct {
    uint32_t cycle_index;
    uint32_t timestamp;
    char flight_mode[16];
    float sensor1;
    float sensor2;
    float sensor3;
    float airspeed;
    float calculated_aoa;
    char status[32];
} log_entry_t;

void logger_init(void);
void logger_control_led(bool led_on, uint32_t blink_period_ms);
void logger_write_entry(const log_entry_t *entry);

#endif
