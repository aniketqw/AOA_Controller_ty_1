#ifndef THRESHOLDS_H
#define THRESHOLDS_H

#include "config.h"
#include <stdbool.h>

bool thresholds_lookup(const char *aircraft_type, const char *flight_mode,
                       float *aoa_low, float *aoa_high);

#endif
