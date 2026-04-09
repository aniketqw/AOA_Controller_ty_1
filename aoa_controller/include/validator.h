#ifndef VALIDATOR_H
#define VALIDATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "hal_input.h"

typedef struct {
    float sensor1_value;
    float sensor2_value;
    float sensor3_value;
    bool sensor1_valid;
    bool sensor2_valid;
    bool sensor3_valid;
    float median_aoa;
    uint32_t num_valid_sensors;
} validator_result_t;

validator_result_t validator_run(sensor_data_t *sensor_data);
float calculate_median(float v1, float v2, float v3);
validator_result_t validator_run_values(float s1, float s2, float s3);

#endif
