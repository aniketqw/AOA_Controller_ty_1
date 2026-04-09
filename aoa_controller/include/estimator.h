#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "validator.h"

typedef struct {
    float fused_aoa;
    float final_calculated_aoa;
    bool is_initialized;
} estimator_state_t;

typedef struct {
    float estimated_aoa;
    float estimated_variance;
} kalman_state_t;

void estimator_init(void);
void estimator_run(validator_result_t *validator_result, estimator_state_t *estimator_state);
float perform_weighted_fusion(validator_result_t *validator_result);
void apply_kalman_filter(float measurement, kalman_state_t *kalman_state);

#endif
