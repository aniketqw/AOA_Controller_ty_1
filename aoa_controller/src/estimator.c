#include "estimator.h"
#include <esp_log.h>

static const char *TAG = "ESTIMATOR";
static kalman_state_t kalman_state = {0};

void estimator_init(void) {
    kalman_state.estimated_aoa = 0.0f;
    kalman_state.estimated_variance = 1.0f;
    ESP_LOGI(TAG, "Estimator initialized");
}

float perform_weighted_fusion(validator_result_t *validator_result) {
    if (!validator_result || validator_result->num_valid_sensors == 0) return 0.0f;

    float sum = 0.0f;
    float weights = 0.0f;
    const float w = 1.0f;

    if (validator_result->sensor1_valid) { sum += w * validator_result->sensor1_value; weights += w; }
    if (validator_result->sensor2_valid) { sum += w * validator_result->sensor2_value; weights += w; }
    if (validator_result->sensor3_valid) { sum += w * validator_result->sensor3_value; weights += w; }

    return weights > 0.0f ? (sum / weights) : 0.0f;
}

void apply_kalman_filter(float measurement, kalman_state_t *kstate) {
    if (!kstate) return;
    float prior = kstate->estimated_aoa;
    float prior_var = kstate->estimated_variance + KALMAN_PROCESS_NOISE;
    float gain = prior_var / (prior_var + KALMAN_MEASUREMENT_NOISE);
    kstate->estimated_aoa = prior + gain * (measurement - prior);
    kstate->estimated_variance = (1.0f - gain) * prior_var;
}

void estimator_run(validator_result_t *validator_result, estimator_state_t *estimator_state) {
    if (!estimator_state) return;
    estimator_state->fused_aoa = perform_weighted_fusion(validator_result);
    apply_kalman_filter(estimator_state->fused_aoa, &kalman_state);
    estimator_state->final_calculated_aoa = kalman_state.estimated_aoa;
    estimator_state->is_initialized = true;
    ESP_LOGD(TAG, "Estimation AOA=%.2f", estimator_state->final_calculated_aoa);
}
