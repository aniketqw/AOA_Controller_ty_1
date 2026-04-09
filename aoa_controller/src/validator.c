#include "validator.h"
#include <math.h>
#include <esp_log.h>

static const char *TAG = "VALIDATOR";

float calculate_median(float v1, float v2, float v3) {
    if (v1 > v2) { float t = v1; v1 = v2; v2 = t; }
    if (v2 > v3) { float t = v2; v2 = v3; v3 = t; }
    if (v1 > v2) { float t = v1; v1 = v2; v2 = t; }
    return v2;
}

validator_result_t validator_run(sensor_data_t *sensor_data) {
    validator_result_t result = {0};

    result.sensor1_value = circular_buffer_get_newest(&sensor_data->sensor1_buffer);
    result.sensor2_value = circular_buffer_get_newest(&sensor_data->sensor2_buffer);
    result.sensor3_value = circular_buffer_get_newest(&sensor_data->sensor3_buffer);

    result.median_aoa = calculate_median(result.sensor1_value,
                                         result.sensor2_value,
                                         result.sensor3_value);

    float diff1 = fabsf(result.sensor1_value - result.median_aoa);
    float diff2 = fabsf(result.sensor2_value - result.median_aoa);
    float diff3 = fabsf(result.sensor3_value - result.median_aoa);

    result.sensor1_valid = (diff1 <= OUTLIER_THRESHOLD_DEG);
    result.sensor2_valid = (diff2 <= OUTLIER_THRESHOLD_DEG);
    result.sensor3_valid = (diff3 <= OUTLIER_THRESHOLD_DEG);

    result.num_valid_sensors = 0;
    if (result.sensor1_valid) result.num_valid_sensors++;
    if (result.sensor2_valid) result.num_valid_sensors++;
    if (result.sensor3_valid) result.num_valid_sensors++;

    ESP_LOGI(TAG, "Validation: S1=%.1f(%s) S2=%.1f(%s) S3=%.1f(%s) Median=%.1f Valid=%u",
             result.sensor1_value, result.sensor1_valid ? "OK" : "BAD",
             result.sensor2_value, result.sensor2_valid ? "OK" : "BAD",
             result.sensor3_value, result.sensor3_valid ? "OK" : "BAD",
             result.median_aoa, result.num_valid_sensors);

    return result;
}

validator_result_t validator_run_values(float s1, float s2, float s3) {
    validator_result_t result = {0};
    result.sensor1_value = s1;
    result.sensor2_value = s2;
    result.sensor3_value = s3;

    result.median_aoa = calculate_median(s1, s2, s3);

    float diff1 = fabsf(s1 - result.median_aoa);
    float diff2 = fabsf(s2 - result.median_aoa);
    float diff3 = fabsf(s3 - result.median_aoa);

    result.sensor1_valid = (diff1 <= OUTLIER_THRESHOLD_DEG);
    result.sensor2_valid = (diff2 <= OUTLIER_THRESHOLD_DEG);
    result.sensor3_valid = (diff3 <= OUTLIER_THRESHOLD_DEG);

    result.num_valid_sensors = 0;
    if (result.sensor1_valid) result.num_valid_sensors++;
    if (result.sensor2_valid) result.num_valid_sensors++;
    if (result.sensor3_valid) result.num_valid_sensors++;

    ESP_LOGD(TAG, "Validation (values): S1=%.1f(%s) S2=%.1f(%s) S3=%.1f(%s) Median=%.1f Valid=%u",
             result.sensor1_value, result.sensor1_valid ? "OK" : "BAD",
             result.sensor2_value, result.sensor2_valid ? "OK" : "BAD",
             result.sensor3_value, result.sensor3_valid ? "OK" : "BAD",
             result.median_aoa, result.num_valid_sensors);

    return result;
}
