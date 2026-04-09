#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "hal_input.h"
#include "validator.h"
#include "estimator.h"
#include "fsm.h"
#include "logger.h"

static const char *TAG = "MAIN";
static EventGroupHandle_t event_group = NULL;
static uint32_t cycle_count = 0;
static esp_timer_handle_t control_timer = NULL;

static void control_timer_callback(void *arg) {
    xEventGroupSetBits(event_group, BIT_RUN_CYCLE);
}

static void control_task(void *pvParameters) {
    ESP_LOGI(TAG, "Control task started on Core 1");

    estimator_init();
    fsm_init();
    
    fsm_set_thresholds("Aircraft_A", "CRUISE");

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(
            event_group,
            BIT_RUN_CYCLE,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );

        if (!(bits & BIT_RUN_CYCLE)) continue;

        cycle_count++;

        float s1 = 0.0f, s2 = 0.0f, s3 = 0.0f, airspeed = 0.0f;
        uint32_t timestamp = 0;
        char flight_mode_local[16] = "UNKNOWN";

        hal_lock_sensor_data();
        sensor_data_t *sd = hal_get_sensor_data();
        
        s1 = circular_buffer_get_newest(&sd->sensor1_buffer);
        s2 = circular_buffer_get_newest(&sd->sensor2_buffer);
        s3 = circular_buffer_get_newest(&sd->sensor3_buffer);
        airspeed = sd->airspeed;
        timestamp = sd->timestamp;
        
        strncpy(flight_mode_local, sd->flight_mode, sizeof(flight_mode_local) - 1);
        flight_mode_local[sizeof(flight_mode_local) - 1] = '\0';
        
        hal_unlock_sensor_data();

        validator_result_t validation = validator_run_values(s1, s2, s3);

        estimator_state_t estimator_state = {0};
        estimator_run(&validation, &estimator_state);

        fsm_set_thresholds("Aircraft_A", flight_mode_local);
        fsm_output_t fsm_output = fsm_run(estimator_state.final_calculated_aoa);

        logger_control_led(fsm_output.led_on, fsm_output.led_blink_period);

        log_entry_t log_entry = {0};
        log_entry.cycle_index = cycle_count;
        log_entry.timestamp = timestamp;
        
        strncpy(log_entry.flight_mode, flight_mode_local, sizeof(log_entry.flight_mode) - 1);
        log_entry.flight_mode[sizeof(log_entry.flight_mode) - 1] = '\0';
        
        log_entry.sensor1 = validation.sensor1_value;
        log_entry.sensor2 = validation.sensor2_value;
        log_entry.sensor3 = validation.sensor3_value;
        log_entry.airspeed = airspeed;
        log_entry.calculated_aoa = estimator_state.final_calculated_aoa;
        
        strncpy(log_entry.status, fsm_output.status_str, sizeof(log_entry.status) - 1);
        log_entry.status[sizeof(log_entry.status) - 1] = '\0';

        logger_write_entry(&log_entry);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Aircraft AoA Safety Controller Boot Sequence ===");

    event_group = xEventGroupCreate();
    if (!event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

    hal_input_init(event_group);
    logger_init();

    BaseType_t ret = xTaskCreatePinnedToCore(
        hal_input_task,
        "hal_input_task",
        4096,
        (void *)event_group,
        5,
        NULL,
        0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create input task");
        return;
    }

    ret = xTaskCreatePinnedToCore(
        control_task,
        "control_task",
        4096,
        NULL,
        10,
        NULL,
        1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task");
        return;
    }

    esp_timer_create_args_t timer_args = {
        .callback = control_timer_callback,
        .name = "control_cycle_timer",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
    };

    if (esp_timer_create(&timer_args, &control_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer");
        return;
    }

    if (esp_timer_start_periodic(control_timer, CONTROL_CYCLE_PERIOD_MS * 1000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer");
        return;
    }

    ESP_LOGI(TAG, "System initialized successfully. Entering normal operation.");
}