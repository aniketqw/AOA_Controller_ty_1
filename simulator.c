#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// Mock ESP-IDF functions
#define ESP_LOGI(tag, fmt, ...) printf("[%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW ESP_LOGI
#define ESP_LOGD ESP_LOGI

typedef int EventGroupHandle_t;
typedef int SemaphoreHandle_t;
#define portMAX_DELAY 0

void xSemaphoreTake(SemaphoreHandle_t sem, int delay) {}
void xSemaphoreGive(SemaphoreHandle_t sem) {}
void xEventGroupSetBits(EventGroupHandle_t eg, int bits) {}

// Include project headers
#include "components/aoa_controller/include/config.h"
#include "components/aoa_controller/include/hal_input.h"
#include "components/aoa_controller/include/elevator.h"
#include "components/aoa_controller/include/fsm.h"

// Global variables  
static EventGroupHandle_t event_group_handle = 0;

// Forward declarations from hal_input.c
int parse_aoa_message(const char *line);

// Main simulation
int main() {
    hal_input_init(event_group_handle);
    elevator_init();
    fsm_init();
    fsm_set_thresholds("Aircraft_A", "CRUISE"); // Default thresholds
    char line[256];
    while (1) {
        printf("Enter AOA message (e.g. $AOA,S1=10.0,S2=10.0,S3=10.0,TS=12345*): ");
        if (fgets(line, sizeof(line), stdin) == NULL) break;
        line[strcspn(line, "\n")] = 0; // remove newline
        if (parse_aoa_message(line) == 0) {
            // Process
            sensor_data_t *sd = hal_get_sensor_data();
            float s1 = circular_buffer_get_newest(&sd->sensor1_buffer);
            float s2 = circular_buffer_get_newest(&sd->sensor2_buffer);
            float s3 = circular_buffer_get_newest(&sd->sensor3_buffer);
            float aoa = (s1 + s2 + s3) / 3.0f; // simple average
            fsm_output_t fsm_out = fsm_run(aoa); // Real FSM (also calls elevator_apply_cmd internally)
            elevator_tick(20); // 20ms
            printf("AOA: %.1f, State: %s, Elevator: %.1f, LED: %s\n", aoa, fsm_out.status_str, elevator_get_current_deflection(), fsm_out.led_on ? "ON" : "OFF");
        }
    }
    return 0;
}