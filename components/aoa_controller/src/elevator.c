#include "elevator.h"
#include <math.h>
#include <esp_log.h>

static const char *TAG = "ELEVATOR";

// Encapsulated Module State (Zero Global Variables Rule)
typedef struct {
    float current_deflection_deg;
    float target_deflection_deg;
    bool cmd_active;
} elevator_state_t;

static elevator_state_t elevator_state = {0};

void elevator_init(void) {
    elevator_state.current_deflection_deg = 0.0f;
    elevator_state.target_deflection_deg = 0.0f;
    elevator_state.cmd_active = false;
    ESP_LOGI(TAG, "Elevator Command Manager initialized");
}

void elevator_apply_cmd(fsm_state_t current_state, float calculated_aoa, float limit_high) {
    // 1. Determine Target based on FSM state
    if (current_state == FSM_STATE_PROTECTION || current_state == FSM_STATE_OVERRIDE) {
        elevator_state.cmd_active = true;
        // Simple proportional push-down logic: 2 degrees of elevator per 1 degree of AoA exceedance
        float overage = calculated_aoa - limit_high;
        if (overage < 0) overage = 0; 
        elevator_state.target_deflection_deg = -(overage * 2.0f); // Negative means pitch down
    } else {
        elevator_state.cmd_active = false;
        elevator_state.target_deflection_deg = 0.0f; // Return to neutral
    }

    // 2. Enforce Clamping (Saturation Protection)
    if (elevator_state.target_deflection_deg < -ELEVATOR_MAX_DEFLECTION_DEG) {
        elevator_state.target_deflection_deg = -ELEVATOR_MAX_DEFLECTION_DEG;
        ESP_LOGW(TAG, "Command saturated at -%.1f deg!", ELEVATOR_MAX_DEFLECTION_DEG);
    }
}

void elevator_tick(uint32_t dt_ms) {
    // Calculate maximum allowed step for this cycle
    float max_step = ELEVATOR_MAX_SLEW_RATE_DEG_PER_SEC * (dt_ms / 1000.0f);
    
    float error = elevator_state.target_deflection_deg - elevator_state.current_deflection_deg;

    // 3. Enforce Slew Rate Limiting
    if (fabsf(error) <= max_step) {
        // Close enough, snap to target
        elevator_state.current_deflection_deg = elevator_state.target_deflection_deg;
    } else {
        // Move towards target by the maximum allowed step
        if (error > 0.0f) {
            elevator_state.current_deflection_deg += max_step;
        } else {
            elevator_state.current_deflection_deg -= max_step;
        }
    }

    // TODO: In a real system, send current_deflection_deg to PWM hardware driver here
}

float elevator_get_current_deflection(void) {
    return elevator_state.current_deflection_deg;
}