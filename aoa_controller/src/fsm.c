#include "fsm.h"
#include <string.h>
#include <esp_log.h>
#include "thresholds.h"

static const char *TAG = "FSM";
static fsm_context_t fsm_context = {0};

void fsm_init(void) {
    fsm_context.current_state = FSM_STATE_NORMAL;
    fsm_context.aoa_limit_low = 0.0f;
    fsm_context.aoa_limit_high = 20.0f;
}

void fsm_set_thresholds(const char *aircraft_type, const char *flight_mode) {
    float low = 0, high = 0;
    if (thresholds_lookup(aircraft_type, flight_mode, &low, &high)) {
        fsm_context.aoa_limit_low = low;
        fsm_context.aoa_limit_high = high;
        ESP_LOGI(TAG, "Thresholds set: %s/%s -> Low=%.1f High=%.1f",
                 aircraft_type, flight_mode, low, high);
    }
}

fsm_context_t *fsm_get_context(void) { return &fsm_context; }

fsm_output_t fsm_run(float calculated_aoa) {
    fsm_output_t out = {0};
    out.aoa_value = calculated_aoa;

    float limit_high = fsm_context.aoa_limit_high;
    float limit_protection = limit_high - FSM_PROTECTION_MARGIN_DEG;
    float limit_caution = limit_high - FSM_CAUTION_MARGIN_DEG;

    fsm_state_t new_state = FSM_STATE_NORMAL;
    if (calculated_aoa > limit_high) new_state = FSM_STATE_OVERRIDE;
    else if (calculated_aoa > limit_protection) new_state = FSM_STATE_PROTECTION;
    else if (calculated_aoa > limit_caution) new_state = FSM_STATE_CAUTION;

    if (new_state != fsm_context.current_state) {
        fsm_context.current_state = new_state;
        fsm_context.state_entry_time = 0;
        ESP_LOGI(TAG, "FSM state -> %d", new_state);
    }

    out.state = fsm_context.current_state;
    switch (out.state) {
        case FSM_STATE_NORMAL:
            out.led_on = false; out.led_blink_period = 0; strncpy(out.status_str, "NORMAL", sizeof(out.status_str)-1); break;
        case FSM_STATE_CAUTION:
            out.led_on = true; out.led_blink_period = LED_CAUTION_BLINK_PERIOD_MS; strncpy(out.status_str, "CAUTION", sizeof(out.status_str)-1); break;
        case FSM_STATE_PROTECTION:
            out.led_on = true; out.led_blink_period = LED_PROTECTION_BLINK_PERIOD_MS; strncpy(out.status_str, "PROTECTION", sizeof(out.status_str)-1); break;
        case FSM_STATE_OVERRIDE:
            out.led_on = true; out.led_blink_period = 0; strncpy(out.status_str, "OVERRIDE", sizeof(out.status_str)-1); break;
        default:
            strncpy(out.status_str, "UNKNOWN", sizeof(out.status_str)-1); out.led_on = false; break;
    }

    ESP_LOGD(TAG, "FSM: %s aoa=%.2f limit=%.2f", out.status_str, calculated_aoa, limit_high);
    return out;
}
