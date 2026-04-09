#ifndef AOACONFIG_H
#define AOACONFIG_H

#include <stdint.h>

#define CONTROL_CYCLE_PERIOD_MS 20

#define SENSOR_BUFFER_SIZE 5

#define OUTLIER_THRESHOLD_DEG 5.0f
#define SENSOR_MIN_DEG -20.0f
#define SENSOR_MAX_DEG 40.0f
#define AIRSPEED_MIN_KTS 50.0f
#define AIRSPEED_MAX_KTS 500.0f

#define KALMAN_PROCESS_NOISE 0.01f
#define KALMAN_MEASUREMENT_NOISE 0.5f

#define FSM_CAUTION_MARGIN_DEG 2.0f
#define FSM_PROTECTION_MARGIN_DEG 0.5f

#define LED_CAUTION_BLINK_PERIOD_MS 500
#define LED_PROTECTION_BLINK_PERIOD_MS 200

#define LED_GPIO_PIN 4
#define UART_TX_PIN 1
#define UART_RX_PIN 3
#define UART_BAUD_RATE 115200
#define UART_BUFFER_SIZE 256

#define BIT_DATA_READY (1 << 0)
#define BIT_RUN_CYCLE (1 << 1)

typedef struct {
    char aircraft_type[16];
    char flight_mode[16];
    float aoa_low;
    float aoa_high;
} threshold_entry_t;

typedef enum {
    FSM_STATE_NORMAL = 0,
    FSM_STATE_CAUTION = 1,
    FSM_STATE_PROTECTION = 2,
    FSM_STATE_OVERRIDE = 3
} fsm_state_t;

#endif
