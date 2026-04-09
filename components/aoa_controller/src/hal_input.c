#include "hal_input.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <driver/uart.h>
#include <esp_log.h>

static const char *TAG = "HAL_INPUT";
static sensor_data_t sensor_data = {0};
static EventGroupHandle_t event_group_handle = NULL;

void circular_buffer_init(circular_buffer_t *buffer) {
    memset(buffer->sensor_values, 0, sizeof(buffer->sensor_values));
    buffer->write_index = 0;
}

void circular_buffer_push(circular_buffer_t *buffer, float value) {
    buffer->sensor_values[buffer->write_index] = value;
    buffer->write_index = (buffer->write_index + 1) % SENSOR_BUFFER_SIZE;
}

float circular_buffer_get_newest(const circular_buffer_t *buffer) {
    uint32_t newest = buffer->write_index == 0 ? (SENSOR_BUFFER_SIZE - 1) : (buffer->write_index - 1);
    return buffer->sensor_values[newest];
}

sensor_data_t *hal_get_sensor_data(void) { return &sensor_data; }
void hal_lock_sensor_data(void) { xSemaphoreTake(sensor_data.buffer_mutex, portMAX_DELAY); }
void hal_unlock_sensor_data(void) { xSemaphoreGive(sensor_data.buffer_mutex); }

static int parse_aoa_message(const char *line) {
    if (strncmp(line, "$AOA,", 5) != 0) return -1;
    float s1, s2, s3; uint32_t ts;
    if (sscanf(line, "$AOA,S1=%f,S2=%f,S3=%f,TS=%lu*", &s1, &s2, &s3, &ts) != 4) return -1;
    if (s1 < SENSOR_MIN_DEG || s1 > SENSOR_MAX_DEG || s2 < SENSOR_MIN_DEG || s2 > SENSOR_MAX_DEG || s3 < SENSOR_MIN_DEG || s3 > SENSOR_MAX_DEG) return -1;
    hal_lock_sensor_data();
    circular_buffer_push(&sensor_data.sensor1_buffer, s1);
    circular_buffer_push(&sensor_data.sensor2_buffer, s2);
    circular_buffer_push(&sensor_data.sensor3_buffer, s3);
    sensor_data.timestamp = ts;
    hal_unlock_sensor_data();
    if (event_group_handle) xEventGroupSetBits(event_group_handle, BIT_DATA_READY);
    ESP_LOGD(TAG, "AoA parsed: S1=%.1f S2=%.1f S3=%.1f TS=%lu", s1, s2, s3, ts);
    return 0;
}

static int parse_flight_params_message(const char *line) {
    if (strncmp(line, "$FLIGHT_PARAMS,", 15) != 0) return -1;
    float airspeed; uint32_t ts;
    if (sscanf(line, "$FLIGHT_PARAMS,AIRSPEED=%f,TS=%lu*", &airspeed, &ts) != 2) return -1;
    if (airspeed < AIRSPEED_MIN_KTS || airspeed > AIRSPEED_MAX_KTS) return -1;
    hal_lock_sensor_data(); sensor_data.airspeed = airspeed; hal_unlock_sensor_data();
    ESP_LOGD(TAG, "Flight params: AIRSPEED=%.1f TS=%lu", airspeed, ts);
    return 0;
}

static int parse_flight_mode_message(const char *line) {
    if (strncmp(line, "$FLIGHT_MODE,", 13) != 0) return -1;
    char mode_str[32]; uint32_t ts;
    if (sscanf(line, "$FLIGHT_MODE,MODE=%31[^,],TS=%lu*", mode_str, &ts) != 2) return -1;
    hal_lock_sensor_data();
    strncpy(sensor_data.flight_mode, mode_str, sizeof(sensor_data.flight_mode) - 1);
    sensor_data.flight_mode[sizeof(sensor_data.flight_mode) - 1] = '\0';
    hal_unlock_sensor_data();

    ESP_LOGD(TAG, "Flight mode: MODE=%s TS=%lu", sensor_data.flight_mode, ts);
    return 0;
}

static void process_uart_line(const char *line) {
    if (!line || !*line) return;
    if (parse_aoa_message(line) == 0) return;
    if (parse_flight_params_message(line) == 0) return;
    if (parse_flight_mode_message(line) == 0) return;
    ESP_LOGD(TAG, "Unrecognized message: %s", line);
}

void hal_input_task(void *pvParameters) {
    EventGroupHandle_t event_group = (EventGroupHandle_t)pvParameters;
    event_group_handle = event_group;
    uint8_t uart_buffer[UART_BUFFER_SIZE];
    char line_buffer[UART_BUFFER_SIZE];
    int line_index = 0;
    ESP_LOGI(TAG, "Input task started");
    for (;;) {
        int bytes = uart_read_bytes(UART_NUM_0, uart_buffer, UART_BUFFER_SIZE - 1, pdMS_TO_TICKS(100));
        if (bytes > 0) {
            for (int i = 0; i < bytes; ++i) {
                char c = (char)uart_buffer[i];
                if (c == '\n') { line_buffer[line_index] = '\0'; process_uart_line(line_buffer); line_index = 0; }
                else if (c != '\r') { if (line_index < UART_BUFFER_SIZE - 1) line_buffer[line_index++] = c; }
            }
        }
        taskYIELD();
    }
}

void hal_input_init(EventGroupHandle_t event_group) {
    circular_buffer_init(&sensor_data.sensor1_buffer);
    circular_buffer_init(&sensor_data.sensor2_buffer);
    circular_buffer_init(&sensor_data.sensor3_buffer);
    sensor_data.airspeed = 0.0f; sensor_data.timestamp = 0;
    sensor_data.buffer_mutex = xSemaphoreCreateMutex();
    if (!sensor_data.buffer_mutex) { ESP_LOGE(TAG, "Failed to create mutex"); return; }
    event_group_handle = event_group;
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, UART_BUFFER_SIZE, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "HAL Input initialized");
}
