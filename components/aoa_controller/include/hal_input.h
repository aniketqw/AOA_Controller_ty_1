#ifndef HAL_INPUT_H
#define HAL_INPUT_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include "config.h"

typedef struct {
    float sensor_values[SENSOR_BUFFER_SIZE];
    uint32_t write_index;
} circular_buffer_t;

typedef struct {
    circular_buffer_t sensor1_buffer;
    circular_buffer_t sensor2_buffer;
    circular_buffer_t sensor3_buffer;
    float airspeed;
    uint32_t timestamp;
    char flight_mode[16];
    SemaphoreHandle_t buffer_mutex;
} sensor_data_t;

void hal_input_init(EventGroupHandle_t event_group);
void circular_buffer_init(circular_buffer_t *buffer);
void circular_buffer_push(circular_buffer_t *buffer, float value);
float circular_buffer_get_newest(const circular_buffer_t *buffer);
sensor_data_t *hal_get_sensor_data(void);
void hal_lock_sensor_data(void);
void hal_unlock_sensor_data(void);
void hal_input_task(void *pvParameters);

#endif
