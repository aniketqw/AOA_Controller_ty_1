// Mock freertos/task.h for PC simulation
#ifndef FREERTOS_TASK_H
#define FREERTOS_TASK_H

typedef int TaskHandle_t;

static inline void vTaskDelete(TaskHandle_t xTaskToDelete) {}
static inline void xTaskCreate(void *pvTaskCode, const char * const pcName, unsigned int usStackDepth, void *pvParameters, unsigned int uxPriority, TaskHandle_t *pxCreatedTask) {}

#endif
