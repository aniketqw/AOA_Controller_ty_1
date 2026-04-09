#include "logger.h"
#include <stdio.h>
#include <string.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>

static const char *TAG = "LOGGER";
static uint32_t led_blink_counter = 0;
static uint32_t led_blink_period = 0;

void logger_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO_PIN, 0);
    ESP_LOGI(TAG, "Logger initialized");
}

void logger_control_led(bool led_on, uint32_t blink_period_ms) {
    led_blink_period = blink_period_ms;
    if (blink_period_ms == 0) { gpio_set_level(LED_GPIO_PIN, led_on ? 1 : 0); }
}

static void update_led_blink(void) {
    if (led_blink_period == 0) return;
    led_blink_counter += CONTROL_CYCLE_PERIOD_MS;
    if (led_blink_counter >= led_blink_period) {
        led_blink_counter = 0;
        gpio_set_level(LED_GPIO_PIN, 1 - gpio_get_level(LED_GPIO_PIN));
    }
}

void logger_write_entry(const log_entry_t *entry) {
    char log_line[256];
    snprintf(log_line, sizeof(log_line),
             "IDX=%lu TS=%lu MODE=%s S1=%.2f S2=%.2f S3=%.2f AIRSPEED=%.1f AOA=%.2f STATUS=%s\r\n",
             entry->cycle_index,
             entry->timestamp,
             entry->flight_mode,
             entry->sensor1,
             entry->sensor2,
             entry->sensor3,
             entry->airspeed,
             entry->calculated_aoa,
             entry->status);
    uart_write_bytes(UART_NUM_0, (const char *)log_line, strlen(log_line));
    ESP_LOGD(TAG, "Logged: %s", log_line);
    update_led_blink();
}
