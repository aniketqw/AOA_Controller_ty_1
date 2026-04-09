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
#define ESP_LOGE ESP_LOGI

typedef int EventGroupHandle_t;
typedef int SemaphoreHandle_t;
#define portMAX_DELAY 0

void xSemaphoreTake(SemaphoreHandle_t sem, int delay) {}
void xSemaphoreGive(SemaphoreHandle_t sem) {}
void xEventGroupSetBits(EventGroupHandle_t eg, int bits) {}
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return 0; }
void taskYIELD(void) {}

// Mock UART functions
typedef int uart_port_t;
typedef struct {
    int baud_rate, data_bits, parity, stop_bits, flow_ctrl;
} uart_config_t;

#define UART_NUM_0 0
#define UART_DATA_8_BITS 3
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_PIN_NO_CHANGE -1
#define pdMS_TO_TICKS(ms) ((ms) / 10)

int uart_param_config(uart_port_t p, const uart_config_t *c) { return 0; }
int uart_set_pin(uart_port_t p, int tx, int rx, int rts, int cts) { return 0; }
int uart_driver_install(uart_port_t p, int rx_size, int tx_size, int q_size, void *q, int flags) { return 0; }
int uart_read_bytes(uart_port_t p, unsigned char *buf, unsigned int len, unsigned int timeout) { return 0; }

// Include project headers (minimal imports)
#include "components/aoa_controller/include/config.h"
#include "components/aoa_controller/include/elevator.h"

// Simplified FSM state constants
typedef enum {
    FSM_NORMAL = 0, FSM_CAUTION = 1, FSM_PROTECTION = 2, FSM_OVERRIDE = 3
} simple_fsm_t;

// CSV row structure
typedef struct {
    unsigned int idx, ts;
    char mode[16];
    float s1, s2, s3, airspeed;
    bool s1_valid, s2_valid, s3_valid, airspeed_valid;
} csv_row_t;

// Simple CSV parser
int read_csv_line(FILE *fp, csv_row_t *row) {
    char line[512];
    if (fgets(line, sizeof(line), fp) == NULL) return -1;
    line[strcspn(line, "\n")] = 0;
    
    char *p = line;
    char field[64];
    int field_count = 0;
    
    while (*p && field_count < 7) {
        int i = 0;
        while (*p && *p != ',' && i < 63) field[i++] = *p++;
        field[i] = '\0';
        if (*p == ',') p++;
        
        switch (field_count) {
            case 0: row->idx = atoi(field); break;
            case 1: row->ts = atoi(field); break;
            case 2: strncpy(row->mode, field, sizeof(row->mode) - 1); break;
            case 3: row->s1_valid = (strlen(field) > 0); row->s1 = row->s1_valid ? atof(field) : 0.0f; break;
            case 4: row->s2_valid = (strlen(field) > 0); row->s2 = row->s2_valid ? atof(field) : 0.0f; break;
            case 5: row->s3_valid = (strlen(field) > 0); row->s3 = row->s3_valid ? atof(field) : 0.0f; break;
            case 6: row->airspeed_valid = (strlen(field) > 0); row->airspeed = row->airspeed_valid ? atof(field) : 0.0f; break;
        }
        field_count++;
    }
    return 0;
}

// Simple FSM logic (without calling project code to avoid crashes)
simple_fsm_t simple_fsm_run(float aoa, float aoa_high) {
    if (aoa > aoa_high) return FSM_OVERRIDE;
    if (aoa > (aoa_high - 0.5f)) return FSM_PROTECTION;
    if (aoa > (aoa_high - 2.0f)) return FSM_CAUTION;
    return FSM_NORMAL;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <csv_file>\n", argv[0]);
        return 1;
    }
    
    FILE *fp = fopen(argv[1], "r");
    if (!fp) {
        printf("Error: Could not open %s\n", argv[1]);
        return 1;
    }
    
    // Initialize only elevator (safer than full FSM)
    elevator_init();
    
    printf("=== AOA Controller CSV Simulation (Safe Mode) ===\n");
    printf("idx,ts,mode,s1,s2,s3,aoa,fsm_state,elevator\n");
    
    char header[512];
    fgets(header, sizeof(header), fp);
    
    csv_row_t row;
    unsigned int count = 0;
    float aoa_high = 22.0f; // Default CRUISE threshold
    
    while (read_csv_line(fp, &row) == 0 && count < 200) {
        float s1 = row.s1_valid ? row.s1 : 0.0f;
        float s2 = row.s2_valid ? row.s2 : 0.0f;
        float s3 = row.s3_valid ? row.s3 : 0.0f;
        float aoa = (s1 + s2 + s3) / 3.0f;
        
        // Update threshold based on mode
        if (strlen(row.mode) > 0) {
            if (strcmp(row.mode, "TAKEOFF") == 0) aoa_high = 15.0f;
            else if (strcmp(row.mode, "CLIMB") == 0) aoa_high = 18.0f;
            else if (strcmp(row.mode, "CRUISE") == 0) aoa_high = 22.0f;
            else if (strcmp(row.mode, "LANDING") == 0) aoa_high = 16.0f;
        }
        
        // Simple FSM
        simple_fsm_t fsm_state = simple_fsm_run(aoa, aoa_high);
        
        // Elevator command (simplified, no project code)
        float elevator_cmd = 0.0f;
        if (fsm_state == FSM_PROTECTION || fsm_state == FSM_OVERRIDE) {
            float overage = aoa - (aoa_high - 0.5f);
            if (overage > 0) elevator_cmd = -(overage * 2.0f);
            if (elevator_cmd < -15.0f) elevator_cmd = -15.0f;
        }
        
        // Slew rate limiting
        static float last_elevator = 0.0f;
        float max_step = 10.0f * (20.0f / 1000.0f); // 10 deg/sec, 20ms cycle
        if (fabsf(elevator_cmd - last_elevator) > max_step) {
            if (elevator_cmd > last_elevator) last_elevator += max_step;
            else last_elevator -= max_step;
        } else {
            last_elevator = elevator_cmd;
        }
        
        printf("%u,%u,%s,%.2f,%.2f,%.2f,%.2f,%d,%.2f\n",
               row.idx, row.ts, strlen(row.mode) > 0 ? row.mode : "-",
               s1, s2, s3, aoa, fsm_state, last_elevator);
        
        count++;
    }
    
    fclose(fp);
    printf("\n=== Complete (%u rows) ===\n", count);
    return 0;
}
