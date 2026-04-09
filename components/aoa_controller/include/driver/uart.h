// Mock driver/uart.h for PC simulation
#ifndef DRIVER_UART_H
#define DRIVER_UART_H

typedef int uart_port_t;
typedef struct {
    int baud_rate;
    int data_bits;
    int parity;
    int stop_bits;
    int flow_ctrl;
} uart_config_t;

#define UART_NUM_0 0
#define UART_DATA_8_BITS 3
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_PIN_NO_CHANGE -1
#define pdMS_TO_TICKS(ms) ((ms) / 10)

static inline int uart_param_config(uart_port_t port, const uart_config_t *cfg) { return 0; }
static inline int uart_set_pin(uart_port_t port, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num) { return 0; }
static inline int uart_driver_install(uart_port_t port, int rx_buffer_size, int tx_buffer_size, int queue_size, void *queue_handle, int intr_alloc_flags) { return 0; }
static inline int uart_read_bytes(uart_port_t port, unsigned char *buf, unsigned int length, unsigned int timeout_ms) { return 0; }

#endif