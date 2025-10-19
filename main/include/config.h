#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Command data sizes
#define CFG_MAIN_DATA_SIZE 203
#define CFG_EXTRA_DATA_SIZE 110
#define CFG_OPT_DATA_SIZE 20

// UART configuration constants
#define CFG_UART_NUM UART_NUM_2
#define CFG_BAUD_RATE 9600
#define CFG_DATA_BITS UART_DATA_8_BITS
#define CFG_PARITY UART_PARITY_EVEN
#define CFG_STOP_BITS UART_STOP_BITS_1
#define CFG_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE

// GPIO pins
#define CFG_TX_PIN 17
#define CFG_RX_PIN 16

// Timing constants
#define CFG_READ_TIMEOUT_MS 2000
#define CFG_QUERY_INTERVAL_MS 10000

// Protocol constants
#define CFG_QUEUE_SIZE 10

// Decoder constants
#define CFG_NUMBER_OF_TOPICS 139
#define CFG_NUMBER_OF_TOPICS_EXTRA 6
#define CFG_NUMBER_OF_OPT_TOPICS 7

#ifdef __cplusplus
}
#endif
#endif // CONFIG_H