/**
 * @file protocol.h
 * @brief Heat pump protocol communication header
 * @version 1.0.0
 * @date 2024
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// Protocol constants
#define PROTOCOL_UART_NUM UART_NUM_2
#define PROTOCOL_BUFFER_SIZE 256
#define PROTOCOL_MAX_COMMAND_SIZE 256
#define PROTOCOL_QUEUE_SIZE 10

// Data sizes (based on HeishaMon analysis)
#define PROTOCOL_MAIN_DATA_SIZE 203
#define PROTOCOL_EXTRA_DATA_SIZE 110
#define PROTOCOL_OPT_DATA_SIZE 20

// Global buffer for UART communication
extern uint8_t g_protocol_rx_buffer[PROTOCOL_BUFFER_SIZE];

// UART configuration constants
#define PROTOCOL_BAUD_RATE 9600
#define PROTOCOL_DATA_BITS UART_DATA_8_BITS
#define PROTOCOL_PARITY UART_PARITY_EVEN
#define PROTOCOL_STOP_BITS UART_STOP_BITS_1
#define PROTOCOL_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE

// GPIO pins
#define PROTOCOL_TX_PIN 17
#define PROTOCOL_RX_PIN 16

// Timing constants
#define PROTOCOL_READ_TIMEOUT_MS 1000
#define PROTOCOL_QUERY_INTERVAL_MS 10000

// Protocol command types
typedef enum {
    PROTOCOL_CMD_INITIAL = 0,
    PROTOCOL_CMD_MAIN_DATA,
    PROTOCOL_CMD_EXTRA_DATA,
    PROTOCOL_CMD_OPT_DATA,
    PROTOCOL_CMD_WRITE
} protocol_cmd_type_t;

// Protocol packet types
typedef enum {
    PROTOCOL_PKT_READ = 0x71,
    PROTOCOL_PKT_INIT = 0x31,
    PROTOCOL_PKT_WRITE = 0xF1
} protocol_pkt_type_t;

// Protocol data types
typedef enum {
    PROTOCOL_DATA_MAIN = 0x10,
    PROTOCOL_DATA_EXTRA = 0x21,
    PROTOCOL_DATA_OPT = 0x50
} protocol_data_type_t;

// Protocol command structure
typedef struct {
    protocol_cmd_type_t type;
    uint8_t data[PROTOCOL_MAX_COMMAND_SIZE];
    size_t data_size;    
} protocol_cmd_t;

// Protocol context
typedef struct {
    uart_config_t uart_config;
    QueueHandle_t command_queue;
    TaskHandle_t protocol_task_handle;
} protocol_context_t;

// Global protocol context
extern protocol_context_t g_protocol_ctx;

/**
 * @brief Initialize heat pump protocol
 * @return ESP_OK on success
 */
esp_err_t protocol_init(void);

/**
 * @brief Start protocol communication task
 * @return ESP_OK on success
 */
esp_err_t protocol_start(void);


/**
 * @brief Send command to heat pump
 * @param cmd Command to send
 * @return ESP_OK on success
 */
esp_err_t protocol_send_command(const protocol_cmd_t *cmd);

/**
 * @brief Send initial query to heat pump
 * @return ESP_OK on success
 */
esp_err_t protocol_send_initial_query(void);

/**
 * @brief Request main data from heat pump
 * @return ESP_OK on success
 */
esp_err_t protocol_request_main_data(void);

/**
 * @brief Request extra data from heat pump
 * @return ESP_OK on success
 */
esp_err_t protocol_request_extra_data(void);

/**
 * @brief Request optional data from heat pump
 * @return ESP_OK on success
 */
esp_err_t protocol_request_opt_data(void);

/**
 * @brief Send write command to heat pump
 * @param data Data to write
 * @param size Data size
 * @return ESP_OK on success
 */
esp_err_t protocol_send_write_command(const uint8_t *data, size_t size);


/**
 * @brief Calculate checksum for data
 * @param data Data to calculate checksum for
 * @param size Data size
 * @return Calculated checksum
 */
uint8_t protocol_calculate_checksum(const uint8_t *data, size_t size);

/**
 * @brief Validate received data checksum
 * @param data Received data
 * @param size Data size
 * @return true if checksum is valid
 */
bool protocol_validate_checksum(const uint8_t *data, size_t size);

/**
 * @brief Protocol communication task
 * @param pvParameters Task parameters
 */
void protocol_task(void *pvParameters);

/**
 * @brief Hex dump data
 * @param data Data to dump
 * @param size Data size
 */
esp_err_t protocol_hex_dump(const uint8_t *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H
