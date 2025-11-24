/**
 * @file protocol.h
 * @brief Heat pump protocol communication header
 * @version 0.1.0
 * @date 2025-11
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// UART configuration
#define PROTOCOL_UART_NUM UART_NUM_2
#define PROTOCOL_BAUD_RATE 9600
#define PROTOCOL_DATA_BITS UART_DATA_8_BITS
#define PROTOCOL_PARITY UART_PARITY_EVEN
#define PROTOCOL_STOP_BITS UART_STOP_BITS_1
#define PROTOCOL_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE

// GPIO pins
#define PROTOCOL_TX_PIN 17
#define PROTOCOL_RX_PIN 16

// Timing constants
#define PROTOCOL_READ_TIMEOUT_MS 2000
#define PROTOCOL_QUERY_INTERVAL_MS 10000

// Queue size
#define PROTOCOL_QUEUE_SIZE 10

// Command data sizes
#define PROTOCOL_MAX_DATA_SIZE 256
#define PROTOCOL_MAIN_DATA_SIZE 203
#define PROTOCOL_EXTRA_DATA_SIZE 110
#define PROTOCOL_OPT_DATA_SIZE 20
#define PROTOCOL_WRITE_SIZE 110
#define PROTOCOL_OPT_WRITE_SIZE 19
#define PROTOCOL_HANDSHAKE_DATA_SIZE 51

#define PROTOCOL_OPT_AVAILABLE false

// RX buffer with length metadata
typedef struct {
    uint8_t data[PROTOCOL_MAX_DATA_SIZE];
    size_t len;
} protocol_rx_t;

extern protocol_rx_t g_protocol_rx;

// Protocol packet types
typedef enum {
    PROTOCOL_PKT_READ = 0x71,
    PROTOCOL_PKT_INIT = 0x31,
    PROTOCOL_PKT_WRITE = 0xF1,
    PROTOCOL_PKT_HANDSHAKE = 0x10
} protocol_pkt_type_t;

// Protocol data types
typedef enum {
    PROTOCOL_DATA_MAIN = 0x10,
    PROTOCOL_DATA_EXTRA = 0x21,
    PROTOCOL_DATA_OPT = 0x50
} protocol_data_type_t;

// Protocol command structure
typedef struct {
    uint8_t data[PROTOCOL_WRITE_SIZE];
    size_t len;    
} protocol_cmd_t;

// Protocol context
typedef struct {
    QueueHandle_t command_queue;
    TaskHandle_t protocol_task_handle;
    bool extra_data_block_available;
    bool opt_data_block_available;
} protocol_context_t;

// Global protocol context
extern protocol_context_t g_protocol_ctx;

extern const uint8_t panasonic_query[];
extern const uint8_t optional_pcb_query[];

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
 * @brief Минидамп 256-байтного массива (HEX + ASCII, 16 байт на строку)
 * @param data Указатель на массив из 256 байт
 */
void protocol_mini_dump_256(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H
