/**
 * @file protocol.c
 * @brief Heat pump protocol communication implementation
 * @version 0.1.0
 * @date 2025
 */

#include "protocol.h"
#include "decoder.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"
#include "stdlib.h"

static const char *TAG = "PROTOCOL";

// Global RX buffer with length metadata
protocol_rx_t g_protocol_rx = {0};

// Global protocol context
protocol_context_t g_protocol_ctx = {0};

// Protocol command templates (based on HeishaMon analysis)
static const uint8_t initial_query[] = {0x31, 0x05, 0x10, 0x01, 0x00, 0x00, 0x00};

const uint8_t panasonic_query[] = {
    0x71, 0x6c, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t optional_pcb_query[] = {
    0xF1, 0x11, 0x01, 0x50, 0x00, 0x00, 0x40, 0xFF,
    0xFF, 0xE5, 0xFF, 0xFF, 0x00, 0xFF, 0xEB, 0xFF,
    0xFF, 0x00, 0x00
};

/**
 * @brief Calculate checksum for data
 * @param data Data to calculate checksum for
 * @param size Data size
 * @return Calculated checksum
 */
uint8_t protocol_calculate_checksum(const uint8_t *data, size_t size) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < size; i++) {
        checksum += data[i];
    }
    return (~checksum + 1);
}

/**
 * @brief Validate received data checksum
 * @param data Received data
 * @param size Data size
 * @return true if checksum is valid
 */
bool protocol_validate_checksum(const uint8_t *data, size_t size) {
    if (size < 2) {
        return false;
    }
    
    uint8_t calculated_checksum = protocol_calculate_checksum(data, size - 1);
    return (calculated_checksum == data[size - 1]);
}

/**
 * @brief Send data via UART with checksum
 * @param data Data to send
 * @param size Data size
 * @return ESP_OK on success
 */
static esp_err_t protocol_uart_send(const uint8_t *data, size_t size) {
    // Calculate checksum
    uint8_t checksum = protocol_calculate_checksum(data, size);
    
    // Send data first
    int bytes_written = uart_write_bytes(PROTOCOL_UART_NUM, data, size);
    if (bytes_written != size) {
        ESP_LOGE(TAG, "Failed to write data bytes: %d/%d", bytes_written, size);
        return ESP_FAIL;
    }
    
    // Send checksum separately
    int checksum_written = uart_write_bytes(PROTOCOL_UART_NUM, &checksum, 1);
    if (checksum_written != 1) {
        ESP_LOGE(TAG, "Failed to write checksum: %d/1", checksum_written);
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Sent %d bytes + checksum 0x%02X", size, checksum);
    return ESP_OK;
}

/**
 * @brief Receive data via UART
 * @param data Buffer to store received data
 * @param max_size Maximum buffer size
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes received
 */
static int protocol_uart_receive(uint8_t *data, size_t max_size) {
    return uart_read_bytes(PROTOCOL_UART_NUM, data, max_size, pdMS_TO_TICKS(PROTOCOL_READ_TIMEOUT_MS));
}

/**
 * @brief Process received data
 * @param data Received data
 * @param size Data size
 * @return ESP_OK on success
 */
static esp_err_t protocol_process_received_data(const uint8_t *data, size_t size) {
    // Validate data size
    if (size < 3) {
        ESP_LOGW(TAG, "Received data too short: %d bytes", size);
        return ESP_ERR_INVALID_SIZE;
    }

    // Validate data size
    if(data[1] + 3 != size) {
        ESP_LOGW(TAG, "Received data size mismatch: %d bytes, expected: %d bytes", size, data[1] + 3);
        return ESP_ERR_INVALID_SIZE;
    }

    // Validate header
    if (data[0] != PROTOCOL_PKT_READ && data[0] != PROTOCOL_PKT_INIT) {
        ESP_LOGW(TAG, "Invalid header: 0x%02X", data[0]);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate checksum
    if (!protocol_validate_checksum(data, size)) {
        ESP_LOGW(TAG, "Checksum validation failed");
        return ESP_ERR_INVALID_CRC;
    }

    ESP_LOGI(TAG, "Received valid data: %d bytes, header: 0x%02X", size, data[0]);

    // Process based on data type
    if (size == PROTOCOL_MAIN_DATA_SIZE && data[3] == PROTOCOL_DATA_MAIN) {
        ESP_LOGI(TAG, "Received main data block");
        
        //do we have valid header and byte 0xc7 is more or equal 3 then assume K&L and more series
        if ((data[0] == PROTOCOL_PKT_READ) && (data[0xC7] >= 3)) g_protocol_ctx.extra_data_block_available = true;
      
        // Decode main data
        esp_err_t decode_ret = decode_main_data();
        if (decode_ret == ESP_OK) {
            ESP_LOGI(TAG, "Main data decoded successfully");
            // Log main data
            log_main_data();
        } else {
            ESP_LOGE(TAG, "Failed to decode main data: %s", esp_err_to_name(decode_ret));
        }
        
    } else if (size == PROTOCOL_EXTRA_DATA_SIZE && data[3] == PROTOCOL_DATA_EXTRA) {
        ESP_LOGI(TAG, "Received extra data block");
        
        // Decode extra data
        esp_err_t decode_ret = decode_extra_data();
        if (decode_ret == ESP_OK) {
            ESP_LOGI(TAG, "Extra data decoded successfully");
            // Log extra data
            log_extra_data();
        } else {
            ESP_LOGE(TAG, "Failed to decode extra data: %s", esp_err_to_name(decode_ret));
        }
        
    } else if (size == PROTOCOL_OPT_DATA_SIZE && data[3] == PROTOCOL_DATA_OPT) {
        ESP_LOGI(TAG, "Received optional data block");
        
        // Decode optional data
        esp_err_t decode_ret = decode_opt_data();
        if (decode_ret == ESP_OK) {
            ESP_LOGI(TAG, "Optional data decoded successfully");
            // Log optional data
            log_opt_data();
        } else {
            ESP_LOGE(TAG, "Failed to decode optional data: %s", esp_err_to_name(decode_ret));
        }
        
    } else {
        ESP_LOGW(TAG, "Unknown data block: size=%d, type=0x%02X", size, data[3]);
    }

    return ESP_OK;
}

/**
 * @brief Protocol communication task
 * @param pvParameters Task parameters
 */
void protocol_task(void *pvParameters) {
    TickType_t last_query_time = 0;
    const TickType_t query_interval = pdMS_TO_TICKS(PROTOCOL_QUERY_INTERVAL_MS);

    ESP_LOGI(TAG, "Protocol task started");
    
    // Send initial query
    protocol_send_initial_query();
    ESP_LOGI(TAG, "Initial query sent");

    while (1) {
        // Process commands from queue
        protocol_cmd_t cmd;
        if (xQueueReceive(g_protocol_ctx.command_queue, &cmd, 0) == pdTRUE) {
            
            // Log command being sent
            ESP_LOGI(TAG, "Sending command: type=0x%02X, size=%d", cmd.data[0], cmd.len);
            
            // Send command
            esp_err_t ret = protocol_uart_send(cmd.data, cmd.len);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Command sent successfully, waiting for response...");
                
                // Wait for response
                int bytes_received = protocol_uart_receive(g_protocol_rx.data, sizeof(g_protocol_rx.data));
                if (bytes_received > 0) {
                    ESP_LOGI(TAG, "Received %d bytes response", bytes_received);
                    g_protocol_rx.len = (size_t)bytes_received;
                    if(protocol_process_received_data(g_protocol_rx.data, bytes_received) != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to process received data");
                    }
                } else {
                        ESP_LOGW(TAG, "No response received for command type: 0x%02X (timeout after %d ms)", cmd.data[0], PROTOCOL_READ_TIMEOUT_MS);
                }
            } else {
                ESP_LOGE(TAG, "Failed to send command type: 0x%02X", cmd.data[0]);
            }
        }

        // Periodic data queries
        TickType_t current_time = xTaskGetTickCount();
        if (current_time - last_query_time >= query_interval) {
            last_query_time = current_time;
            
            // Send periodic queries
            protocol_request_main_data();
            if(g_protocol_ctx.extra_data_block_available) protocol_request_extra_data();
            if(g_protocol_ctx.opt_data_block_available) protocol_request_opt_data();
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent busy waiting
    }

    ESP_LOGE(TAG, "Protocol task stopped");
}

/**
 * @brief Initialize heat pump protocol
 * @return ESP_OK on success
 */
esp_err_t protocol_init(void) {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing heat pump protocol");

    // Initialize decoder
    ret = decoder_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize decoder: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure UART
    uart_config_t uart_config = (uart_config_t) {
        .baud_rate = PROTOCOL_BAUD_RATE,
        .data_bits = PROTOCOL_DATA_BITS,
        .parity = PROTOCOL_PARITY,
        .stop_bits = PROTOCOL_STOP_BITS,
        .flow_ctrl = PROTOCOL_FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT
    };

    // Install UART driver
    ret = uart_driver_install(PROTOCOL_UART_NUM, PROTOCOL_MAIN_DATA_SIZE + 1, 
        PROTOCOL_MAIN_DATA_SIZE + 1, 0, NULL, 0); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "UART driver installed successfully");

    // Configure UART parameters
    ret = uart_param_config(PROTOCOL_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "UART parameters configured: baud=%d, data_bits=%d, parity=%d, stop_bits=%d", 
             PROTOCOL_BAUD_RATE, PROTOCOL_DATA_BITS, PROTOCOL_PARITY, PROTOCOL_STOP_BITS);

    // Set UART pins
    ret = uart_set_pin(PROTOCOL_UART_NUM, PROTOCOL_TX_PIN, PROTOCOL_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "UART pins set: TX=%d, RX=%d", PROTOCOL_TX_PIN, PROTOCOL_RX_PIN);

    // Create command queue
    g_protocol_ctx.command_queue = xQueueCreate(PROTOCOL_QUEUE_SIZE, sizeof(protocol_cmd_t));
    if (g_protocol_ctx.command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
        return ESP_ERR_NO_MEM;
    }

    g_protocol_ctx.opt_data_block_available = PROTOCOL_OPT_AVAILABLE;

    ESP_LOGI(TAG, "Heat pump protocol initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start protocol communication task
 * @return ESP_OK on success
 */
esp_err_t protocol_start(void)
{
    // Create protocol task
    BaseType_t ret = xTaskCreate(protocol_task, "protocol", 4096,
                                 NULL, 5, 
                                 &g_protocol_ctx.protocol_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create protocol task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Heat pump protocol started successfully");
    return ESP_OK;
}

/**
 * @brief Send command to heat pump
 * @param cmd Command to send
 * @return ESP_OK on success
 */
esp_err_t protocol_send_command(const protocol_cmd_t *cmd) {
    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xQueueSend(g_protocol_ctx.command_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to send command to queue");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

/**
 * @brief Send initial query to heat pump
 * @return ESP_OK on success
 */
esp_err_t protocol_send_initial_query(void) {
    protocol_cmd_t cmd = {0};
    
    cmd.len = sizeof(initial_query);
    
    memcpy(cmd.data, initial_query, sizeof(initial_query));
    
    ESP_LOGD(TAG, "Sending initial query");
    return protocol_send_command(&cmd);
}

/**
 * @brief Request main data from heat pump
 * @return ESP_OK on success
 */
esp_err_t protocol_request_main_data(void) {
    protocol_cmd_t cmd = {0};
    
    cmd.len = sizeof(panasonic_query);
    
    memcpy(cmd.data, panasonic_query, sizeof(panasonic_query));
    
    ESP_LOGD(TAG, "Requesting main data");
    return protocol_send_command(&cmd);
}

/**
 * @brief Request extra data from heat pump
 * @return ESP_OK on success
 */
esp_err_t protocol_request_extra_data(void)
{
    protocol_cmd_t cmd = {0};
    
    cmd.len = sizeof(panasonic_query);
    
    memcpy(cmd.data, panasonic_query, sizeof(panasonic_query));
    cmd.data[3] = PROTOCOL_DATA_EXTRA; // Set data type to extra
    
    ESP_LOGD(TAG, "Requesting extra data");
    return protocol_send_command(&cmd);
}

/**
 * @brief Request optional data from heat pump
 * @return ESP_OK on success
 */
esp_err_t protocol_request_opt_data(void)
{
    protocol_cmd_t cmd = {0};
    
    cmd.len = sizeof(optional_pcb_query);
    
    memcpy(cmd.data, optional_pcb_query, sizeof(optional_pcb_query));
    
    ESP_LOGD(TAG, "Requesting optional data");
    return protocol_send_command(&cmd);
}
