/**
 * @file modbus_slave.h
 * @brief Modbus RTU slave interface for heat pump monitoring and control
 * @version 1.0.0
 * @date 2025
 */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "hal/uart_types.h"

// Modbus communication parameters (defaults)
#define MB_PORT_NUM         UART_NUM_1    // UART1
#define MB_DEV_SPEED        (9600)        // Default baud rate
#define MB_SLAVE_ADDR       (7)           // Default slave address
#define MB_UART_TXD         (25)          // TX pin
#define MB_UART_RXD         (26)          // RX pin
#define MB_UART_RTS         (23)          // RTS pin for RS485 direction control

typedef struct {
    uint32_t baudrate;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_word_length_t data_bits;
    uint8_t slave_addr;
} modbus_serial_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Modbus RTU slave
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_init(void);

/**
 * @brief Start Modbus RTU slave communication
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_start(void);

/**
 * @brief Apply a new Modbus serial configuration (baud rate, frame, slave ID)
 * @param cfg Pointer to configuration structure
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_apply_serial_config(const modbus_serial_config_t *cfg);

/**
 * @brief Get current Modbus serial configuration
 * @param cfg_out Output pointer
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_get_serial_config(modbus_serial_config_t *cfg_out);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_SLAVE_H

