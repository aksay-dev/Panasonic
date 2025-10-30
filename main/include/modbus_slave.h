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
#include "driver/uart.h"

// Modbus communication parameters
#define MB_PORT_NUM         UART_NUM_1    // UART1
#define MB_DEV_SPEED        (9600)        // Baud rate
#define MB_SLAVE_ADDR       (7)           // Slave address
#define MB_UART_TXD         (25)          // TX pin
#define MB_UART_RXD         (26)          // RX pin
#define MB_UART_RTS         (23)          // RTS pin for RS485 direction control

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
 * @brief Update input registers from heat pump decoded data
 * Should be called periodically to sync Modbus registers with heat pump state
 * @return ESP_OK on success
 */
esp_err_t modbus_update_input_registers(void);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_SLAVE_H

