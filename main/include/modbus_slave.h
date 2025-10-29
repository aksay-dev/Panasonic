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
 * @brief Stop Modbus RTU slave communication
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_stop(void);

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

