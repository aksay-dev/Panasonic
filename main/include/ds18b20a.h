/**
 * @file ds18b20.h
 * @brief DS18B20 temperature sensor driver interface
 * @version 1.0.0
 * @date 2025
 * 
 * This module handles DS18B20 temperature sensor connected via 1-Wire bus.
 * Temperature values are read with 12-bit resolution and updated every second.
 * Values are stored in Modbus input register in format: temperature in °C * 100.
 */

#ifndef DS18B20_H
#define DS18B20_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DS18B20 GPIO pin
 */
#define DS18B20_GPIO 22

/**
 * @brief Update interval in milliseconds
 */
#define DS18B20_UPDATE_INTERVAL_MS 1000

/**
 * @brief Maximum supported DS18B20 sensors on the bus
 */
#define DS18B20_MAX_SENSORS 8

/**
 * @brief DS18B20 resolution (9-12 bits)
 * 12-bit resolution provides 0.0625°C precision
 */
#define DS18B20_RESOLUTION_BITS 12

/**
 * @brief Initialize DS18B20 sensor
 * @return ESP_OK on success
 */
esp_err_t ds18b20_init(void);

/**
 * @brief Start DS18B20 reading task
 * This task reads temperature values with 12-bit resolution
 * and updates Modbus register every second
 * @return ESP_OK on success
 */
esp_err_t ds18b20_start(void);

/**
 * @brief Stop DS18B20 reading task
 * @return ESP_OK on success
 */
esp_err_t ds18b20_stop(void);

/**
 * @brief Get current cached temperature value
 * @param temperature Pointer to store temperature in degrees Celsius * 100
 *                     (e.g., 2550 for 25.5°C)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if sensor not initialized
 */
esp_err_t ds18b20_get_cached_temperature(int16_t *temperature);

#ifdef __cplusplus
}
#endif

#endif // DS18B20_H

