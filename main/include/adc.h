/**
 * @file adc.h
 * @brief ADC analog input handling with moving average filter
 * @version 1.0.0
 * @date 2025
 * 
 * This module handles three ADC channels on GPIO32, GPIO34, GPIO35.
 * Values are filtered using a moving average filter and updated every second.
 * Filtered values are stored in Modbus input registers.
 */

#ifndef ADC_H
#define ADC_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Number of ADC channels
 */
#define ADC_CHANNEL_COUNT 3

/**
 * @brief ADC channel GPIO pins
 */
#define ADC_CH0_GPIO 32  // GPIO32
#define ADC_CH1_GPIO 34  // GPIO34
#define ADC_CH2_GPIO 35  // GPIO35

/**
 * @brief Moving average filter size
 * Larger values provide more smoothing but slower response
 */
#define ADC_FILTER_SIZE 10

/**
 * @brief Update interval in milliseconds
 */
#define ADC_UPDATE_INTERVAL_MS 100

/**
 * @brief Voltage divider configuration for NTC sensors
 * 
 * Voltage divider: 3V3 -> [R_top] -> ADC pin -> [NTC] -> GND
 * 
 * These values can be adjusted to match your specific NTC thermistor:
 * - ADC_NTC_VOLTAGE_DIVIDER_TOP_OHM: Upper resistor value (typically 10 kOhm)
 * - ADC_NTC_R0_OHM: NTC resistance at 25°C (check NTC datasheet, typically 10 kOhm)
 * - ADC_NTC_BETA_COEFFICIENT: Beta coefficient (check NTC datasheet, typically 3950 for 10k NTC)
 */
#define ADC_NTC_VOLTAGE_DIVIDER_TOP_OHM  10000  // Upper resistor in voltage divider (10 kOhm)
#define ADC_NTC_R0_OHM                   10000  // NTC resistance at 25°C (10 kOhm)
#define ADC_NTC_BETA_COEFFICIENT         3950   // Beta coefficient for NTC (typical value)
#define ADC_NTC_T0_KELVIN                298.15 // Reference temperature in Kelvin (25°C)
#define ADC_ADC_REFERENCE_VOLTAGE_MV     3300   // ADC reference voltage in millivolts (3.3V)
#define ADC_ADC_MAX_VALUE                4095   // Maximum ADC value (12-bit)

/**
 * @brief Initialize ADC module
 * @return ESP_OK on success
 */
esp_err_t adc_init(void);

/**
 * @brief Start ADC reading task
 * This task reads ADC values, applies moving average filter,
 * and updates Modbus registers every second
 * @return ESP_OK on success
 */
esp_err_t adc_start(void);

/**
 * @brief Stop ADC reading task
 * @return ESP_OK on success
 */
esp_err_t adc_stop(void);

/**
 * @brief Get current filtered ADC value for a channel
 * @param channel Channel number (0-2)
 * @param value Pointer to store the filtered value (0-4095 for 12-bit ADC)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if channel is invalid
 */
esp_err_t adc_get_filtered_value(uint8_t channel, uint16_t *value);

/**
 * @brief Get raw (unfiltered) ADC value for a channel
 * @param channel Channel number (0-2)
 * @param value Pointer to store the raw value (0-4095 for 12-bit ADC)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if channel is invalid
 */
esp_err_t adc_get_raw_value(uint8_t channel, uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif // ADC_H

