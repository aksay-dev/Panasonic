/**
 * @file decoder.h
 * @brief Heat pump data decoder header
 * @version 1.0.0
 * @date 2024
 * 
 * Based on HeishaMon decode.h/c implementation
 */

#ifndef DECODER_NEW_H
#define DECODER_NEW_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decode main heat pump data
 * @return ESP_OK on success
 */
esp_err_t decode_main_data(void);

/**
 * @brief Decode extra heat pump data
 * @return ESP_OK on success
 */
esp_err_t decode_extra_data(void);

/**
 * @brief Decode optional PCB data
 * @return ESP_OK on success
 */
esp_err_t decode_opt_data(void);

void log_main_data(void);
void log_extra_data(void);
void log_opt_data(void);

#ifdef __cplusplus
}
#endif

#endif // DECODER_NEW_H
