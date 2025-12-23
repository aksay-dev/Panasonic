/**
 * @file http_server.h
 * @brief HTTP server for viewing heat pump parameters
 * @version 1.0.0
 * @date 2025
 */

#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize HTTP server
 * @return ESP_OK on success
 */
esp_err_t http_server_init(void);

/**
 * @brief Start HTTP server
 * @return ESP_OK on success
 */
esp_err_t http_server_start(void);

/**
 * @brief Stop HTTP server
 * @return ESP_OK on success
 */
esp_err_t http_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif // HTTP_SERVER_H

