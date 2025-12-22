/**
 * @file mqtt_pub.h
 * @brief MQTT client for publishing heat pump data
 * @version 1.0.0
 * @date 2025
 */

#ifndef MQTT_PUB_H
#define MQTT_PUB_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MQTT client
 * @return ESP_OK on success
 */
esp_err_t mqtt_client_init(void);

/**
 * @brief Start MQTT client
 * @return ESP_OK on success
 */
esp_err_t mqtt_client_start(void);

/**
 * @brief Stop MQTT client
 * @return ESP_OK on success
 */
esp_err_t mqtt_client_stop(void);

/**
 * @brief Check if MQTT client is connected
 * @return true if connected, false otherwise
 */
bool mqtt_client_is_connected(void);

/**
 * @brief Publish heat pump data to MQTT
 * This function reads data from Modbus registers and publishes them
 * @return ESP_OK on success
 */
esp_err_t mqtt_client_publish_data(void);

typedef struct {
    uint16_t reg_addr;
    const char *name;
    const char *subtopic;
} mqtt_name_t;

#ifdef __cplusplus
}
#endif

#endif // MQTT_PUB_H

