/**
 * @file wifi_connect.h
 * @brief WiFi connection management
 * @version 1.0.0
 * @date 2025
 */

#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_SSID_MAX_LEN 32
#define WIFI_PASSWORD_MAX_LEN 64

/**
 * @brief WiFi credentials structure (to avoid conflict with esp_wifi wifi_config_t)
 */
typedef struct {
    char ssid[WIFI_SSID_MAX_LEN];
    char password[WIFI_PASSWORD_MAX_LEN];
} wifi_credentials_t;

/**
 * @brief Initialize WiFi connection module
 * @return ESP_OK on success
 */
esp_err_t wifi_connect_init(void);

/**
 * @brief Start WiFi connection
 * @return ESP_OK on success
 */
esp_err_t wifi_connect_start(void);

/**
 * @brief Stop WiFi connection
 * @return ESP_OK on success
 */
esp_err_t wifi_connect_stop(void);

/**
 * @brief Check if WiFi is connected
 * @return true if connected, false otherwise
 */
bool wifi_connect_is_connected(void);

/**
 * @brief Get current IP address
 * @param ip_str Buffer to store IP address string (at least 16 bytes)
 * @return ESP_OK on success
 */
esp_err_t wifi_connect_get_ip(char *ip_str, size_t len);

/**
 * @brief Set WiFi credentials
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success
 */
esp_err_t wifi_connect_set_credentials(const char *ssid, const char *password);

/**
 * @brief Load WiFi credentials from NVS
 * @param config Pointer to wifi_credentials_t structure to fill
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not configured
 */
esp_err_t wifi_connect_load_config(wifi_credentials_t *config);

/**
 * @brief Save WiFi credentials to NVS
 * @param config Pointer to wifi_credentials_t structure
 * @return ESP_OK on success
 */
esp_err_t wifi_connect_save_config(const wifi_credentials_t *config);

#ifdef __cplusplus
}
#endif

#endif // WIFI_CONNECT_H

