/**
 * @file project_config.h
 * @brief Project configuration - WiFi and MQTT settings
 * @version 1.0.0
 * @date 2025
 * 
 * This file contains default configuration values for WiFi and MQTT.
 * These values can be overridden via NVS or Modbus registers.
 */

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// WiFi Configuration
// ============================================================================

/**
 * @brief Default WiFi SSID
 * Can be changed via NVS or Modbus register
 */
#define CONFIG_WIFI_SSID_DEFAULT "WiFi"

/**
 * @brief Default WiFi Password
 * Can be changed via NVS or Modbus register
 */
#define CONFIG_WIFI_PASSWORD_DEFAULT "boss29586"
#define CONFIG_WIFI_RECONNECT 30000

// ============================================================================
// MQTT Configuration
// ============================================================================

/**
 * @brief Default MQTT broker URL
 * Format: "mqtt://hostname:port" or "mqtts://hostname:port" for SSL
 * Can be changed via NVS or Modbus register
 */
#define CONFIG_MQTT_BROKER_URL_DEFAULT "mqtt://m8.wqtt.ru:16778"

#define CONFIG_MQTT_USERNAME_DEFAULT "u_1LBQB6"
#define CONFIG_MQTT_PASSWORD_DEFAULT "UF0Pi2i7"


/**
 * @brief Default MQTT topic base
 * All topics will be prefixed with this base path
 * Format: "hp" -> topics like "hp/temperature/main_inlet"
 */
#define CONFIG_MQTT_TOPIC_BASE_DEFAULT "hp"

/**
 * @brief Maximum length for MQTT client ID
 */
#define CONFIG_MQTT_CLIENT_ID_MAX_LEN 32

/**
 * @brief MQTT connection timeout in seconds
 */
#define CONFIG_MQTT_CONNECT_TIMEOUT_SEC 10

/**
 * @brief MQTT keepalive interval in seconds
 */
#define CONFIG_MQTT_KEEPALIVE_SEC 60

/**
 * @brief MQTT QoS level (0, 1, or 2)
 */
#define CONFIG_MQTT_QOS_LEVEL 1

/**
 * @brief MQTT retain flag (0 or 1)
 */
#define CONFIG_MQTT_RETAIN 0

#ifdef __cplusplus
}
#endif

#endif // PROJECT_CONFIG_H

