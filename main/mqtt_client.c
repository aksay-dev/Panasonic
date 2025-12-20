/**
 * @file mqtt_client.c
 * @brief MQTT client implementation for publishing heat pump data
 * @version 1.0.0
 * @date 2025
 * 
 * Note: This module requires WiFi to be connected before MQTT can work.
 * The MQTT client will attempt to connect, but will fail if WiFi is not available.
 */

// Include ESP-IDF MQTT component first
#include "mqtt_client.h"
#include "include/mqtt_pub.h"
#include "include/modbus_params.h"
#include "include/project_config.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "MQTT_CLIENT";

// MQTT client handle
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Connection status
static bool mqtt_connected = false;

// MQTT configuration (using values from project_config.h)
#define MQTT_BROKER_URL CONFIG_MQTT_BROKER_URL_DEFAULT
#define MQTT_TOPIC_BASE CONFIG_MQTT_TOPIC_BASE_DEFAULT
#define MQTT_CLIENT_ID_MAX_LEN CONFIG_MQTT_CLIENT_ID_MAX_LEN

// Remove unused define
// #define MQTT_BROKER_PORT 1883

// Task handle for publishing (removed - publishing now happens after decoding)

/**
 * @brief Generate unique client ID from MAC address
 */
static void mqtt_generate_client_id(char *client_id, size_t len) {
    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret == ESP_OK) {
        snprintf(client_id, len, "panasonic_%02x%02x%02x%02x%02x%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        snprintf(client_id, len, "panasonic_%u", (unsigned int)esp_random());
    }
}

/**
 * @brief MQTT event handler
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            mqtt_connected = true;
            // Subscribe to any topics if needed
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            mqtt_connected = false;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT subscribed, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT unsubscribed, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            // ESP_LOGI(TAG, "MQTT published, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT data received, topic=%.*s, data=%.*s",
                     event->topic_len, event->topic, event->data_len, event->data);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            if (event->error_handle) {
                ESP_LOGE(TAG, "Error type: %d", event->error_handle->error_type);
            }
            mqtt_connected = false;
            break;

        default:
            ESP_LOGD(TAG, "MQTT event: %d", event_id);
            break;
    }
}

// Periodic publish task removed - publishing now happens after data decoding in protocol.c

/**
 * @brief Initialize MQTT client
 */
esp_err_t mqtt_client_init(void) {
    if (mqtt_client != NULL) {
        ESP_LOGW(TAG, "MQTT client already initialized");
        return ESP_OK;
    }

    // Generate unique client ID
    char client_id[MQTT_CLIENT_ID_MAX_LEN];
    mqtt_generate_client_id(client_id, sizeof(client_id));

    // MQTT broker configuration
    esp_mqtt_client_config_t mqtt_cfg = {0};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URL;
    mqtt_cfg.credentials.client_id = client_id;
    mqtt_cfg.credentials.username = CONFIG_MQTT_USERNAME_DEFAULT;
    mqtt_cfg.credentials.authentication.password = CONFIG_MQTT_PASSWORD_DEFAULT;
    mqtt_cfg.session.keepalive = CONFIG_MQTT_KEEPALIVE_SEC;

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    // Register event handler
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    ESP_LOGI(TAG, "MQTT client initialized with ID: %s", client_id);
    return ESP_OK;
}

/**
 * @brief Start MQTT client
 */
esp_err_t mqtt_client_start(void) {
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = esp_mqtt_client_start(mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        return ret;
    }

    // Publish task removed - publishing now happens after data decoding in protocol.c
    ESP_LOGI(TAG, "MQTT client started (publishing on decode events)");
    return ESP_OK;
}

/**
 * @brief Stop MQTT client
 */
esp_err_t mqtt_client_stop(void) {
    if (mqtt_client == NULL) {
        return ESP_OK;
    }

    // Publish task removed - no need to delete it

    esp_err_t ret = esp_mqtt_client_stop(mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop MQTT client: %s", esp_err_to_name(ret));
    }

    mqtt_connected = false;
    ESP_LOGI(TAG, "MQTT client stopped");
    return ret;
}

/**
 * @brief Check if MQTT client is connected
 */
bool mqtt_client_is_connected(void) {
    return mqtt_connected;
}

/**
 * @brief Publish a single value to MQTT
 */
static esp_err_t mqtt_publish_value(const char *topic, const char *value) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, value, 0, CONFIG_MQTT_QOS_LEVEL, CONFIG_MQTT_RETAIN);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to %s", topic);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Publish heat pump data to MQTT
 */
esp_err_t mqtt_client_publish_data(void) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    char topic[128];
    char value[32];
    esp_err_t ret = ESP_OK;

    // Publish basic temperatures
    snprintf(topic, sizeof(topic), "%s/temperature/main_inlet", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%.2f", mb_input_registers[MB_INPUT_MAIN_INLET_TEMP] / 100.0f);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/temperature/main_outlet", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%.2f", mb_input_registers[MB_INPUT_MAIN_OUTLET_TEMP] / 100.0f);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/temperature/main_target", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_MAIN_TARGET_TEMP]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/temperature/dhw", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_DHW_TEMP]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/temperature/dhw_target", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_DHW_TARGET_TEMP]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/temperature/outside", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_OUTSIDE_TEMP]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/temperature/room_thermostat", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_ROOM_THERMOSTAT_TEMP]);
    mqtt_publish_value(topic, value);

    // Publish power data
    snprintf(topic, sizeof(topic), "%s/power/heat_production", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_HEAT_POWER_PRODUCTION]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/power/heat_consumption", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/power/cool_production", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_COOL_POWER_PRODUCTION]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/power/cool_consumption", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/power/dhw_production", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_DHW_POWER_PRODUCTION]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/power/dhw_consumption", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION]);
    mqtt_publish_value(topic, value);

    // Publish states
    snprintf(topic, sizeof(topic), "%s/state/heatpump", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_HEATPUMP_STATE]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/state/operation_mode", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_OPERATING_MODE_STATE]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/state/holiday_mode", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_HOLIDAY_MODE_STATE]);
    mqtt_publish_value(topic, value);

    // Publish technical parameters
    snprintf(topic, sizeof(topic), "%s/technical/compressor_freq", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_COMPRESSOR_FREQ]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/technical/pump_flow", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_PUMP_FLOW]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/technical/operations_hours", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_OPERATIONS_HOURS]);
    mqtt_publish_value(topic, value);

    // Publish error info
    snprintf(topic, sizeof(topic), "%s/error/type", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_ERROR_TYPE]);
    mqtt_publish_value(topic, value);

    snprintf(topic, sizeof(topic), "%s/error/number", MQTT_TOPIC_BASE);
    snprintf(value, sizeof(value), "%d", mb_input_registers[MB_INPUT_ERROR_NUMBER]);
    mqtt_publish_value(topic, value);

    return ret;
}

