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

#define MQTT_SUB_SYS "sys"
#define MQTT_SUB_TEMP "temp"
#define MQTT_SUB_FLOW "flow"
#define MQTT_SUB_STATE "state"
#define MQTT_SUB_POWER "power"
#define MQTT_SUB_FREQ "freq"
#define MQTT_SUB_HOUR "hour"
#define MQTT_SUB_COUNT "count"
#define MQTT_SUB_SPEED "speed"
#define MQTT_SUB_PRESS "press"
#define MQTT_SUB_CURRENT "current"
#define MQTT_SUB_DUTY "duty"
#define MQTT_SUB_ERROR "error"

static const mqtt_name_t mqtt_names[] = {
    {MB_INPUT_STATUS, "status", MQTT_SUB_SYS},
    {MB_INPUT_EXTENDED_DATA, "extended_data", MQTT_SUB_SYS},
    {MB_INPUT_MAIN_INLET_TEMP, "main_inlet", MQTT_SUB_TEMP},
    {MB_INPUT_MAIN_OUTLET_TEMP, "main_outlet", MQTT_SUB_TEMP},
    {MB_INPUT_MAIN_TARGET_TEMP, "main_target", MQTT_SUB_TEMP},
    {MB_INPUT_DHW_TEMP, "dhw", MQTT_SUB_TEMP},
    {MB_INPUT_DHW_TARGET_TEMP, "dhw_target", MQTT_SUB_TEMP},
    {MB_INPUT_OUTSIDE_TEMP, "outside", MQTT_SUB_TEMP},
    {MB_INPUT_ROOM_THERMOSTAT_TEMP, "room_thermostat", MQTT_SUB_TEMP},
    {MB_INPUT_BUFFER_TEMP, "buffer", MQTT_SUB_TEMP},
    {MB_INPUT_SOLAR_TEMP, "solar", MQTT_SUB_TEMP},
    {MB_INPUT_POOL_TEMP, "pool", MQTT_SUB_TEMP},
    {MB_INPUT_MAIN_HEX_OUTLET_TEMP, "main_hex_outlet", MQTT_SUB_TEMP},
    {MB_INPUT_DISCHARGE_TEMP, "discharge", MQTT_SUB_TEMP},
    {MB_INPUT_INSIDE_PIPE_TEMP, "inside_pipe", MQTT_SUB_TEMP},
    {MB_INPUT_DEFROST_TEMP, "defrost", MQTT_SUB_TEMP},
    {MB_INPUT_EVA_OUTLET_TEMP, "eva_outlet", MQTT_SUB_TEMP},
    {MB_INPUT_BYPASS_OUTLET_TEMP, "bypass_outlet", MQTT_SUB_TEMP},
    {MB_INPUT_IPM_TEMP, "ipm", MQTT_SUB_TEMP},
    {MB_INPUT_OUTSIDE_PIPE_TEMP, "outside_pipe", MQTT_SUB_TEMP},
    {MB_INPUT_Z1_ROOM_TEMP, "z1_room", MQTT_SUB_TEMP},
    {MB_INPUT_Z2_ROOM_TEMP, "z2_room", MQTT_SUB_TEMP},
    {MB_INPUT_Z1_WATER_TEMP, "z1_water", MQTT_SUB_TEMP},
    {MB_INPUT_Z2_WATER_TEMP, "z2_water", MQTT_SUB_TEMP},
    {MB_INPUT_Z1_WATER_TARGET_TEMP, "z1_water_target", MQTT_SUB_TEMP},
    {MB_INPUT_Z2_WATER_TARGET_TEMP, "z2_water_target", MQTT_SUB_TEMP},
    {MB_INPUT_SECOND_INLET_TEMP, "second_inlet", MQTT_SUB_TEMP},
    {MB_INPUT_ECONOMIZER_OUTLET_TEMP, "economizer_outlet", MQTT_SUB_TEMP},
    {MB_INPUT_SECOND_ROOM_THERMO_TEMP, "second_room_thermo", MQTT_SUB_TEMP},
    {MB_INPUT_Z1_HEAT_REQUEST_TEMP, "z1_heat_request", MQTT_SUB_TEMP},
    {MB_INPUT_Z1_COOL_REQUEST_TEMP, "z1_cool_request", MQTT_SUB_TEMP},
    {MB_INPUT_Z2_HEAT_REQUEST_TEMP, "z2_heat_request", MQTT_SUB_TEMP},
    {MB_INPUT_Z2_COOL_REQUEST_TEMP, "z2_cool_request", MQTT_SUB_TEMP},
    {MB_INPUT_HEAT_POWER_PRODUCTION, "heat_prod", MQTT_SUB_POWER},
    {MB_INPUT_HEAT_POWER_CONSUMPTION, "heat_cons", MQTT_SUB_POWER},
    {MB_INPUT_COOL_POWER_PRODUCTION, "cool_prod", MQTT_SUB_POWER},
    {MB_INPUT_COOL_POWER_CONSUMPTION, "cool_cons", MQTT_SUB_POWER},
    {MB_INPUT_DHW_POWER_PRODUCTION, "dhw_prod", MQTT_SUB_POWER},
    {MB_INPUT_DHW_POWER_CONSUMPTION, "dhw_cons", MQTT_SUB_POWER},
    {MB_INPUT_COMPRESSOR_FREQ, "compressor", MQTT_SUB_FREQ},
    {MB_INPUT_PUMP_FLOW, "pump", MQTT_SUB_FLOW},
    {MB_INPUT_OPERATIONS_HOURS, "operations", MQTT_SUB_HOUR},
    {MB_INPUT_OPERATIONS_COUNTER, "operations", MQTT_SUB_COUNT},
    {MB_INPUT_FAN1_MOTOR_SPEED, "fan1", MQTT_SUB_SPEED},
    {MB_INPUT_FAN2_MOTOR_SPEED, "fan2", MQTT_SUB_SPEED},
    {MB_INPUT_HIGH_PRESSURE, "high", MQTT_SUB_PRESS},
    {MB_INPUT_PUMP_SPEED, "pump", MQTT_SUB_SPEED},
    {MB_INPUT_LOW_PRESSURE, "low", MQTT_SUB_PRESS},
    {MB_INPUT_COMPRESSOR_CURRENT, "compressor", MQTT_SUB_CURRENT},
    {MB_INPUT_PUMP_DUTY, "pump", MQTT_SUB_DUTY},
    {MB_INPUT_MAX_PUMP_DUTY, "max_pump", MQTT_SUB_SYS},
    {MB_INPUT_HEATPUMP_STATE, "heatpump_state", MQTT_SUB_STATE},
    {MB_INPUT_FORCE_DHW_STATE, "force_dhw", MQTT_SUB_STATE},
    {MB_INPUT_OPERATING_MODE_STATE, "operating", MQTT_SUB_STATE},
    {MB_INPUT_QUIET_MODE_SCHEDULE, "quiet_schedule", MQTT_SUB_STATE},
    {MB_INPUT_POWERFUL_MODE_TIME, "powerful_time", MQTT_SUB_HOUR},
    {MB_INPUT_QUIET_MODE_LEVEL, "quiet_level", MQTT_SUB_STATE},
    {MB_INPUT_HOLIDAY_MODE_STATE, "holiday", MQTT_SUB_STATE},
    {MB_INPUT_THREE_WAY_VALVE_STATE, "three_way_valve", MQTT_SUB_STATE},
    {MB_INPUT_DEFROSTING_STATE, "defrosting", MQTT_SUB_STATE},
    {MB_INPUT_MAIN_SCHEDULE_STATE, "main_schedule", MQTT_SUB_STATE},
    {MB_INPUT_ZONES_STATE, "zones", MQTT_SUB_STATE},
    {MB_INPUT_DHW_HEATER_STATE, "dhw_heater", MQTT_SUB_STATE},
    {MB_INPUT_ROOM_HEATER_STATE, "room_heater", MQTT_SUB_STATE},
    {MB_INPUT_INTERNAL_HEATER_STATE, "internal_heater", MQTT_SUB_STATE},
    {MB_INPUT_EXTERNAL_HEATER_STATE, "external_heater", MQTT_SUB_STATE},
    {MB_INPUT_FORCE_HEATER_STATE, "force_heater", MQTT_SUB_STATE},
    {MB_INPUT_STERILIZATION_STATE, "sterilization", MQTT_SUB_STATE},
    {MB_INPUT_STERILIZATION_TEMP, "sterilization_temp", MQTT_SUB_TEMP},
    {MB_INPUT_STERILIZATION_MAX_TIME, "sterilization_max_time", MQTT_SUB_HOUR},
    {MB_INPUT_DHW_HEAT_DELTA, "dhw_heat_delta", MQTT_SUB_TEMP},
    {MB_INPUT_HEAT_DELTA, "heat_delta", MQTT_SUB_TEMP},
    {MB_INPUT_COOL_DELTA, "cool_delta", MQTT_SUB_TEMP},
    {MB_INPUT_DHW_HOLIDAY_SHIFT_TEMP, "dhw_holiday_shift", MQTT_SUB_TEMP},
    {MB_INPUT_ROOM_HOLIDAY_SHIFT_TEMP, "room_holiday_shift", MQTT_SUB_TEMP},
    {MB_INPUT_BUFFER_TANK_DELTA, "buffer_delta", MQTT_SUB_TEMP},
    {MB_INPUT_HEATING_MODE, "heating_mode", MQTT_SUB_STATE},
    {MB_INPUT_HEATING_OFF_OUTDOOR_TEMP, "heating_off_outdoor", MQTT_SUB_TEMP},
    {MB_INPUT_HEATER_ON_OUTDOOR_TEMP, "heater_on_outdoor", MQTT_SUB_TEMP},
    {MB_INPUT_HEAT_TO_COOL_TEMP, "heat_to_cool", MQTT_SUB_TEMP},
    {MB_INPUT_COOL_TO_HEAT_TEMP, "cool_to_heat", MQTT_SUB_TEMP},
    {MB_INPUT_COOLING_MODE, "cooling_mode", MQTT_SUB_STATE},
    {MB_INPUT_BUFFER_INSTALLED, "buffer_installed", MQTT_SUB_SYS},
    {MB_INPUT_DHW_INSTALLED, "dhw_installed", MQTT_SUB_SYS},
    {MB_INPUT_SOLAR_MODE, "solar", MQTT_SUB_STATE},
    {MB_INPUT_SOLAR_ON_DELTA, "solar_on_delta", MQTT_SUB_TEMP},
    {MB_INPUT_SOLAR_OFF_DELTA, "solar_off_delta", MQTT_SUB_TEMP},
    {MB_INPUT_SOLAR_FROST_PROTECTION, "solar_frost_protection", MQTT_SUB_TEMP},
    {MB_INPUT_SOLAR_HIGH_LIMIT, "solar_high_limit", MQTT_SUB_TEMP},
    {MB_INPUT_PUMP_FLOWRATE_MODE, "pump_flowrate", MQTT_SUB_STATE},
    {MB_INPUT_LIQUID_TYPE, "liquid_type", MQTT_SUB_SYS},
    {MB_INPUT_ALT_EXTERNAL_SENSOR, "alt_external_sensor", MQTT_SUB_SYS},
    {MB_INPUT_ANTI_FREEZE_MODE, "anti_freeze", MQTT_SUB_STATE},
    {MB_INPUT_OPTIONAL_PCB, "optional_pcb", MQTT_SUB_SYS},
    {MB_INPUT_Z1_SENSOR_SETTINGS, "z1_sensor_settings", MQTT_SUB_SYS},
    {MB_INPUT_Z2_SENSOR_SETTINGS, "z2_sensor_settings", MQTT_SUB_SYS},
    {MB_INPUT_EXTERNAL_PAD_HEATER, "external_pad_heater", MQTT_SUB_STATE},
    {MB_INPUT_WATER_PRESSURE, "water_pressure", MQTT_SUB_PRESS},
    {MB_INPUT_EXTERNAL_CONTROL, "external_control", MQTT_SUB_STATE},
    {MB_INPUT_EXTERNAL_HEAT_COOL_CONTROL, "external_heat_cool", MQTT_SUB_STATE},
    {MB_INPUT_EXTERNAL_ERROR_SIGNAL, "external_error", MQTT_SUB_STATE},
    {MB_INPUT_EXTERNAL_COMPRESSOR_CONTROL, "external_compressor", MQTT_SUB_STATE},
    {MB_INPUT_Z2_PUMP_STATE, "z2_pump", MQTT_SUB_STATE},
    {MB_INPUT_Z1_PUMP_STATE, "z1_pump", MQTT_SUB_STATE},
    {MB_INPUT_TWO_WAY_VALVE_STATE, "two_way_valve", MQTT_SUB_STATE},
    {MB_INPUT_THREE_WAY_VALVE_STATE2, "three_way_valve2", MQTT_SUB_STATE},
    {MB_INPUT_Z1_VALVE_PID, "z1_valve_pid", MQTT_SUB_SYS},
    {MB_INPUT_Z2_VALVE_PID, "z2_valve_pid", MQTT_SUB_SYS},
    {MB_INPUT_BIVALENT_CONTROL, "bivalent_control", MQTT_SUB_STATE},
    {MB_INPUT_BIVALENT_MODE, "bivalent_mode", MQTT_SUB_STATE},
    {MB_INPUT_BIVALENT_START_TEMP, "bivalent_start_temp", MQTT_SUB_TEMP},
    {MB_INPUT_BIVALENT_ADVANCED_HEAT, "bivalent_adv_heat", MQTT_SUB_STATE},
    {MB_INPUT_BIVALENT_ADVANCED_DHW, "bivalent_adv_dhw", MQTT_SUB_STATE},
    {MB_INPUT_BIVALENT_ADVANCED_START_TEMP, "bivalent_adv_start", MQTT_SUB_TEMP},
    {MB_INPUT_BIVALENT_ADVANCED_STOP_TEMP, "bivalent_adv_stop", MQTT_SUB_TEMP},
    {MB_INPUT_BIVALENT_ADVANCED_START_DELAY, "bivalent_adv_start_delay", MQTT_SUB_HOUR},
    {MB_INPUT_BIVALENT_ADVANCED_STOP_DELAY, "bivalent_adv_stop_delay", MQTT_SUB_HOUR},
    {MB_INPUT_BIVALENT_ADVANCED_DHW_DELAY, "bivalent_adv_dhw_delay", MQTT_SUB_HOUR},
    {MB_INPUT_HEATER_DELAY_TIME, "heater_delay_time", MQTT_SUB_HOUR},
    {MB_INPUT_HEATER_START_DELTA, "heater_start_delta", MQTT_SUB_TEMP},
    {MB_INPUT_HEATER_STOP_DELTA, "heater_stop_delta", MQTT_SUB_TEMP},
    {MB_INPUT_ERROR_TYPE, "error_type", MQTT_SUB_ERROR},
    {MB_INPUT_ERROR_NUMBER, "error_number", MQTT_SUB_ERROR},
    {MB_INPUT_ROOM_HEATER_OPS_HOURS, "room_heater_ops_hours", MQTT_SUB_HOUR},
    {MB_INPUT_DHW_HEATER_OPS_HOURS, "dhw_heater_ops_hours", MQTT_SUB_HOUR},
    {MB_INPUT_Z1_WATER_PUMP, "z1_water_pump", MQTT_SUB_STATE},
    {MB_INPUT_Z1_MIXING_VALVE, "z1_mixing_valve", MQTT_SUB_STATE},
    {MB_INPUT_Z2_WATER_PUMP, "z2_water_pump", MQTT_SUB_STATE},
    {MB_INPUT_Z2_MIXING_VALVE, "z2_mixing_valve", MQTT_SUB_STATE},
    {MB_INPUT_POOL_WATER_PUMP, "pool_water_pump", MQTT_SUB_STATE},
    {MB_INPUT_SOLAR_WATER_PUMP, "solar_water_pump", MQTT_SUB_STATE},
    {MB_INPUT_ALARM_STATE, "alarm_state", MQTT_SUB_STATE},
    {MB_INPUT_ADC_AIN, "adc_ain", MQTT_SUB_SYS},
    {MB_INPUT_ADC_NTC1, "adc_ntc1", MQTT_SUB_SYS},
    {MB_INPUT_ADC_NTC2, "adc_ntc2", MQTT_SUB_SYS},
    {MB_INPUT_DS18B20_TEMP, "ds18b20", MQTT_SUB_TEMP}
};


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
    const char *template_topic = "%s/%s/%s";
    
    for(size_t i = 0; i < sizeof(mqtt_names) / sizeof(mqtt_name_t); i++) {
        snprintf(topic, sizeof(topic), template_topic, MQTT_TOPIC_BASE, mqtt_names[i].subtopic, mqtt_names[i].name);
        snprintf(value, sizeof(value), "%d", mb_input_registers[mqtt_names[i].reg_addr]);
        ret = mqtt_publish_value(topic, value) || ret;
    }

    return ret;
}

