/**
 * @file hpc.c
 * @brief Main HPC application implementation
 * @version 0.1.0
 * @date 2025
 */

#include "include/hpc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "include/protocol.h"
#include "include/mqtt.h"

// test_decoder disabled

static const char *TAG = "HPC";

/**
 * @brief Initialize HPC application
 * @return ESP_OK on success
 */
esp_err_t hpc_init(void) {
    // Init NVS for storing runtime configuration
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    if (nvs_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NVS: %s", esp_err_to_name(nvs_ret));
        return nvs_ret;
    }
    esp_err_t ret = protocol_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize protocol: %s", esp_err_to_name(ret));
        return ret;
    }
    // Load MQTT configuration: try NVS overrides, fallback to sdkconfig
    char uri[128] = {0};
    char client_id[64] = {0};
    char base_topic[64] = {0};
    char username[64] = {0};
    char password[64] = {0};

    // Defaults from sdkconfig
    snprintf(uri, sizeof(uri), "%s",
#ifdef CONFIG_HPC_MQTT_URI
        CONFIG_HPC_MQTT_URI
#else
        "mqtt://192.168.1.100:1883"
#endif
    );
    snprintf(client_id, sizeof(client_id), "%s",
#ifdef CONFIG_HPC_MQTT_CLIENT_ID
        CONFIG_HPC_MQTT_CLIENT_ID
#else
        "panasonic"
#endif
    );
    snprintf(base_topic, sizeof(base_topic), "%s",
#ifdef CONFIG_HPC_MQTT_BASE_TOPIC
        CONFIG_HPC_MQTT_BASE_TOPIC
#else
        "hpc"
#endif
    );
    snprintf(username, sizeof(username), "%s",
#ifdef CONFIG_HPC_MQTT_USERNAME
        CONFIG_HPC_MQTT_USERNAME
#else
        ""
#endif
    );
    snprintf(password, sizeof(password), "%s",
#ifdef CONFIG_HPC_MQTT_PASSWORD
        CONFIG_HPC_MQTT_PASSWORD
#else
        ""
#endif
    );

    // Try load overrides from NVS
    nvs_handle_t nvs;
    if (nvs_open("hpc", NVS_READONLY, &nvs) == ESP_OK) {
        size_t len;
        len = sizeof(uri); nvs_get_str(nvs, "mqtt_uri", uri, &len);
        len = sizeof(client_id); nvs_get_str(nvs, "client_id", client_id, &len);
        len = sizeof(base_topic); nvs_get_str(nvs, "base_topic", base_topic, &len);
        len = sizeof(username); nvs_get_str(nvs, "username", username, &len);
        len = sizeof(password); nvs_get_str(nvs, "password", password, &len);
        nvs_close(nvs);
    }

    const hpc_mqtt_config_t mqtt_cfg = {
        .broker_uri = uri,
        .client_id = client_id,
        .username = (username[0] ? username : NULL),
        .password = (password[0] ? password : NULL),
        .base_topic = base_topic
    };
    ret = hpc_mqtt_init(&mqtt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

/**
 * @brief Start HPC application
 * @return ESP_OK on success
 */
esp_err_t hpc_start(void) {
    esp_err_t ret = protocol_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start protocol: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = hpc_mqtt_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

/**
 * @brief Main application entry point
 */
 void app_main(void) {
    esp_err_t ret;

    // Initialize application
    ret = hpc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize hpc: %s", esp_err_to_name(ret));
        esp_restart();
    }

    // Start application
    ret = hpc_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start hpc: %s", esp_err_to_name(ret));
        esp_restart();
    }

    ESP_LOGI(TAG, "HPC application version %s started successfully", HPC_VERSION_STRING);

    // test_decoder disabled

    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}