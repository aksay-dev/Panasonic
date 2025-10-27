/**
 * @file hpc.c
 * @brief Main HPC application implementation
 * @version 0.1.0
 * @date 2025
 */

#include "include/hpc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "include/protocol.h"

// test_decoder disabled

static const char *TAG = "HPC";

/**
 * @brief Initialize HPC application
 * @return ESP_OK on success
 */
esp_err_t hpc_init(void) {
    // Init NVS for storing runtime configuration
    esp_err_t ret = protocol_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize protocol: %s", esp_err_to_name(ret));
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