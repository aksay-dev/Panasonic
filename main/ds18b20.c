/**
 * @file ds18b20.c
 * @brief DS18B20 temperature sensor implementation
 * @version 1.0.0
 * @date 2025
 */

#include "include/ds18b20a.h"
#include "include/modbus_params.h"
#include "esp_log.h"
#include "esp_err.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <limits.h>

static const char *TAG = "DS18B20";

// 1-Wire bus handle
static onewire_bus_handle_t onewire_bus = NULL;

// DS18B20 device handle
static ds18b20_device_handle_t ds18b20_device = NULL;

// Task handle
static TaskHandle_t ds18b20_task_handle = NULL;
static bool ds18b20_initialized = false;

// Current temperature value (in °C * 100)
static int16_t current_temperature = 0;

/**
 * @brief DS18B20 reading task
 * Reads temperature with 12-bit resolution and updates Modbus register
 */
static void ds18b20_task(void *pvParameters) {
    ESP_LOGI(TAG, "DS18B20 task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(DS18B20_UPDATE_INTERVAL_MS);
    
    while (1) {
        if (onewire_bus != NULL && ds18b20_device != NULL) {
            float temperature = 0.0f;
            
            // Trigger temperature conversion for all sensors on the bus
            esp_err_t ret = ds18b20_trigger_temperature_conversion_for_all(onewire_bus);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to trigger temperature conversion: %s", esp_err_to_name(ret));
            } else {
                // Wait for conversion to complete (750ms for 12-bit resolution)
                vTaskDelay(pdMS_TO_TICKS(800));
                
                // Read temperature from DS18B20
                ret = ds18b20_get_temperature(ds18b20_device, &temperature);
                
                if (ret == ESP_OK) {
                    // Convert to int16_t * 100 format (same as other temperature registers)
                    int16_t temp_x100 = (int16_t)(temperature * 100.0f);
                    current_temperature = temp_x100;
                    
                    // Update Modbus register
                    uint16_t reg_addr = MB_INPUT_DS18B20_TEMP;
                    if (reg_addr >= MB_REG_INPUT_START && reg_addr < MB_REG_INPUT_START + MB_REG_INPUT_COUNT) {
                        uint16_t reg_index = reg_addr - MB_REG_INPUT_START;
                        mb_input_registers[reg_index] = temp_x100;
                    }
                    
                    ESP_LOGD(TAG, "DS18B20 temperature: %.2f°C (raw: %d)", 
                             temperature, temp_x100);
                } else {
                    ESP_LOGW(TAG, "Failed to read DS18B20 temperature: %s", esp_err_to_name(ret));
                    // Set invalid value on error
                    current_temperature = INT16_MIN;
                    uint16_t reg_addr = MB_INPUT_DS18B20_TEMP;
                    if (reg_addr >= MB_REG_INPUT_START && reg_addr < MB_REG_INPUT_START + MB_REG_INPUT_COUNT) {
                        uint16_t reg_index = reg_addr - MB_REG_INPUT_START;
                        mb_input_registers[reg_index] = INT16_MIN;
                    }
                }
            }
        } else {
            ESP_LOGW(TAG, "DS18B20 device not initialized");
        }
        
        // Wait for next update interval
        vTaskDelayUntil(&last_wake_time, interval);
    }
}

/**
 * @brief Initialize DS18B20 sensor
 */
esp_err_t ds18b20_init(void) {
    if (ds18b20_initialized) {
        ESP_LOGW(TAG, "DS18B20 already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing DS18B20 on GPIO%d", DS18B20_GPIO);
    
    // Configure 1-Wire bus
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = DS18B20_GPIO,
        .flags = {
            .en_pull_up = true, // Enable internal pull-up resistor
        }
    };
    
    // Configure RMT for 1-Wire bus
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    
    // Create 1-Wire bus with RMT
    esp_err_t ret = onewire_new_bus_rmt(&bus_config, &rmt_config, &onewire_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create 1-Wire bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "1-Wire bus installed on GPIO%d", DS18B20_GPIO);
    
    // Search for DS18B20 devices
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    bool device_found = false;
    
    // Create device iterator
    ret = onewire_new_device_iter(onewire_bus, &iter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create device iterator: %s", esp_err_to_name(ret));
        onewire_bus_del(onewire_bus);
        onewire_bus = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    
    // Search for DS18B20 device
    esp_err_t search_result = ESP_OK;
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) {
            // Try to create DS18B20 device from found device
            ds18b20_config_t ds_cfg = {};  // Empty config uses defaults
            
            ret = ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20_device);
            if (ret == ESP_OK) {
                // Found DS18B20 device
                onewire_device_address_t address;
                ds18b20_get_device_address(ds18b20_device, &address);
                ESP_LOGI(TAG, "Found DS18B20, address: %016llX", address);
                
                // Set resolution to 12 bits
                ret = ds18b20_set_resolution(ds18b20_device, DS18B20_RESOLUTION_12B);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to set DS18B20 resolution: %s", esp_err_to_name(ret));
                }
                
                device_found = true;
                break;
            } else {
                ESP_LOGD(TAG, "Found unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    
    // Clean up iterator
    onewire_del_device_iter(iter);
    
    if (!device_found) {
        ESP_LOGE(TAG, "DS18B20 device not found on 1-Wire bus");
        onewire_bus_del(onewire_bus);
        onewire_bus = NULL;
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "DS18B20 initialized successfully with %d-bit resolution", DS18B20_RESOLUTION_BITS);
    
    ds18b20_initialized = true;
    current_temperature = 0;
    
    return ESP_OK;
}

/**
 * @brief Start DS18B20 reading task
 */
esp_err_t ds18b20_start(void) {
    if (!ds18b20_initialized) {
        ESP_LOGE(TAG, "DS18B20 not initialized. Call ds18b20_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (ds18b20_task_handle != NULL) {
        ESP_LOGW(TAG, "DS18B20 task already running");
        return ESP_OK;
    }
    
    // Create DS18B20 reading task
    BaseType_t ret = xTaskCreate(
        ds18b20_task,
        "ds18b20_task",
        4096,  // Stack size
        NULL,
        4,     // Priority
        &ds18b20_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create DS18B20 task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "DS18B20 task started");
    return ESP_OK;
}

/**
 * @brief Stop DS18B20 reading task
 */
esp_err_t ds18b20_stop(void) {
    if (ds18b20_task_handle == NULL) {
        return ESP_OK;
    }
    
    vTaskDelete(ds18b20_task_handle);
    ds18b20_task_handle = NULL;
    
    ESP_LOGI(TAG, "DS18B20 task stopped");
    return ESP_OK;
}

/**
 * @brief Get current cached temperature value
 */
esp_err_t ds18b20_get_cached_temperature(int16_t *temperature) {
    if (!ds18b20_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *temperature = current_temperature;
    return ESP_OK;
}
