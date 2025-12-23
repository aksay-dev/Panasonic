/**
 * @file hpc.c
 * @brief Main HPC application implementation
 * @version 0.1.0
 * @date 2025
 */

#include "include/hpc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/protocol.h"
#include "include/modbus_slave.h"
#include "include/modbus_params.h"
#include "include/nvs_hp.h"
#include "include/mqtt_pub.h"
#include "include/wifi_connect.h"
#include "include/adc.h"
#include "include/ds18b20a.h"
#include "include/http_server.h"

// test_decoder disabled

static const char *TAG = "HPC";

// Factory reset button
static bool reset_button_ready = false;
static TickType_t reset_press_start = 0;
static bool reset_action_performed = false;

#define HPC_RESET_BUTTON_GPIO   GPIO_NUM_0
#define HPC_RESET_HOLD_TICKS    pdMS_TO_TICKS(4000)

/**
 * @brief Initialize HPC application
 * @return ESP_OK on success
 */
esp_err_t hpc_init(void) {
    esp_err_t ret;
    
    // Initialize WiFi first (required for MQTT)
    ret = wifi_connect_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize heat pump protocol
    ret = protocol_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize protocol: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Modbus slave
    ret = modbus_slave_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Modbus slave: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize factory reset button
    hpc_factory_reset_button_init();

    // Initialize ADC module
    ret = adc_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize ADC: %s (continuing without ADC)", esp_err_to_name(ret));
        // Don't fail initialization if ADC fails - it's optional
    }

    // Initialize DS18B20 sensor
    ret = ds18b20_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize DS18B20: %s (continuing without DS18B20)", esp_err_to_name(ret));
        // Don't fail initialization if DS18B20 fails - it's optional
    }

    // Initialize MQTT client (will connect when WiFi is ready)
    ret = mqtt_client_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize MQTT client: %s (continuing without MQTT)", esp_err_to_name(ret));
        // Don't fail initialization if MQTT fails - it's optional
    }

    return ESP_OK;
}

/**
 * @brief Start HPC application
 * @return ESP_OK on success
 */
esp_err_t hpc_start(void) {
    esp_err_t ret;
    
    // Start WiFi connection first (required for MQTT and HTTP server)
    ret = wifi_connect_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start WiFi: %s (MQTT and HTTP server will not work)", esp_err_to_name(ret));
        // Continue anyway - WiFi/MQTT/HTTP are optional
    } else {
        char ip_str[16];
        if (wifi_connect_get_ip(ip_str, sizeof(ip_str)) == ESP_OK) {
            ESP_LOGI(TAG, "WiFi connected, IP: %s", ip_str);
            
            // Start HTTP server
            ret = http_server_start();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "HTTP server started on http://%s", ip_str);
            }
        }
    }
    
    // Start heat pump protocol
    ret = protocol_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start protocol: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start Modbus slave
    ret = modbus_slave_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Modbus slave: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start ADC reading task
    ret = adc_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start ADC: %s (continuing without ADC)", esp_err_to_name(ret));
        // Don't fail start if ADC fails - it's optional
    } else {
        ESP_LOGI(TAG, "ADC started successfully");
    }

    // Start DS18B20 reading task
    ret = ds18b20_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start DS18B20: %s (continuing without DS18B20)", esp_err_to_name(ret));
        // Don't fail start if DS18B20 fails - it's optional
    } else {
        ESP_LOGI(TAG, "DS18B20 started successfully");
    }

    // Start MQTT client (only if WiFi is connected)
    if (wifi_connect_is_connected()) {
        ret = mqtt_client_start();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start MQTT client: %s (continuing without MQTT)", esp_err_to_name(ret));
            // Don't fail start if MQTT fails - it's optional
        }
    } else {
        ESP_LOGW(TAG, "WiFi not connected, skipping MQTT start");
    }

    return ESP_OK;
}

/**
 * @brief Apply factory default Modbus settings (9600 8N1, slave ID 7)
 *
 * По требованию: контроллер не пересоздаем, только записываем
 * заводские параметры в NVS, чтобы они вступили в силу при
 * следующей перезагрузке.
 */
static void hpc_apply_factory_defaults(void) {
    modbus_serial_config_t default_cfg = {
        .baudrate = MB_DEV_SPEED,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .data_bits = UART_DATA_8_BITS,
        .slave_addr = MB_SLAVE_ADDR
    };
    
    ESP_LOGI(TAG, "Factory reset: saving default Modbus settings (9600 8N1, slave=%d) to NVS", MB_SLAVE_ADDR);
    
    // Сохраняем дефолтные параметры Modbus в NVS
    esp_err_t ret = modbus_nvs_save_config(&default_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save factory Modbus settings to NVS: %s", esp_err_to_name(ret));
        return;
    }

    // Сбрасываем флаг опционной платы в 0 и сохраняем в NVS
    int32_t opt_index = MB_HOLDING_OPT_PCB_AVAILABLE - MB_REG_HOLDING_START;
    if (opt_index >= 0 && opt_index < MB_REG_HOLDING_COUNT) {
        mb_holding_registers[opt_index] = 0;
    }
    
    esp_err_t save_ret = modbus_nvs_save_opt_pcb(0);
    if (save_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset OPT_PCB flag in NVS: %s", esp_err_to_name(save_ret));
    } else {
        ESP_LOGI(TAG, "OPT_PCB flag reset to factory default");
    }
    
    ESP_LOGI(TAG, "Factory Modbus settings saved to NVS (will apply after reboot)");
}

/**
 * @brief Initialize factory reset button (BOOT button)
 */
void hpc_factory_reset_button_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HPC_RESET_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err == ESP_OK) {
        reset_button_ready = true;
        reset_press_start = 0;
        reset_action_performed = false;
        ESP_LOGI(TAG, "Factory reset button initialized on GPIO%d", HPC_RESET_BUTTON_GPIO);
    } else {
        reset_button_ready = false;
        ESP_LOGE(TAG, "Failed to configure factory reset button: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Poll factory reset button (BOOT button)
 * Must be called periodically (e.g., every 100ms)
 */
void hpc_factory_reset_button_poll(void) {
    if (!reset_button_ready) {
        return;
    }

    int level = gpio_get_level(HPC_RESET_BUTTON_GPIO);
    TickType_t now = xTaskGetTickCount();

    if (level == 0) { // button pressed (active low)
        if (reset_press_start == 0) {
            reset_press_start = now;
            reset_action_performed = false;
            ESP_LOGI(TAG, "Factory reset button pressed");
        } else if (!reset_action_performed) {
            TickType_t hold_time = now - reset_press_start;
            if (hold_time >= HPC_RESET_HOLD_TICKS) {
                ESP_LOGI(TAG, "Factory reset button held for 4 seconds, applying defaults");
                hpc_apply_factory_defaults();
                reset_action_performed = true;
            }
        }
    } else {
        if (reset_press_start != 0 && !reset_action_performed) {
            TickType_t hold_time = now - reset_press_start;
            ESP_LOGI(TAG, "Factory reset button released after %lu ms", pdTICKS_TO_MS(hold_time));
        }
        reset_press_start = 0;
        reset_action_performed = false;
    }
}

/*
 * @brief Restart application
 */
void app_restart() {
    ESP_LOGE(TAG, "Restarting application");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
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
        app_restart();
    }

    // Start application
    ret = hpc_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start hpc: %s", esp_err_to_name(ret));
        app_restart();
    }

    ESP_LOGI(TAG, "HPC application version %s started successfully", HPC_VERSION_STRING);

    // Main loop - poll factory reset button
    while (1) {
        hpc_factory_reset_button_poll();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}