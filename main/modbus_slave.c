/**
 * @file modbus_slave.c
 * @brief Modbus RTU slave implementation using ESP-IDF esp-modbus v2.x
 * @version 1.0.0
 * @date 2025
 */

#include "include/modbus_slave.h"
#include "include/modbus_params.h"
#include "include/decoder.h"
#include "esp_modbus_slave.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MODBUS_SLAVE";

// Modbus slave handle
static void *mbc_slave_handle = NULL;

// Modbus communication parameters
#define MB_PORT_NUM         UART_NUM_1    // UART1
#define MB_DEV_SPEED        (9600)        // Baud rate
#define MB_SLAVE_ADDR       (7)           // Slave address
#define MB_UART_TXD         (25)          // TX pin
#define MB_UART_RXD         (26)          // RX pin

// Task handle
static TaskHandle_t mb_task_handle = NULL;

/**
 * @brief Modbus task - periodically update input registers and check events
 */
static void modbus_task(void *pvParameters) {
    mb_param_info_t reg_info;
    
    ESP_LOGI(TAG, "Modbus task started");
    
    while (1) {
        // Update input registers from heat pump data
        esp_err_t ret = modbus_params_update_inputs();
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "Failed to update input registers: %s", esp_err_to_name(ret));
        }
        
        // Check for Modbus events
        mb_event_group_t event = mbc_slave_check_event(mbc_slave_handle, 
                                                       MB_EVENT_HOLDING_REG_WR | MB_EVENT_INPUT_REG_RD);
        
        if (event & MB_EVENT_HOLDING_REG_WR) {
            ESP_LOGI(TAG, "Holding register write event");
            
            // Get event info
            ret = mbc_slave_get_param_info(mbc_slave_handle, &reg_info, MB_PAR_INFO_TOUT);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Write to address: 0x%04X, size: %d", 
                         reg_info.mb_offset, reg_info.size);
                
                // Process the write - iterate through written registers
                for (uint16_t i = 0; i < reg_info.size; i++) {
                    uint16_t reg_addr = reg_info.mb_offset + i;
                    if (reg_addr >= MB_REG_HOLDING_START && 
                        reg_addr < MB_REG_HOLDING_START + MB_REG_HOLDING_COUNT) {
                        modbus_params_process_holding_write(reg_addr);
                    }
                }
            }
        }
        
        if (event & MB_EVENT_INPUT_REG_RD) {
            ESP_LOGD(TAG, "Input register read event");
        }
        
        // Update every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Initialize Modbus RTU slave
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_init(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing Modbus RTU slave");
    
    // Initialize Modbus parameters
    ret = modbus_params_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Modbus parameters: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize Modbus slave controller
    mb_communication_info_t comm_config = {
        .ser_opts.port = MB_PORT_NUM,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = MB_DEV_SPEED,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = MB_SLAVE_ADDR,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1
    };
    
    ret = mbc_slave_create_serial(&comm_config, &mbc_slave_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Modbus slave create failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Define Modbus memory areas
    mb_register_area_descriptor_t reg_area;
    
    // Input registers area
    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START;
    reg_area.address = (void *)mb_input_registers;
    reg_area.size = MB_REG_INPUT_COUNT * sizeof(uint16_t);
    reg_area.access = MB_ACCESS_RO;
    ret = mbc_slave_set_descriptor(mbc_slave_handle, reg_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set input registers descriptor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Holding registers area
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START;
    reg_area.address = (void *)mb_holding_registers;
    reg_area.size = MB_REG_HOLDING_COUNT * sizeof(uint16_t);
    reg_area.access = MB_ACCESS_RW;
    ret = mbc_slave_set_descriptor(mbc_slave_handle, reg_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set holding registers descriptor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set UART pins
    ret = uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, 
                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Modbus RTU slave initialized successfully");
    ESP_LOGI(TAG, "Slave address: %d, Port: UART%d, Baud: %d", 
             MB_SLAVE_ADDR, MB_PORT_NUM, MB_DEV_SPEED);
    ESP_LOGI(TAG, "UART pins: TX=GPIO%d, RX=GPIO%d", MB_UART_TXD, MB_UART_RXD);
    ESP_LOGI(TAG, "Input registers: 0x%04X-0x%04X (%d regs)", 
             MB_REG_INPUT_START, MB_REG_INPUT_START + MB_REG_INPUT_COUNT - 1, MB_REG_INPUT_COUNT);
    ESP_LOGI(TAG, "Holding registers: 0x%04X-0x%04X (%d regs)", 
             MB_REG_HOLDING_START, MB_REG_HOLDING_START + MB_REG_HOLDING_COUNT - 1, MB_REG_HOLDING_COUNT);
    
    return ESP_OK;
}

/**
 * @brief Start Modbus RTU slave communication
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_start(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Starting Modbus RTU slave");
    
    if (mbc_slave_handle == NULL) {
        ESP_LOGE(TAG, "Modbus slave not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Start Modbus controller
    ret = mbc_slave_start(mbc_slave_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Modbus slave start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create task to periodically update input registers and process events
    BaseType_t task_ret = xTaskCreate(modbus_task, "modbus_task", 4096, 
                                      NULL, 5, &mb_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Modbus task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Modbus RTU slave started successfully");
    
    return ESP_OK;
}

/**
 * @brief Stop Modbus RTU slave communication
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_stop(void) {
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Stopping Modbus RTU slave");
    
    // Delete task if running
    if (mb_task_handle != NULL) {
        vTaskDelete(mb_task_handle);
        mb_task_handle = NULL;
    }
    
    // Stop and delete Modbus controller
    if (mbc_slave_handle != NULL) {
        ret = mbc_slave_stop(mbc_slave_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Modbus slave stop failed: %s", esp_err_to_name(ret));
        }
        
        ret = mbc_slave_delete(mbc_slave_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Modbus slave delete failed: %s", esp_err_to_name(ret));
        }
        
        mbc_slave_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Modbus RTU slave stopped");
    
    return ret;
}

/**
 * @brief Update input registers from heat pump decoded data
 * @return ESP_OK on success
 */
esp_err_t modbus_update_input_registers(void) {
    return modbus_params_update_inputs();
}
