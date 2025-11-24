/**
 * @file modbus_slave.c
 * @brief Modbus RTU slave implementation using ESP-IDF esp-modbus v2.x
 * @version 1.0.0
 * @date 2025
 */

#include "modbus_slave.h"
#include "modbus_params.h"
#include "esp_modbus_slave.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MODBUS_SLAVE";

// Modbus slave handle
static void *mbc_slave_handle = NULL;

// Task handle
static TaskHandle_t mb_task_handle = NULL;

static modbus_serial_config_t current_serial_cfg = {
    .baudrate = MB_DEV_SPEED,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .data_bits = UART_DATA_8_BITS,
    .slave_addr = MB_SLAVE_ADDR
};

static bool modbus_slave_running = false;

static esp_err_t modbus_slave_setup_controller(void);
static bool modbus_slave_validate_serial_config(const modbus_serial_config_t *cfg);
static void modbus_slave_restore_previous_controller(const modbus_serial_config_t *prev_cfg,
                                                     bool restart_required,
                                                     bool had_controller);

static esp_err_t modbus_slave_setup_controller(void) {
    if (mbc_slave_handle != NULL) {
        ESP_LOGW(TAG, "Modbus controller already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    mb_communication_info_t comm_config = {
        .ser_opts.port = MB_PORT_NUM,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = current_serial_cfg.baudrate,
        .ser_opts.parity = current_serial_cfg.parity,
        .ser_opts.uid = current_serial_cfg.slave_addr,
        .ser_opts.data_bits = current_serial_cfg.data_bits,
        .ser_opts.stop_bits = current_serial_cfg.stop_bits
    };

    esp_err_t ret = mbc_slave_create_serial(&comm_config, &mbc_slave_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Modbus slave create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    mb_register_area_descriptor_t reg_area = {
        .type = MB_PARAM_INPUT,
        .start_offset = MB_REG_INPUT_START,
        .address = (void *)mb_input_registers,
        .size = MB_REG_INPUT_COUNT * sizeof(uint16_t),
        .access = MB_ACCESS_RO
    };

    ret = mbc_slave_set_descriptor(mbc_slave_handle, reg_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set input registers descriptor: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START;
    reg_area.address = (void *)mb_holding_registers;
    reg_area.size = MB_REG_HOLDING_COUNT * sizeof(uint16_t);
    reg_area.access = MB_ACCESS_RW;

    ret = mbc_slave_set_descriptor(mbc_slave_handle, reg_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set holding registers descriptor: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART RS485 mode: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ESP_LOGI(TAG, "UART1 configured: TX=GPIO%d, RX=GPIO%d, RTS=GPIO%d, Baud=%lu",
             MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, (unsigned long)current_serial_cfg.baudrate);
    ESP_LOGI(TAG, "RS485 Half-Duplex mode enabled");
    ESP_LOGI(TAG, "Slave address: %u, Data bits: %d, Stop bits: %d, Parity: %d",
             current_serial_cfg.slave_addr,
             current_serial_cfg.data_bits,
             current_serial_cfg.stop_bits,
             current_serial_cfg.parity);
    ESP_LOGI(TAG, "Input registers: 0x%04X-0x%04X (%d regs)",
             MB_REG_INPUT_START, MB_REG_INPUT_START + MB_REG_INPUT_COUNT - 1, MB_REG_INPUT_COUNT);
    ESP_LOGI(TAG, "Holding registers: 0x%04X-0x%04X (%d regs)",
             MB_REG_HOLDING_START, MB_REG_HOLDING_START + MB_REG_HOLDING_COUNT - 1, MB_REG_HOLDING_COUNT);

    return ESP_OK;

cleanup:
    if (mbc_slave_handle != NULL) {
        mbc_slave_delete(mbc_slave_handle);
        mbc_slave_handle = NULL;
    }
    return ret;
}

static bool modbus_slave_validate_serial_config(const modbus_serial_config_t *cfg) {
    if (cfg == NULL) {
        return false;
    }
    if (cfg->baudrate < 1200 || cfg->baudrate > 65535) {
        return false;
    }
    if (cfg->slave_addr < 1 || cfg->slave_addr > 247) {
        return false;
    }
    switch (cfg->parity) {
        case UART_PARITY_DISABLE:
        case UART_PARITY_EVEN:
        case UART_PARITY_ODD:
            break;
        default:
            return false;
    }
    switch (cfg->stop_bits) {
        case UART_STOP_BITS_1:
        case UART_STOP_BITS_2:
            break;
        default:
            return false;
    }
    switch (cfg->data_bits) {
        case UART_DATA_7_BITS:
        case UART_DATA_8_BITS:
            break;
        default:
            return false;
    }
    return true;
}

static void modbus_slave_restore_previous_controller(const modbus_serial_config_t *prev_cfg,
                                                     bool restart_required,
                                                     bool had_controller) {
    if (!had_controller || prev_cfg == NULL) {
        return;
    }

    current_serial_cfg = *prev_cfg;
    esp_err_t ret = modbus_slave_setup_controller();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore previous Modbus controller: %s", esp_err_to_name(ret));
        return;
    }

    if (restart_required) {
        ret = mbc_slave_start(mbc_slave_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restart restored Modbus controller: %s", esp_err_to_name(ret));
            (void)mbc_slave_delete(mbc_slave_handle);
            mbc_slave_handle = NULL;
        } else {
            modbus_slave_running = true;
        }
    }
}

/**
 * @brief Modbus task - periodically update input registers and check events
 */
static void modbus_task(void *pvParameters) {
    mb_param_info_t reg_info;
    uint32_t event_count = 0;
    
    ESP_LOGI(TAG, "Modbus task started");
    
    while (1) {
        // Update input registers from heat pump data
        esp_err_t ret;
        if (mbc_slave_handle == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Check for Modbus events
        mb_event_group_t event = mbc_slave_check_event(mbc_slave_handle, 
                                                       MB_EVENT_HOLDING_REG_WR | MB_EVENT_INPUT_REG_RD | 
                                                       MB_EVENT_HOLDING_REG_RD | MB_EVENT_COILS_RD | 
                                                       MB_EVENT_COILS_WR | MB_EVENT_DISCRETE_RD);
        
        if (event) {
            event_count++;
            ESP_LOGI(TAG, "Modbus event detected! Event: 0x%04X, Count: %lu", event, event_count);
        }
        
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
        
        if (event & MB_EVENT_HOLDING_REG_RD) {
            ESP_LOGI(TAG, "Holding register read event");
        }
        
        if (event & MB_EVENT_INPUT_REG_RD) {
            ESP_LOGI(TAG, "Input register read event");
        }
        
        if (event & MB_EVENT_COILS_RD) {
            ESP_LOGI(TAG, "Coils read event");
        }
        
        if (event & MB_EVENT_DISCRETE_RD) {
            ESP_LOGI(TAG, "Discrete inputs read event");
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
    
    ret = modbus_slave_setup_controller();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create Modbus controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Modbus RTU slave initialized successfully");
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
    modbus_slave_running = true;
    
    // Create task to periodically update input registers and process events
    BaseType_t task_ret = xTaskCreate(modbus_task, "modbus_task", 4096, 
                                      NULL, 5, &mb_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Modbus task");
        (void)mbc_slave_stop(mbc_slave_handle);
        modbus_slave_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Modbus RTU slave started successfully");
    
    return ESP_OK;
}

esp_err_t modbus_slave_apply_serial_config(const modbus_serial_config_t *cfg) {
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!modbus_slave_validate_serial_config(cfg)) {
        ESP_LOGE(TAG, "Invalid Modbus serial configuration requested");
        return ESP_ERR_INVALID_ARG;
    }

    bool controller_exists = (mbc_slave_handle != NULL);
    bool restart_required = modbus_slave_running;
    modbus_serial_config_t previous_cfg = current_serial_cfg;

    if (controller_exists) {
        if (restart_required) {
            esp_err_t stop_ret = mbc_slave_stop(mbc_slave_handle);
            if (stop_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to stop Modbus controller before reconfig: %s",
                         esp_err_to_name(stop_ret));
            }
            modbus_slave_running = false;
        }
        esp_err_t del_ret = mbc_slave_delete(mbc_slave_handle);
        if (del_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete Modbus controller: %s", esp_err_to_name(del_ret));
            return del_ret;
        }
        mbc_slave_handle = NULL;
    }

    current_serial_cfg = *cfg;

    esp_err_t ret = modbus_slave_setup_controller();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup controller with new config: %s", esp_err_to_name(ret));
        current_serial_cfg = previous_cfg;
        modbus_slave_restore_previous_controller(&previous_cfg, restart_required, controller_exists);
        return ret;
    }

    if (restart_required) {
        ret = mbc_slave_start(mbc_slave_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restart Modbus controller: %s", esp_err_to_name(ret));
            (void)mbc_slave_delete(mbc_slave_handle);
            mbc_slave_handle = NULL;
            modbus_slave_restore_previous_controller(&previous_cfg, true, controller_exists);
            current_serial_cfg = previous_cfg;
            return ret;
        }
        modbus_slave_running = true;
    }

    ESP_LOGI(TAG, "Applied Modbus config: baud=%lu, parity=%d, data_bits=%d, stop_bits=%d, slave_id=%u",
             (unsigned long)current_serial_cfg.baudrate,
             current_serial_cfg.parity,
             current_serial_cfg.data_bits,
             current_serial_cfg.stop_bits,
             current_serial_cfg.slave_addr);

    return ESP_OK;
}

esp_err_t modbus_slave_get_serial_config(modbus_serial_config_t *cfg_out) {
    if (cfg_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *cfg_out = current_serial_cfg;
    return ESP_OK;
}
