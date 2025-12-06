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
#include "freertos/task.h"
#include <string.h>
#include "include/nvs_hp.h"

static const char *TAG = "MODBUS_SLAVE";

// Modbus slave handle
static void *mbc_slave_handle = NULL;

// Task handle
static TaskHandle_t mb_task_handle = NULL;

modbus_serial_config_t base_serial_cfg = {
    .baudrate = MB_DEV_SPEED,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .data_bits = UART_DATA_8_BITS,
    .slave_addr = MB_SLAVE_ADDR
};

static bool modbus_slave_running = false;

// Shadow copy of holding registers to detect actual changes
static int16_t mb_holding_registers_shadow[MB_REG_HOLDING_COUNT];
static bool shadow_initialized = false;

// Forward declarations
static esp_err_t modbus_slave_setup_controller(void);
static bool modbus_slave_validate_serial_config(const modbus_serial_config_t *cfg);
static void modbus_log_serial_config(const char *prefix, const modbus_serial_config_t *cfg);

/**
 * @brief Setup Modbus controller with current serial configuration
 */
static esp_err_t modbus_slave_setup_controller(void) {
    if (mbc_slave_handle != NULL) {
        ESP_LOGW(TAG, "Modbus controller already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    mb_communication_info_t comm_config = {
        .ser_opts.port = MB_PORT_NUM,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = base_serial_cfg.baudrate,
        .ser_opts.parity = base_serial_cfg.parity,
        .ser_opts.uid = base_serial_cfg.slave_addr,
        .ser_opts.data_bits = base_serial_cfg.data_bits,
        .ser_opts.stop_bits = base_serial_cfg.stop_bits
    };

    ESP_LOGI(TAG, "Creating Modbus controller with slave_id=%u, baud=%lu",
             base_serial_cfg.slave_addr, (unsigned long)base_serial_cfg.baudrate);

    esp_err_t ret = mbc_slave_create_serial(&comm_config, &mbc_slave_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Modbus slave create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure input registers
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

    // Configure holding registers
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

    // Configure UART pins
    ret = uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Set RS485 half-duplex mode
    ret = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART RS485 mode: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ESP_LOGI(TAG, "UART1 configured: TX=GPIO%d, RX=GPIO%d, RTS=GPIO%d, Baud=%lu",
             MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, (unsigned long)base_serial_cfg.baudrate);
    ESP_LOGI(TAG, "RS485 Half-Duplex mode enabled");
    ESP_LOGI(TAG, "Slave address: %u, Data bits: %d, Stop bits: %d, Parity: %d",
             base_serial_cfg.slave_addr,
             base_serial_cfg.data_bits,
             base_serial_cfg.stop_bits,
             base_serial_cfg.parity);
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

/**
 * @brief Validate serial configuration parameters
 */
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

/**
 * @brief Log serial configuration
 */
static void modbus_log_serial_config(const char *prefix, const modbus_serial_config_t *cfg) {
    if (cfg == NULL) {
        return;
    }
    ESP_LOGI(TAG, "%s Modbus cfg -> baud=%lu, parity=%d, data_bits=%d, stop_bits=%d, slave_id=%u",
             prefix ? prefix : "Current",
             (unsigned long)cfg->baudrate,
             cfg->parity,
             cfg->data_bits,
             cfg->stop_bits,
             cfg->slave_addr);
}

/**
 * @brief Process holding register write event by comparing with shadow copy
 * This approach is more reliable than using mbc_slave_get_param_info() which
 * often returns incorrect size/address information
 */
static void modbus_process_holding_write_event(void) {
    // Shadow copy must be initialized before first write event (in modbus_slave_start)
    if (!shadow_initialized) {
        ESP_LOGW(TAG, "Shadow copy not initialized! This should not happen.");
        memcpy(mb_holding_registers_shadow, mb_holding_registers, sizeof(mb_holding_registers_shadow));
        shadow_initialized = true;
        // Don't process on emergency initialization - wait for next write
        return;
    }

    // Compare current values with shadow to detect actual changes
    uint16_t changed_count = 0;
    for (uint16_t i = 0; i < MB_REG_HOLDING_COUNT; i++) {
        if (mb_holding_registers[i] != mb_holding_registers_shadow[i]) {
            uint16_t reg_addr = MB_REG_HOLDING_START + i;
            int16_t old_val = mb_holding_registers_shadow[i];
            int16_t new_val = mb_holding_registers[i];
            
            ESP_LOGI(TAG, "Register 0x%04X changed: %d -> %d", reg_addr, old_val, new_val);
            
            // Update shadow BEFORE processing (so if processing fails, we don't retry)
            mb_holding_registers_shadow[i] = new_val;
            
            // Process the change
            modbus_params_process_holding_write(reg_addr);
            changed_count++;
        }
    }

    if (changed_count == 0) {
        ESP_LOGD(TAG, "Holding write event but no changes detected");
    } else {
        ESP_LOGI(TAG, "Processed %u holding register change(s)", changed_count);
    }
}

/**
 * @brief Modbus task - periodically check events
 */
static void modbus_task(void *pvParameters) {
    ESP_LOGI(TAG, "Modbus task started");
    
    while (1) {
        // Check for Modbus events only if controller is valid
        if (mbc_slave_handle != NULL) {
            mb_event_group_t event = mbc_slave_check_event(mbc_slave_handle, 
                                                           MB_EVENT_HOLDING_REG_WR | 
                                                           MB_EVENT_INPUT_REG_RD | 
                                                           MB_EVENT_HOLDING_REG_RD);
            
            if (event & MB_EVENT_HOLDING_REG_WR) {
                modbus_process_holding_write_event();
            }
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

    // Load stored configuration from NVS
    modbus_serial_config_t stored_cfg;
    esp_err_t load_ret = modbus_nvs_load_config(&stored_cfg);
    if (load_ret == ESP_OK) {
        if (modbus_slave_validate_serial_config(&stored_cfg)) {
            base_serial_cfg = stored_cfg;
            modbus_log_serial_config("Loaded NVS", &base_serial_cfg);
        } else {
            ESP_LOGW(TAG, "Stored Modbus config invalid, reverting to defaults");
        }
    } else if (load_ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "No Modbus config stored in NVS, using defaults");
    } else {
        ESP_LOGW(TAG, "Failed to load Modbus config from NVS: %s", esp_err_to_name(load_ret));
    }
    
    // Initialize Modbus parameters
    ret = modbus_params_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Modbus parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    // Restore optional PCB flag from NVS (default 0)
    uint8_t opt_pcb_flag = 0;
    esp_err_t opt_load_ret = modbus_nvs_load_opt_pcb(&opt_pcb_flag);
    if (opt_load_ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded OPT_PCB flag from NVS: %u", opt_pcb_flag);
    } else if (opt_load_ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "No OPT_PCB flag stored in NVS, using default 0");
        opt_pcb_flag = 0;
    } else {
        ESP_LOGW(TAG, "Failed to load OPT_PCB flag from NVS: %s", esp_err_to_name(opt_load_ret));
        opt_pcb_flag = 0;
    }
    int32_t opt_index = MB_HOLDING_OPT_PCB_AVAILABLE - MB_REG_HOLDING_START;
    if (opt_index >= 0 && opt_index < MB_REG_HOLDING_COUNT) {
        mb_holding_registers[opt_index] = (int16_t)(opt_pcb_flag ? 1 : 0);
    }
    
    // Setup Modbus controller
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
    
    // Initialize shadow copy of holding registers BEFORE first write event
    // This ensures we can detect changes on the very first write
    memcpy(mb_holding_registers_shadow, mb_holding_registers, sizeof(mb_holding_registers_shadow));
    shadow_initialized = true;
    ESP_LOGI(TAG, "Holding registers shadow initialized");
    
    // Create task to process events
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


/**
 * @brief Get current serial configuration
 * @param cfg_out Output buffer for configuration
 * @return ESP_OK on success
 */
esp_err_t modbus_slave_get_serial_config(modbus_serial_config_t *cfg_out) {
    if (cfg_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *cfg_out = base_serial_cfg;
    return ESP_OK;
}

void modbus_slave_update_shadow_copy(void) {
    if (shadow_initialized) {
        memcpy(mb_holding_registers_shadow, mb_holding_registers, sizeof(mb_holding_registers_shadow));
        ESP_LOGD(TAG, "Shadow copy updated after holding registers sync");
    }
}

