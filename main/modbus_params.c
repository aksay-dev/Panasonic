/**
 * @file modbus_params.c
 * @brief Modbus register mapping and data synchronization - FULL decoder data
 * @version 2.0.0
 * @date 2025
 */

#include "include/modbus_params.h"
#include "include/decoder.h"
#include "include/commands.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "MODBUS_PARAMS";

// Register arrays
// Note: Modbus registers are 16-bit values. They can be interpreted as signed or unsigned
// by the Modbus client. We use int16_t for values that can be negative (temperatures, etc.)
int16_t mb_input_registers[MB_REG_INPUT_COUNT] = {0};
int16_t mb_holding_registers[MB_REG_HOLDING_COUNT] = {0};

// Mutex to protect register access (shared resource between decoder task and Modbus task)
static SemaphoreHandle_t mb_registers_mutex = NULL;

/**
 * @brief Initialize Modbus parameter structures
 * @return ESP_OK on success
 */
esp_err_t modbus_params_init(void) {
    // Create mutex to protect register access
    mb_registers_mutex = xSemaphoreCreateMutex();
    if (mb_registers_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create registers mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Clear all registers
    memset(mb_input_registers, 0, sizeof(mb_input_registers));
    memset(mb_holding_registers, 0, sizeof(mb_holding_registers));
    
    ESP_LOGI(TAG, "Modbus parameters initialized: %d input, %d holding registers",
             MB_REG_INPUT_COUNT, MB_REG_HOLDING_COUNT);
    
    return ESP_OK;
}

/**
 * @brief Lock registers mutex (call before accessing registers)
 * @return true if lock acquired, false on timeout
 * @note Use portMAX_DELAY for infinite wait
 */
bool mb_registers_lock(TickType_t timeout) {
    if (mb_registers_mutex == NULL) {
        ESP_LOGE(TAG, "Registers mutex not initialized");
        return false;
    }
    return xSemaphoreTake(mb_registers_mutex, timeout) == pdTRUE;
}

/**
 * @brief Unlock registers mutex (call after accessing registers)
 */
void mb_registers_unlock(void) {
    if (mb_registers_mutex == NULL) {
        ESP_LOGE(TAG, "Registers mutex not initialized");
        return;
    }
    xSemaphoreGive(mb_registers_mutex);
}

/**
 * @brief Helper function to copy string to Modbus registers (2 bytes per register)
 * @param dest_reg Start register index
 * @param src Source string
 * @param max_len Maximum string length
 */
void copy_string_to_registers(uint16_t dest_reg, const char *src, size_t max_len) {
    size_t len = strlen(src);
    if (len > max_len) {
        len = max_len;
    }
    
    // Copy 2 bytes per register (big-endian format)
    for (size_t i = 0; i < max_len; i += 2) {
        uint8_t byte1 = (i < len) ? (uint8_t)src[i] : 0;
        uint8_t byte2 = (i + 1 < len) ? (uint8_t)src[i + 1] : 0;
        mb_input_registers[dest_reg + i / 2] = (int16_t)((byte1 << 8) | byte2);
    }
}

/**
 * @brief Update input registers from decoded heat pump data
 * @return ESP_OK on success
 */
esp_err_t modbus_params_update_inputs(void) {
    // NOTE: This function is kept for compatibility, but most data is now written
    // directly to Modbus registers in decode_main_data(), decode_extra_data(), 
    // and decode_opt_data(). Only system-level updates would go here if needed.

    // All main data is now written directly in decode_main_data()
    // All extra data is now written directly in decode_extra_data()
    // All optional data is now written directly in decode_opt_data()
    
    // This function may be used for other updates like uptime, status flags, etc.
    // Currently, it's essentially a no-op since all data flows directly from decoder to Modbus registers.

    return ESP_OK;
}

/**
 * @brief Process holding register writes (execute commands)
 * @param reg_addr Register address that was written
 * @return ESP_OK on success
 */
esp_err_t modbus_params_process_holding_write(uint16_t reg_addr) {
    esp_err_t ret = ESP_OK;
    int16_t value = mb_holding_registers[reg_addr - MB_REG_HOLDING_START];
    
    ESP_LOGI(TAG, "Processing write to register 0x%04X, value: %d", reg_addr, value);

    switch (reg_addr) {
        // Control commands
        case MB_HOLDING_SET_HEATPUMP:
            ret = set_heatpump_state(value != 0);
            break;
            
        case MB_HOLDING_SET_PUMP:
            ret = set_pump(value != 0);
            break;
            
        case MB_HOLDING_SET_MAX_PUMP_DUTY:
            if (value <= 100) {
                ret = set_max_pump_duty((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid max pump duty: %u (must be 0-100)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case MB_HOLDING_SET_QUIET_MODE:
            if (value <= 3) {
                ret = set_quiet_mode((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid quiet mode: %u (must be 0-3)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case MB_HOLDING_SET_POWERFUL_MODE:
            if (value <= 90) {
                ret = set_powerful_mode((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid powerful mode: %u (must be 0-90 minutes)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case MB_HOLDING_SET_OPERATION_MODE:
            if (value <= 6) {
                ret = set_operation_mode((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid operation mode: %u (must be 0-6)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case MB_HOLDING_SET_HOLIDAY_MODE:
            ret = set_holiday_mode(value != 0);
            break;
            
        case MB_HOLDING_SET_FORCE_DHW:
            ret = set_force_DHW(value != 0);
            break;
            
        case MB_HOLDING_SET_FORCE_DEFROST:
            ret = set_force_defrost(value != 0);
            break;
            
        case MB_HOLDING_SET_FORCE_STERILIZATION:
            ret = set_force_sterilization(value != 0);
            break;
            
        case MB_HOLDING_SET_MAIN_SCHEDULE:
            ret = set_main_schedule(value != 0);
            break;
            
        case MB_HOLDING_SET_RESET:
            ret = set_reset(value != 0);
            break;
            
        case MB_HOLDING_SET_ZONES:
            if (value <= 2) {
                ret = set_zones((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid zones: %u (must be 0-2)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;

        // External control block (0x100D-0x100F)
        case MB_HOLDING_SET_EXTERNAL_CONTROL:
            ret = set_external_control(value != 0);
            break;
        case MB_HOLDING_SET_EXTERNAL_ERROR:
            ret = set_external_error(value != 0);
            break;
        case MB_HOLDING_SET_EXTERNAL_COMPRESSOR_CONTROL:
            ret = set_external_compressor_control(value != 0);
            break;

        // Additional controls (0x1010-0x1015)
        case MB_HOLDING_SET_EXTERNAL_HEAT_COOL_CONTROL:
            ret = set_external_heat_cool_control(value != 0);
            break;
        case MB_HOLDING_SET_BIVALENT_CONTROL:
            ret = set_bivalent_control(value != 0);
            break;
        case MB_HOLDING_SET_BIVALENT_MODE:
            if (value <= 2) {
                ret = set_bivalent_mode((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid bivalent mode: %d (0-2)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        case MB_HOLDING_SET_ALT_EXTERNAL_SENSOR:
            ret = set_alt_external_sensor(value != 0);
            break;
        case MB_HOLDING_SET_EXTERNAL_PAD_HEATER:
            if (value <= 2) {
                ret = set_external_pad_heater((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid pad heater: %d (0-2)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        case MB_HOLDING_SET_BUFFER:
            ret = set_buffer(value != 0);
            break;

        // Temperature setpoints (value is raw decoded data, direct pass)
        case MB_HOLDING_SET_Z1_HEAT_TEMP:
            ret = set_z1_heat_request_temperature((int8_t)value);
            break;
            
        case MB_HOLDING_SET_Z1_COOL_TEMP:
            ret = set_z1_cool_request_temperature((int8_t)value);
            break;
            
        case MB_HOLDING_SET_Z2_HEAT_TEMP:
            ret = set_z2_heat_request_temperature((int8_t)value);
            break;
            
        case MB_HOLDING_SET_Z2_COOL_TEMP:
            ret = set_z2_cool_request_temperature((int8_t)value);
            break;
            
        case MB_HOLDING_SET_DHW_TEMP:
            ret = set_DHW_temp((int8_t)value);
            break;

        // Optional temperatures (we interpret holding int16 as whole °C and pass as float)
        case MB_HOLDING_SET_POOL_TEMP:
            ret = set_pool_temp((float)value);
            break;
        case MB_HOLDING_SET_BUFFER_TEMP:
            ret = set_buffer_temp((float)value);
            break;
        case MB_HOLDING_SET_Z1_ROOM_TEMP:
            ret = set_z1_room_temp((float)value);
            break;
        case MB_HOLDING_SET_Z1_WATER_TEMP:
            ret = set_z1_water_temp((float)value);
            break;
        case MB_HOLDING_SET_Z2_ROOM_TEMP:
            ret = set_z2_room_temp((float)value);
            break;
        case MB_HOLDING_SET_Z2_WATER_TEMP:
            ret = set_z2_water_temp((float)value);
            break;
        case MB_HOLDING_SET_SOLAR_TEMP:
            ret = set_solar_temp((float)value);
            break;

        // Optional controls
        case MB_HOLDING_SET_HEAT_COOL_MODE:
            ret = set_heat_cool_mode(value != 0);
            break;
        case MB_HOLDING_SET_COMPRESSOR_STATE:
            ret = set_compressor_state(value != 0);
            break;
        case MB_HOLDING_SET_SMART_GRID_MODE:
            ret = set_smart_grid_mode((uint8_t)value);
            break;
        case MB_HOLDING_SET_EXT_THERMOSTAT_1:
            ret = set_external_thermostat_1_state((uint8_t)value);
            break;
        case MB_HOLDING_SET_EXT_THERMOSTAT_2:
            ret = set_external_thermostat_2_state((uint8_t)value);
            break;
        case MB_HOLDING_SET_DEMAND_CONTROL:
            ret = set_demand_control((uint8_t)value);
            break;

        // Curves apply: read block 0x1060.. and invoke set_curves
        case MB_HOLDING_CURVES_APPLY: {
            uint8_t curves_bytes[MB_HOLDING_CURVES_REGS * 2];
            for (int i = 0; i < MB_HOLDING_CURVES_REGS; ++i) {
                int16_t reg = mb_holding_registers[(MB_HOLDING_CURVES_START - MB_REG_HOLDING_START) + i];
                curves_bytes[i * 2] = (uint8_t)((reg >> 8) & 0xFF);
                curves_bytes[i * 2 + 1] = (uint8_t)(reg & 0xFF);
            }
            ret = set_curves(curves_bytes);
            break;
        }

        // Deltas and timing (0x1030-0x1036) - int8 degrees or minutes
        case MB_HOLDING_SET_BUFFER_DELTA:
            ret = set_buffer_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_FLOOR_HEAT_DELTA:
            ret = set_floor_heat_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_FLOOR_COOL_DELTA:
            ret = set_floor_cool_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_DHW_HEAT_DELTA:
            ret = set_dhw_heat_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_HEATER_START_DELTA:
            ret = set_heater_start_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_HEATER_STOP_DELTA:
            ret = set_heater_stop_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_HEATER_DELAY_TIME:
            ret = set_heater_delay_time((uint8_t)value);
            break;

        // Bivalent temperatures (commands accept int8 setpoints)
        case MB_HOLDING_SET_BIVALENT_START_TEMP:
            ret = set_bivalent_start_temp((int8_t)value);
            break;
        case MB_HOLDING_SET_BIVALENT_AP_START_TEMP:
            ret = set_bivalent_ap_start_temp((int8_t)value);
            break;
        case MB_HOLDING_SET_BIVALENT_AP_STOP_TEMP:
            ret = set_bivalent_ap_stop_temp((int8_t)value);
            break;

        default:
            ESP_LOGW(TAG, "Write to unhandled register: 0x%04X", reg_addr);
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Command executed successfully for register 0x%04X", reg_addr);
    } else {
        ESP_LOGE(TAG, "Command failed for register 0x%04X: %s", reg_addr, esp_err_to_name(ret));
    }

    return ret;
}
