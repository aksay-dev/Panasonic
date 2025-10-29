/**
 * @file modbus_params.c
 * @brief Modbus register mapping and data synchronization
 * @version 1.0.0
 * @date 2025
 */

#include "include/modbus_params.h"
#include "include/decoder.h"
#include "include/commands.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "MODBUS_PARAMS";

// Register arrays
uint16_t mb_input_registers[MB_REG_INPUT_COUNT] = {0};
uint16_t mb_holding_registers[MB_REG_HOLDING_COUNT] = {0};

/**
 * @brief Initialize Modbus parameter structures
 * @return ESP_OK on success
 */
esp_err_t modbus_params_init(void) {
    // Clear all registers
    memset(mb_input_registers, 0, sizeof(mb_input_registers));
    memset(mb_holding_registers, 0, sizeof(mb_holding_registers));
    
    ESP_LOGI(TAG, "Modbus parameters initialized: %d input, %d holding registers",
             MB_REG_INPUT_COUNT, MB_REG_HOLDING_COUNT);
    
    return ESP_OK;
}

/**
 * @brief Convert int8_t temperature to Modbus format (°C * 100)
 */
static inline uint16_t temp_int8_to_mb(int8_t temp) {
    return (uint16_t)(temp * 100);
}

/**
 * @brief Convert int16_t (already * 100) to Modbus format
 */
static inline uint16_t temp_int16_to_mb(int16_t temp) {
    return (uint16_t)temp;
}

/**
 * @brief Update input registers from decoded heat pump data
 * @return ESP_OK on success
 */
esp_err_t modbus_params_update_inputs(void) {
    // Check if decoded data is valid
    if (!g_decoded_data.data_valid) {
        ESP_LOGW(TAG, "Decoded data not valid, skipping register update");
        return ESP_ERR_INVALID_STATE;
    }

    // System information
    mb_input_registers[MB_INPUT_STATUS] = g_decoded_data.data_valid ? 1 : 0;
    mb_input_registers[MB_INPUT_UPTIME_LOW] = (uint16_t)(g_decoded_data.last_update_time & 0xFFFF);
    mb_input_registers[MB_INPUT_UPTIME_HIGH] = (uint16_t)((g_decoded_data.last_update_time >> 16) & 0xFFFF);
    mb_input_registers[MB_INPUT_LAST_UPDATE_LOW] = (uint16_t)(g_decoded_data.last_update_time & 0xFFFF);
    mb_input_registers[MB_INPUT_LAST_UPDATE_HIGH] = (uint16_t)((g_decoded_data.last_update_time >> 16) & 0xFFFF);

    // Temperatures - int16_t types (already * 100)
    mb_input_registers[MB_INPUT_MAIN_INLET_TEMP] = temp_int16_to_mb(g_decoded_data.main_inlet_temp);
    mb_input_registers[MB_INPUT_MAIN_OUTLET_TEMP] = temp_int16_to_mb(g_decoded_data.main_outlet_temp);
    
    // Temperatures - int8_t types (need to multiply by 100)
    mb_input_registers[MB_INPUT_DHW_TEMP] = temp_int8_to_mb(g_decoded_data.dhw_temp);
    mb_input_registers[MB_INPUT_OUTSIDE_TEMP] = temp_int8_to_mb(g_decoded_data.outside_temp);
    mb_input_registers[MB_INPUT_BUFFER_TEMP] = temp_int8_to_mb(g_decoded_data.buffer_temp);
    mb_input_registers[MB_INPUT_SOLAR_TEMP] = temp_int8_to_mb(g_decoded_data.solar_temp);
    mb_input_registers[MB_INPUT_POOL_TEMP] = temp_int8_to_mb(g_decoded_data.pool_temp);
    mb_input_registers[MB_INPUT_Z1_ROOM_TEMP] = temp_int8_to_mb(g_decoded_data.z1_temp);
    mb_input_registers[MB_INPUT_Z1_WATER_TEMP] = temp_int8_to_mb(g_decoded_data.z1_water_temp);
    mb_input_registers[MB_INPUT_Z2_ROOM_TEMP] = temp_int8_to_mb(g_decoded_data.z2_temp);
    mb_input_registers[MB_INPUT_Z2_WATER_TEMP] = temp_int8_to_mb(g_decoded_data.z2_water_temp);
    mb_input_registers[MB_INPUT_COMPRESSOR_TEMP] = temp_int8_to_mb(g_decoded_data.discharge_temp);
    mb_input_registers[MB_INPUT_EVAPORATOR_TEMP] = temp_int8_to_mb(g_decoded_data.eva_outlet_temp);
    mb_input_registers[MB_INPUT_CONDENSOR_TEMP] = temp_int8_to_mb(g_decoded_data.main_hex_outlet_temp);
    mb_input_registers[MB_INPUT_INVERTER_TEMP] = temp_int8_to_mb(g_decoded_data.ipm_temp);
    mb_input_registers[MB_INPUT_OUTDOOR_COIL_TEMP] = temp_int8_to_mb(g_decoded_data.outside_pipe_temp);

    // Power and energy - uint16_t types (copy directly)
    mb_input_registers[MB_INPUT_HEAT_POWER_PRODUCTION] = g_decoded_data.heat_power_production;
    mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION] = g_decoded_data.heat_power_consumption;
    mb_input_registers[MB_INPUT_COOL_POWER_PRODUCTION] = g_decoded_data.cool_power_production;
    mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION] = g_decoded_data.cool_power_consumption;
    mb_input_registers[MB_INPUT_DHW_POWER_PRODUCTION] = g_decoded_data.dhw_power_production;
    mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION] = g_decoded_data.dhw_power_consumption;
    mb_input_registers[MB_INPUT_COMPRESSOR_FREQ] = g_decoded_data.compressor_freq;
    mb_input_registers[MB_INPUT_PUMP_SPEED] = g_decoded_data.pump_speed;
    
    // Pump flow - int16_t type (already * 100)
    mb_input_registers[MB_INPUT_PUMP_FLOW] = temp_int16_to_mb(g_decoded_data.pump_flow);
    
    // Pump duty - uint8_t types (copy directly)
    mb_input_registers[MB_INPUT_PUMP_DUTY] = g_decoded_data.pump_duty;
    mb_input_registers[MB_INPUT_MAX_PUMP_DUTY] = g_decoded_data.max_pump_duty;

    // States and modes - uint8_t types (copy directly)
    mb_input_registers[MB_INPUT_HEATPUMP_STATE] = g_decoded_data.heatpump_state;
    mb_input_registers[MB_INPUT_OPERATION_MODE] = g_decoded_data.operating_mode_state;
    mb_input_registers[MB_INPUT_QUIET_MODE] = g_decoded_data.quiet_mode_level;
    mb_input_registers[MB_INPUT_POWERFUL_MODE] = g_decoded_data.powerful_mode_time;
    mb_input_registers[MB_INPUT_HOLIDAY_MODE] = g_decoded_data.holiday_mode_state;
    mb_input_registers[MB_INPUT_FORCE_DHW] = g_decoded_data.force_dhw_state;
    mb_input_registers[MB_INPUT_DEFROSTING_STATE] = g_decoded_data.defrosting_state;
    mb_input_registers[MB_INPUT_THREEWAY_VALVE_STATE] = g_decoded_data.three_way_valve_state;
    mb_input_registers[MB_INPUT_ZONES_STATE] = g_decoded_data.zones_state;
    mb_input_registers[MB_INPUT_MAIN_SCHEDULE_STATE] = g_decoded_data.main_schedule_state;
    mb_input_registers[MB_INPUT_DHW_HEATER_STATE] = g_decoded_data.dhw_heater_state;
    mb_input_registers[MB_INPUT_ROOM_HEATER_STATE] = g_decoded_data.room_heater_state;
    mb_input_registers[MB_INPUT_INTERNAL_HEATER_STATE] = g_decoded_data.internal_heater_state;

    // Zone pumps - uint8_t types (copy directly)
    mb_input_registers[MB_INPUT_Z1_WATER_PUMP] = g_decoded_data.z1_water_pump;
    mb_input_registers[MB_INPUT_Z2_WATER_PUMP] = g_decoded_data.z2_water_pump;
    mb_input_registers[MB_INPUT_POOL_WATER_PUMP] = g_decoded_data.pool_water_pump;
    mb_input_registers[MB_INPUT_SOLAR_WATER_PUMP] = g_decoded_data.solar_water_pump;
    mb_input_registers[MB_INPUT_Z1_PUMP_STATE] = g_decoded_data.z1_pump_state;
    mb_input_registers[MB_INPUT_Z2_PUMP_STATE] = g_decoded_data.z2_pump_state;
    mb_input_registers[MB_INPUT_PUMP_FLOWRATE_MODE] = g_decoded_data.pump_flowrate_mode;

    // Note: Pump state is not a direct field in g_decoded_data
    // It might need to be derived from other states or added to decoder
    mb_input_registers[MB_INPUT_PUMP_STATE] = 0; // TODO: Add proper pump state mapping

    return ESP_OK;
}

/**
 * @brief Process holding register writes (execute commands)
 * @param reg_addr Register address that was written
 * @return ESP_OK on success
 */
esp_err_t modbus_params_process_holding_write(uint16_t reg_addr) {
    esp_err_t ret = ESP_OK;
    uint16_t value = mb_holding_registers[reg_addr - MB_REG_HOLDING_START];
    
    ESP_LOGI(TAG, "Processing write to register 0x%04X, value: %u", reg_addr, value);

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

        // Temperature setpoints (value is in °C * 100, need to convert to °C)
        case MB_HOLDING_SET_Z1_HEAT_TEMP:
            ret = set_z1_heat_request_temperature((int8_t)(value / 100));
            break;
            
        case MB_HOLDING_SET_Z1_COOL_TEMP:
            ret = set_z1_cool_request_temperature((int8_t)(value / 100));
            break;
            
        case MB_HOLDING_SET_Z2_HEAT_TEMP:
            ret = set_z2_heat_request_temperature((int8_t)(value / 100));
            break;
            
        case MB_HOLDING_SET_Z2_COOL_TEMP:
            ret = set_z2_cool_request_temperature((int8_t)(value / 100));
            break;
            
        case MB_HOLDING_SET_DHW_TEMP:
            ret = set_DHW_temp((int8_t)(value / 100));
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

