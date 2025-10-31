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
#include <string.h>

static const char *TAG = "MODBUS_PARAMS";

// Register arrays
// Note: Modbus registers are 16-bit values. They can be interpreted as signed or unsigned
// by the Modbus client. We use int16_t for values that can be negative (temperatures, etc.)
int16_t mb_input_registers[MB_REG_INPUT_COUNT] = {0};
int16_t mb_holding_registers[MB_REG_HOLDING_COUNT] = {0};

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
 * @brief Helper function to copy string to Modbus registers (2 bytes per register)
 * @param dest_reg Start register index
 * @param src Source string
 * @param max_len Maximum string length
 */
static void copy_string_to_registers(uint16_t dest_reg, const char *src, size_t max_len) {
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
    // Check if decoded data is valid
    if (!g_decoded_data.data_valid) {
        ESP_LOGD(TAG, "Decoded data not valid, skipping register update");
        return ESP_ERR_INVALID_STATE;
    }

    // ========================================================================
    // System information (0x0000-0x000F)
    // ========================================================================
    mb_input_registers[MB_INPUT_UPTIME_LOW] = (int16_t)(g_decoded_data.last_update_time & 0xFFFF);
    mb_input_registers[MB_INPUT_UPTIME_HIGH] = (int16_t)(g_decoded_data.last_update_time >> 16);
    mb_input_registers[MB_INPUT_STATUS] = (int16_t)g_decoded_data.heatpump_state;

    // ========================================================================
    // Basic temperatures (0x0010-0x002F)
    // ========================================================================
    // int16_t types (already * 100)
    mb_input_registers[MB_INPUT_MAIN_INLET_TEMP] = g_decoded_data.main_inlet_temp;
    mb_input_registers[MB_INPUT_MAIN_OUTLET_TEMP] = g_decoded_data.main_outlet_temp;
    
    // int8_t types (whole degrees)
    mb_input_registers[MB_INPUT_MAIN_TARGET_TEMP] = (int16_t)g_decoded_data.main_target_temp;
    mb_input_registers[MB_INPUT_DHW_TEMP] = (int16_t)g_decoded_data.dhw_temp;
    mb_input_registers[MB_INPUT_DHW_TARGET_TEMP] = (int16_t)g_decoded_data.dhw_target_temp;
    mb_input_registers[MB_INPUT_OUTSIDE_TEMP] = (int16_t)g_decoded_data.outside_temp;
    mb_input_registers[MB_INPUT_ROOM_THERMOSTAT_TEMP] = (int16_t)g_decoded_data.room_thermostat_temp;
    mb_input_registers[MB_INPUT_BUFFER_TEMP] = (int16_t)g_decoded_data.buffer_temp;
    mb_input_registers[MB_INPUT_SOLAR_TEMP] = (int16_t)g_decoded_data.solar_temp;
    mb_input_registers[MB_INPUT_POOL_TEMP] = (int16_t)g_decoded_data.pool_temp;

    // ========================================================================
    // Additional temperatures (0x0020-0x003F)
    // ========================================================================
    mb_input_registers[MB_INPUT_MAIN_HEX_OUTLET_TEMP] = (int16_t)g_decoded_data.main_hex_outlet_temp;
    mb_input_registers[MB_INPUT_DISCHARGE_TEMP] = (int16_t)g_decoded_data.discharge_temp;
    mb_input_registers[MB_INPUT_INSIDE_PIPE_TEMP] = (int16_t)g_decoded_data.inside_pipe_temp;
    mb_input_registers[MB_INPUT_DEFROST_TEMP] = (int16_t)g_decoded_data.defrost_temp;
    mb_input_registers[MB_INPUT_EVA_OUTLET_TEMP] = (int16_t)g_decoded_data.eva_outlet_temp;
    mb_input_registers[MB_INPUT_BYPASS_OUTLET_TEMP] = (int16_t)g_decoded_data.bypass_outlet_temp;
    mb_input_registers[MB_INPUT_IPM_TEMP] = (int16_t)g_decoded_data.ipm_temp;
    mb_input_registers[MB_INPUT_OUTSIDE_PIPE_TEMP] = (int16_t)g_decoded_data.outside_pipe_temp;
    mb_input_registers[MB_INPUT_Z1_ROOM_TEMP] = (int16_t)g_decoded_data.z1_temp;
    mb_input_registers[MB_INPUT_Z2_ROOM_TEMP] = (int16_t)g_decoded_data.z2_temp;
    mb_input_registers[MB_INPUT_Z1_WATER_TEMP] = (int16_t)g_decoded_data.z1_water_temp;
    mb_input_registers[MB_INPUT_Z2_WATER_TEMP] = (int16_t)g_decoded_data.z2_water_temp;
    mb_input_registers[MB_INPUT_Z1_WATER_TARGET_TEMP] = (int16_t)g_decoded_data.z1_water_target_temp;
    mb_input_registers[MB_INPUT_Z2_WATER_TARGET_TEMP] = (int16_t)g_decoded_data.z2_water_target_temp;
    mb_input_registers[MB_INPUT_SECOND_INLET_TEMP] = (int16_t)g_decoded_data.second_inlet_temp;
    mb_input_registers[MB_INPUT_ECONOMIZER_OUTLET_TEMP] = (int16_t)g_decoded_data.economizer_outlet_temp;
    mb_input_registers[MB_INPUT_SECOND_ROOM_THERMO_TEMP] = (int16_t)g_decoded_data.second_room_thermostat_temp;

    // ========================================================================
    // Zone temperature requests (0x0040-0x004F)
    // ========================================================================
    mb_input_registers[MB_INPUT_Z1_HEAT_REQUEST_TEMP] = (int16_t)g_decoded_data.z1_heat_request_temp;
    mb_input_registers[MB_INPUT_Z1_COOL_REQUEST_TEMP] = (int16_t)g_decoded_data.z1_cool_request_temp;
    mb_input_registers[MB_INPUT_Z2_HEAT_REQUEST_TEMP] = (int16_t)g_decoded_data.z2_heat_request_temp;
    mb_input_registers[MB_INPUT_Z2_COOL_REQUEST_TEMP] = (int16_t)g_decoded_data.z2_cool_request_temp;

    // ========================================================================
    // Zone 1 heating curve (0x0050-0x0053)
    // ========================================================================
    mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_TARGET_HIGH] = (int16_t)g_decoded_data.z1_heat_curve_target_high_temp;
    mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_TARGET_LOW] = (int16_t)g_decoded_data.z1_heat_curve_target_low_temp;
    mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_HIGH] = (int16_t)g_decoded_data.z1_heat_curve_outside_high_temp;
    mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_LOW] = (int16_t)g_decoded_data.z1_heat_curve_outside_low_temp;

    // Zone 1 cooling curve (0x0054-0x0057)
    mb_input_registers[MB_INPUT_Z1_COOL_CURVE_TARGET_HIGH] = (int16_t)g_decoded_data.z1_cool_curve_target_high_temp;
    mb_input_registers[MB_INPUT_Z1_COOL_CURVE_TARGET_LOW] = (int16_t)g_decoded_data.z1_cool_curve_target_low_temp;
    mb_input_registers[MB_INPUT_Z1_COOL_CURVE_OUTSIDE_HIGH] = (int16_t)g_decoded_data.z1_cool_curve_outside_high_temp;
    mb_input_registers[MB_INPUT_Z1_COOL_CURVE_OUTSIDE_LOW] = (int16_t)g_decoded_data.z1_cool_curve_outside_low_temp;

    // ========================================================================
    // Zone 2 heating curve (0x0060-0x0063)
    // ========================================================================
    mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_TARGET_HIGH] = (int16_t)g_decoded_data.z2_heat_curve_target_high_temp;
    mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_TARGET_LOW] = (int16_t)g_decoded_data.z2_heat_curve_target_low_temp;
    mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_HIGH] = (int16_t)g_decoded_data.z2_heat_curve_outside_high_temp;
    mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_LOW] = (int16_t)g_decoded_data.z2_heat_curve_outside_low_temp;

    // Zone 2 cooling curve (0x0064-0x0067)
    mb_input_registers[MB_INPUT_Z2_COOL_CURVE_TARGET_HIGH] = (int16_t)g_decoded_data.z2_cool_curve_target_high_temp;
    mb_input_registers[MB_INPUT_Z2_COOL_CURVE_TARGET_LOW] = (int16_t)g_decoded_data.z2_cool_curve_target_low_temp;
    mb_input_registers[MB_INPUT_Z2_COOL_CURVE_OUTSIDE_HIGH] = (int16_t)g_decoded_data.z2_cool_curve_outside_high_temp;
    mb_input_registers[MB_INPUT_Z2_COOL_CURVE_OUTSIDE_LOW] = (int16_t)g_decoded_data.z2_cool_curve_outside_low_temp;

    // ========================================================================
    // Power and energy (0x0070-0x007F)
    // ========================================================================
    mb_input_registers[MB_INPUT_HEAT_POWER_PRODUCTION] = (int16_t)g_decoded_data.heat_power_production;
    mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION] = (int16_t)g_decoded_data.heat_power_consumption;
    mb_input_registers[MB_INPUT_COOL_POWER_PRODUCTION] = (int16_t)g_decoded_data.cool_power_production;
    mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION] = (int16_t)g_decoded_data.cool_power_consumption;
    mb_input_registers[MB_INPUT_DHW_POWER_PRODUCTION] = (int16_t)g_decoded_data.dhw_power_production;
    mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION] = (int16_t)g_decoded_data.dhw_power_consumption;

    // Extra power data (XTOP0-5)
    mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION_EXTRA] = (int16_t)g_decoded_data.heat_power_consumption_extra;
    mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION_EXTRA] = (int16_t)g_decoded_data.cool_power_consumption_extra;
    mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION_EXTRA] = (int16_t)g_decoded_data.dhw_power_consumption_extra;
    mb_input_registers[MB_INPUT_HEAT_POWER_PRODUCTION_EXTRA] = (int16_t)g_decoded_data.heat_power_production_extra;
    mb_input_registers[MB_INPUT_COOL_POWER_PRODUCTION_EXTRA] = (int16_t)g_decoded_data.cool_power_production_extra;
    mb_input_registers[MB_INPUT_DHW_POWER_PRODUCTION_EXTRA] = (int16_t)g_decoded_data.dhw_power_production_extra;

    // ========================================================================
    // Technical parameters (0x0080-0x009F)
    // ========================================================================
    mb_input_registers[MB_INPUT_COMPRESSOR_FREQ] = (int16_t)g_decoded_data.compressor_freq;
    mb_input_registers[MB_INPUT_PUMP_FLOW] = g_decoded_data.pump_flow;  // int16 (*100)
    mb_input_registers[MB_INPUT_OPERATIONS_HOURS] = (int16_t)g_decoded_data.operations_hours;
    mb_input_registers[MB_INPUT_OPERATIONS_COUNTER] = (int16_t)g_decoded_data.operations_counter;
    mb_input_registers[MB_INPUT_FAN1_MOTOR_SPEED] = (int16_t)g_decoded_data.fan1_motor_speed;
    mb_input_registers[MB_INPUT_FAN2_MOTOR_SPEED] = (int16_t)g_decoded_data.fan2_motor_speed;
    mb_input_registers[MB_INPUT_HIGH_PRESSURE] = g_decoded_data.high_pressure;  // int16 (*100)
    mb_input_registers[MB_INPUT_PUMP_SPEED] = (int16_t)g_decoded_data.pump_speed;
    mb_input_registers[MB_INPUT_LOW_PRESSURE] = g_decoded_data.low_pressure;  // int16 (*100)
    mb_input_registers[MB_INPUT_COMPRESSOR_CURRENT] = g_decoded_data.compressor_current;  // int16 (*100)
    mb_input_registers[MB_INPUT_PUMP_DUTY] = (int16_t)g_decoded_data.pump_duty;
    mb_input_registers[MB_INPUT_MAX_PUMP_DUTY] = (int16_t)g_decoded_data.max_pump_duty;

    // ========================================================================
    // Operation states (0x00A0-0x00AF)
    // ========================================================================
    mb_input_registers[MB_INPUT_HEATPUMP_STATE] = (int16_t)g_decoded_data.heatpump_state;
    mb_input_registers[MB_INPUT_FORCE_DHW_STATE] = (int16_t)g_decoded_data.force_dhw_state;
    mb_input_registers[MB_INPUT_OPERATING_MODE_STATE] = (int16_t)g_decoded_data.operating_mode_state;
    mb_input_registers[MB_INPUT_QUIET_MODE_SCHEDULE] = (int16_t)g_decoded_data.quiet_mode_schedule;
    mb_input_registers[MB_INPUT_POWERFUL_MODE_TIME] = (int16_t)g_decoded_data.powerful_mode_time;
    mb_input_registers[MB_INPUT_QUIET_MODE_LEVEL] = (int16_t)g_decoded_data.quiet_mode_level;
    mb_input_registers[MB_INPUT_HOLIDAY_MODE_STATE] = (int16_t)g_decoded_data.holiday_mode_state;
    mb_input_registers[MB_INPUT_THREE_WAY_VALVE_STATE] = (int16_t)g_decoded_data.three_way_valve_state;
    mb_input_registers[MB_INPUT_DEFROSTING_STATE] = (int16_t)g_decoded_data.defrosting_state;
    mb_input_registers[MB_INPUT_MAIN_SCHEDULE_STATE] = (int16_t)g_decoded_data.main_schedule_state;
    mb_input_registers[MB_INPUT_ZONES_STATE] = (int16_t)g_decoded_data.zones_state;

    // ========================================================================
    // Heaters and sterilization (0x00B0-0x00BF)
    // ========================================================================
    mb_input_registers[MB_INPUT_DHW_HEATER_STATE] = (int16_t)g_decoded_data.dhw_heater_state;
    mb_input_registers[MB_INPUT_ROOM_HEATER_STATE] = (int16_t)g_decoded_data.room_heater_state;
    mb_input_registers[MB_INPUT_INTERNAL_HEATER_STATE] = (int16_t)g_decoded_data.internal_heater_state;
    mb_input_registers[MB_INPUT_EXTERNAL_HEATER_STATE] = (int16_t)g_decoded_data.external_heater_state;
    mb_input_registers[MB_INPUT_FORCE_HEATER_STATE] = (int16_t)g_decoded_data.force_heater_state;
    mb_input_registers[MB_INPUT_STERILIZATION_STATE] = (int16_t)g_decoded_data.sterilization_state;
    mb_input_registers[MB_INPUT_STERILIZATION_TEMP] = g_decoded_data.sterilization_temp;  // int16 (*100)
    mb_input_registers[MB_INPUT_STERILIZATION_MAX_TIME] = (int16_t)g_decoded_data.sterilization_max_time;

    // ========================================================================
    // Deltas and shifts (0x00C0-0x00CF) - all int16 (*100)
    // ========================================================================
    mb_input_registers[MB_INPUT_DHW_HEAT_DELTA] = g_decoded_data.dhw_heat_delta;
    mb_input_registers[MB_INPUT_HEAT_DELTA] = g_decoded_data.heat_delta;
    mb_input_registers[MB_INPUT_COOL_DELTA] = g_decoded_data.cool_delta;
    mb_input_registers[MB_INPUT_DHW_HOLIDAY_SHIFT_TEMP] = g_decoded_data.dhw_holiday_shift_temp;
    mb_input_registers[MB_INPUT_ROOM_HOLIDAY_SHIFT_TEMP] = g_decoded_data.room_holiday_shift_temp;
    mb_input_registers[MB_INPUT_BUFFER_TANK_DELTA] = g_decoded_data.buffer_tank_delta;

    // ========================================================================
    // Heating/Cooling mode settings (0x00D0-0x00DF)
    // ========================================================================
    mb_input_registers[MB_INPUT_HEATING_MODE] = (int16_t)g_decoded_data.heating_mode;
    mb_input_registers[MB_INPUT_HEATING_OFF_OUTDOOR_TEMP] = g_decoded_data.heating_off_outdoor_temp;  // int16 (*100)
    mb_input_registers[MB_INPUT_HEATER_ON_OUTDOOR_TEMP] = g_decoded_data.heater_on_outdoor_temp;  // int16 (*100)
    mb_input_registers[MB_INPUT_HEAT_TO_COOL_TEMP] = g_decoded_data.heat_to_cool_temp;  // int16 (*100)
    mb_input_registers[MB_INPUT_COOL_TO_HEAT_TEMP] = g_decoded_data.cool_to_heat_temp;  // int16 (*100)
    mb_input_registers[MB_INPUT_COOLING_MODE] = (int16_t)g_decoded_data.cooling_mode;

    // ========================================================================
    // Solar and buffer settings (0x00E0-0x00EF)
    // ========================================================================
    mb_input_registers[MB_INPUT_BUFFER_INSTALLED] = (int16_t)g_decoded_data.buffer_installed;
    mb_input_registers[MB_INPUT_DHW_INSTALLED] = (int16_t)g_decoded_data.dhw_installed;
    mb_input_registers[MB_INPUT_SOLAR_MODE] = (int16_t)g_decoded_data.solar_mode;
    mb_input_registers[MB_INPUT_SOLAR_ON_DELTA] = g_decoded_data.solar_on_delta;  // int16 (*100)
    mb_input_registers[MB_INPUT_SOLAR_OFF_DELTA] = g_decoded_data.solar_off_delta;  // int16 (*100)
    mb_input_registers[MB_INPUT_SOLAR_FROST_PROTECTION] = g_decoded_data.solar_frost_protection;  // int16 (*100)
    mb_input_registers[MB_INPUT_SOLAR_HIGH_LIMIT] = g_decoded_data.solar_high_limit;  // int16 (*100)

    // ========================================================================
    // Pump and liquid settings (0x00F0-0x00FF)
    // ========================================================================
    mb_input_registers[MB_INPUT_PUMP_FLOWRATE_MODE] = (int16_t)g_decoded_data.pump_flowrate_mode;
    mb_input_registers[MB_INPUT_LIQUID_TYPE] = (int16_t)g_decoded_data.liquid_type;
    mb_input_registers[MB_INPUT_ALT_EXTERNAL_SENSOR] = (int16_t)g_decoded_data.alt_external_sensor;
    mb_input_registers[MB_INPUT_ANTI_FREEZE_MODE] = (int16_t)g_decoded_data.anti_freeze_mode;
    mb_input_registers[MB_INPUT_OPTIONAL_PCB] = (int16_t)g_decoded_data.optional_pcb;
    mb_input_registers[MB_INPUT_Z1_SENSOR_SETTINGS] = (int16_t)g_decoded_data.z1_sensor_settings;
    mb_input_registers[MB_INPUT_Z2_SENSOR_SETTINGS] = (int16_t)g_decoded_data.z2_sensor_settings;

    // ========================================================================
    // External controls (0x0100-0x010F)
    // ========================================================================
    mb_input_registers[MB_INPUT_EXTERNAL_PAD_HEATER] = (int16_t)g_decoded_data.external_pad_heater;
    mb_input_registers[MB_INPUT_WATER_PRESSURE] = g_decoded_data.water_pressure;  // int16 (*100)
    mb_input_registers[MB_INPUT_EXTERNAL_CONTROL] = (int16_t)g_decoded_data.external_control;
    mb_input_registers[MB_INPUT_EXTERNAL_HEAT_COOL_CONTROL] = (int16_t)g_decoded_data.external_heat_cool_control;
    mb_input_registers[MB_INPUT_EXTERNAL_ERROR_SIGNAL] = (int16_t)g_decoded_data.external_error_signal;
    mb_input_registers[MB_INPUT_EXTERNAL_COMPRESSOR_CONTROL] = (int16_t)g_decoded_data.external_compressor_control;

    // ========================================================================
    // Pump and valve states (0x0110-0x011F)
    // ========================================================================
    mb_input_registers[MB_INPUT_Z2_PUMP_STATE] = (int16_t)g_decoded_data.z2_pump_state;
    mb_input_registers[MB_INPUT_Z1_PUMP_STATE] = (int16_t)g_decoded_data.z1_pump_state;
    mb_input_registers[MB_INPUT_TWO_WAY_VALVE_STATE] = (int16_t)g_decoded_data.two_way_valve_state;
    mb_input_registers[MB_INPUT_THREE_WAY_VALVE_STATE2] = (int16_t)g_decoded_data.three_way_valve_state2;
    mb_input_registers[MB_INPUT_Z1_VALVE_PID] = g_decoded_data.z1_valve_pid;  // int16 (*100)
    mb_input_registers[MB_INPUT_Z2_VALVE_PID] = g_decoded_data.z2_valve_pid;  // int16 (*100)

    // ========================================================================
    // Bivalent settings (0x0120-0x012F)
    // ========================================================================
    mb_input_registers[MB_INPUT_BIVALENT_CONTROL] = (int16_t)g_decoded_data.bivalent_control;
    mb_input_registers[MB_INPUT_BIVALENT_MODE] = (int16_t)g_decoded_data.bivalent_mode;
    mb_input_registers[MB_INPUT_BIVALENT_START_TEMP] = g_decoded_data.bivalent_start_temp;  // int16 (*100)
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_HEAT] = (int16_t)g_decoded_data.bivalent_advanced_heat;
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_DHW] = (int16_t)g_decoded_data.bivalent_advanced_dhw;
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_START_TEMP] = g_decoded_data.bivalent_advanced_start_temp;  // int16 (*100)
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_STOP_TEMP] = g_decoded_data.bivalent_advanced_stop_temp;  // int16 (*100)
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_START_DELAY] = (int16_t)g_decoded_data.bivalent_advanced_start_delay;
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_STOP_DELAY] = (int16_t)g_decoded_data.bivalent_advanced_stop_delay;
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_DHW_DELAY] = (int16_t)g_decoded_data.bivalent_advanced_dhw_delay;

    // ========================================================================
    // Heater timing settings (0x0130-0x013F)
    // ========================================================================
    mb_input_registers[MB_INPUT_HEATER_DELAY_TIME] = (int16_t)g_decoded_data.heater_delay_time;
    mb_input_registers[MB_INPUT_HEATER_START_DELTA] = g_decoded_data.heater_start_delta;  // int16 (*100)
    mb_input_registers[MB_INPUT_HEATER_STOP_DELTA] = g_decoded_data.heater_stop_delta;  // int16 (*100)

    // ========================================================================
    // Error state (0x0140-0x0147) - 16 bytes in 8 registers
    // ========================================================================
    copy_string_to_registers(MB_INPUT_ERROR_STATE_0, g_decoded_data.error_state, 16);

    // ========================================================================
    // Heat pump model (0x0148-0x0157) - 32 bytes in 16 registers
    // ========================================================================
    copy_string_to_registers(MB_INPUT_HP_MODEL_0, g_decoded_data.heat_pump_model, 32);

    // ========================================================================
    // Operation hours (0x0158-0x015F)
    // ========================================================================
    mb_input_registers[MB_INPUT_ROOM_HEATER_OPS_HOURS] = (int16_t)g_decoded_data.room_heater_operations_hours;
    mb_input_registers[MB_INPUT_DHW_HEATER_OPS_HOURS] = (int16_t)g_decoded_data.dhw_heater_operations_hours;

    // ========================================================================
    // Optional PCB data (0x0160-0x016F)
    // ========================================================================
    mb_input_registers[MB_INPUT_Z1_WATER_PUMP] = (int16_t)g_decoded_data.z1_water_pump;
    mb_input_registers[MB_INPUT_Z1_MIXING_VALVE] = (int16_t)g_decoded_data.z1_mixing_valve;
    mb_input_registers[MB_INPUT_Z2_WATER_PUMP] = (int16_t)g_decoded_data.z2_water_pump;
    mb_input_registers[MB_INPUT_Z2_MIXING_VALVE] = (int16_t)g_decoded_data.z2_mixing_valve;
    mb_input_registers[MB_INPUT_POOL_WATER_PUMP] = (int16_t)g_decoded_data.pool_water_pump;
    mb_input_registers[MB_INPUT_SOLAR_WATER_PUMP] = (int16_t)g_decoded_data.solar_water_pump;
    mb_input_registers[MB_INPUT_ALARM_STATE] = (int16_t)g_decoded_data.alarm_state;

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
