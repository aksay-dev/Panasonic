/**
 * @file commands.c
 * @brief Heat pump control commands implementation
 * @version 0.1.0
 * @date 2025
 */

#include "commands.h"
#include "protocol.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

// Protocol data offsets for main commands
#define CMD_OFFSET_HEATPUMP_STATE        4
#define CMD_OFFSET_PUMP_STATE            4
#define CMD_OFFSET_HOLIDAY_MODE          5
#define CMD_OFFSET_MAIN_SCHEDULE         5
#define CMD_OFFSET_OPERATION_MODE        6
#define CMD_OFFSET_ZONES                 6
#define CMD_OFFSET_QUIET_MODE            7
#define CMD_OFFSET_POWERFUL_MODE         7
#define CMD_OFFSET_FORCE_DEFROST         8
#define CMD_OFFSET_FORCE_STERILIZATION   8
#define CMD_OFFSET_RESET                 8
#define CMD_OFFSET_ALT_EXTERNAL_SENSOR   20
#define CMD_OFFSET_EXTERNAL_CONTROL      23
#define CMD_OFFSET_EXTERNAL_HEAT_COOL    23
#define CMD_OFFSET_EXTERNAL_ERROR        23
#define CMD_OFFSET_EXTERNAL_COMPRESSOR   23
#define CMD_OFFSET_BUFFER                24
#define CMD_OFFSET_EXTERNAL_PAD_HEATER   25
#define CMD_OFFSET_BIVALENT_CONTROL      26
#define CMD_OFFSET_BIVALENT_MODE         26
#define CMD_OFFSET_Z1_HEAT_TEMP          38
#define CMD_OFFSET_Z1_COOL_TEMP          39
#define CMD_OFFSET_Z2_HEAT_TEMP          40
#define CMD_OFFSET_Z2_COOL_TEMP          41
#define CMD_OFFSET_DHW_TEMP              42
#define CMD_OFFSET_MAX_PUMP_DUTY         45
#define CMD_OFFSET_BUFFER_DELTA          59
#define CMD_OFFSET_BIVALENT_START        65
#define CMD_OFFSET_BIVALENT_AP_START     66
#define CMD_OFFSET_BIVALENT_AP_STOP      68
#define CMD_OFFSET_HEATING_OFF_TEMP      83
#define CMD_OFFSET_FLOOR_HEAT_DELTA      84
#define CMD_OFFSET_FLOOR_COOL_DELTA      94
#define CMD_OFFSET_DHW_HEAT_DELTA        99
#define CMD_OFFSET_HEATER_DELAY_TIME     104
#define CMD_OFFSET_HEATER_START_DELTA    105
#define CMD_OFFSET_HEATER_STOP_DELTA     106

// Protocol data offsets for curves
#define CMD_OFFSET_CURVES_START_1        75
#define CMD_OFFSET_CURVES_START_2        86
#define CMD_CURVES_COUNT_1               8
#define CMD_CURVES_COUNT_2               8

// Protocol data offsets for optional PCB commands
#define OPT_OFFSET_BYTE_6                6
#define OPT_OFFSET_POOL_TEMP             7
#define OPT_OFFSET_BUFFER_TEMP           8
#define OPT_OFFSET_BYTE_9                9
#define OPT_OFFSET_Z1_ROOM_TEMP          10
#define OPT_OFFSET_Z2_ROOM_TEMP          11
#define OPT_OFFSET_SOLAR_TEMP            13
#define OPT_OFFSET_DEMAND_CONTROL        14
#define OPT_OFFSET_Z2_WATER_TEMP         15
#define OPT_OFFSET_Z1_WATER_TEMP         16

// Command values
#define CMD_VALUE_HEATPUMP_OFF           1
#define CMD_VALUE_HEATPUMP_ON            2
#define CMD_VALUE_PUMP_OFF               16
#define CMD_VALUE_PUMP_ON                32
#define CMD_VALUE_FORCE_DHW_OFF          64
#define CMD_VALUE_FORCE_DHW_ON           128
#define CMD_VALUE_FORCE_DEFROST_OFF      0
#define CMD_VALUE_FORCE_DEFROST_ON       2
#define CMD_VALUE_FORCE_STERILIZATION_OFF 0
#define CMD_VALUE_FORCE_STERILIZATION_ON 4
#define CMD_VALUE_HOLIDAY_OFF            16
#define CMD_VALUE_HOLIDAY_ON             32
#define CMD_VALUE_MAIN_SCHEDULE_OFF      64
#define CMD_VALUE_MAIN_SCHEDULE_ON       128
#define CMD_VALUE_ALT_SENSOR_OFF         16
#define CMD_VALUE_ALT_SENSOR_ON          32
#define CMD_VALUE_BUFFER_OFF             4
#define CMD_VALUE_BUFFER_ON              8
#define CMD_VALUE_EXTERNAL_CONTROL_OFF   1
#define CMD_VALUE_EXTERNAL_CONTROL_ON    2
#define CMD_VALUE_EXTERNAL_HEAT_COOL_OFF 4
#define CMD_VALUE_EXTERNAL_HEAT_COOL_ON  8
#define CMD_VALUE_EXTERNAL_ERROR_OFF     16
#define CMD_VALUE_EXTERNAL_ERROR_ON      32
#define CMD_VALUE_EXTERNAL_COMPRESSOR_OFF 64
#define CMD_VALUE_EXTERNAL_COMPRESSOR_ON 128
#define CMD_VALUE_BIVALENT_CONTROL_OFF   1
#define CMD_VALUE_BIVALENT_CONTROL_ON    2
#define CMD_VALUE_BIVALENT_MODE_ALT      4
#define CMD_VALUE_BIVALENT_MODE_PARALLEL 8
#define CMD_VALUE_BIVALENT_MODE_ADV_PARALLEL 12
#define CMD_VALUE_ZONES_ALL_OFF          64
#define CMD_VALUE_ZONES_Z1_ON            128
#define CMD_VALUE_ZONES_Z2_ON            192
#define CMD_VALUE_ZONES_ALL_ON           255
#define CMD_VALUE_EXTERNAL_PAD_OFF       16
#define CMD_VALUE_EXTERNAL_PAD_ON        32
#define CMD_VALUE_EXTERNAL_PAD_AUTO      48

// Temperature offset for protocol
#define CMD_TEMP_OFFSET                  128

// Temperature conversion utility
uint32_t temp2hex(float temp) {
    int hextemp = 0;
    if (temp > 120) {
        hextemp = 0;
    } else if (temp < -78) {
        hextemp = 255;
    } else {
        uint8_t Uref = 255;
        int constant = 3695;
        int R25 = 6340;
        uint8_t T25 = 25;
        int Rf = 6480;
        float K = 273.15;
        float RT = R25 * exp(constant * (1 / (temp + K) - 1 / (T25 + K)));
        hextemp = Uref * (RT / (Rf + RT));
    }
    return hextemp;
}

/**
 * @brief Create a write command for the heat pump protocol
 * @param data_offset Offset in the data array where to place the value
 * @param value Value to write
 * @return Protocol command structure
 */
static esp_err_t send_command(uint8_t data_offset, uint8_t value) {
    protocol_cmd_t cmd = {0};
    cmd.len = PROTOCOL_WRITE_SIZE;
    cmd.data[0] = PROTOCOL_PKT_WRITE;
    cmd.data[1] = 0x6c;
    cmd.data[2] = 0x01;
    cmd.data[3] = PROTOCOL_DATA_MAIN;
    cmd.data[data_offset] = value;
    return protocol_send_command(&cmd);
}

static esp_err_t temperature_send_command(uint8_t data_offset, int8_t temperature) {
    return send_command(data_offset, temperature + CMD_TEMP_OFFSET);
}

esp_err_t set_heatpump_state(bool state) {
    const uint8_t value = state ? CMD_VALUE_HEATPUMP_ON : CMD_VALUE_HEATPUMP_OFF;
    return send_command(CMD_OFFSET_HEATPUMP_STATE, value);
}

esp_err_t set_pump(bool state) {
    const uint8_t value = state ? CMD_VALUE_PUMP_ON : CMD_VALUE_PUMP_OFF;
    return send_command(CMD_OFFSET_PUMP_STATE, value);
}

esp_err_t set_max_pump_duty(uint8_t duty) {
    const uint8_t value = duty + 1;
    return send_command(CMD_OFFSET_MAX_PUMP_DUTY, value);
}

esp_err_t set_quiet_mode(uint8_t mode) {
    const uint8_t value = (mode + 1) * 8;
    return send_command(CMD_OFFSET_QUIET_MODE, value);
}    

esp_err_t set_z1_heat_request_temperature(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_Z1_HEAT_TEMP, temperature);
}

esp_err_t set_z1_cool_request_temperature(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_Z1_COOL_TEMP, temperature);
}

esp_err_t set_z2_heat_request_temperature(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_Z2_HEAT_TEMP, temperature);
}

esp_err_t set_z2_cool_request_temperature(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_Z2_COOL_TEMP, temperature);
}

esp_err_t set_bivalent_start_temp(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_BIVALENT_START, temperature);
}

esp_err_t set_bivalent_ap_start_temp(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_BIVALENT_AP_START, temperature);
}

esp_err_t set_bivalent_ap_stop_temp(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_BIVALENT_AP_STOP, temperature);
}

esp_err_t set_force_DHW(bool state) {
    const uint8_t value = state ? CMD_VALUE_FORCE_DHW_ON : CMD_VALUE_FORCE_DHW_OFF;
    return send_command(CMD_OFFSET_HEATPUMP_STATE, value);
}

esp_err_t set_force_defrost(bool state) {
    const uint8_t value = state ? CMD_VALUE_FORCE_DEFROST_ON : CMD_VALUE_FORCE_DEFROST_OFF;
    return send_command(CMD_OFFSET_FORCE_DEFROST, value);
}

esp_err_t set_force_sterilization(bool state) {
    const uint8_t value = state ? CMD_VALUE_FORCE_STERILIZATION_ON : CMD_VALUE_FORCE_STERILIZATION_OFF;
    return send_command(CMD_OFFSET_FORCE_STERILIZATION, value);
}

esp_err_t set_holiday_mode(bool state) {
    const uint8_t value = state ? CMD_VALUE_HOLIDAY_ON : CMD_VALUE_HOLIDAY_OFF;
    return send_command(CMD_OFFSET_HOLIDAY_MODE, value);
}

esp_err_t set_powerful_mode(uint8_t mode) {
    const uint8_t value = mode + 73;
    return send_command(CMD_OFFSET_POWERFUL_MODE, value);
}

esp_err_t set_DHW_temp(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_DHW_TEMP, temperature);
}

esp_err_t set_curves(const uint8_t *curves) {
    protocol_cmd_t cmd = {0};
    cmd.len = PROTOCOL_WRITE_SIZE;
    cmd.data[0] = PROTOCOL_PKT_WRITE;
    cmd.data[1] = 0x6c;
    cmd.data[2] = 0x01;
    cmd.data[3] = PROTOCOL_DATA_MAIN;
    for (int i = 0; i < CMD_CURVES_COUNT_1; i++) {
        cmd.data[i + CMD_OFFSET_CURVES_START_1] = curves[i] + CMD_TEMP_OFFSET;
    }
    for (int i = CMD_CURVES_COUNT_1; i < CMD_CURVES_COUNT_1 + CMD_CURVES_COUNT_2; i++) {
        cmd.data[i + CMD_OFFSET_CURVES_START_2] = curves[i] + CMD_TEMP_OFFSET;
    }
    return protocol_send_command(&cmd);
}

esp_err_t set_operation_mode(uint8_t mode) {
    uint8_t value;
    switch (mode) {
        case 0: value = 18; break;
        case 1: value = 19; break;
        case 2: value = 24; break;
        case 3: value = 33; break;
        case 4: value = 34; break;
        case 5: value = 35; break;
        case 6: value = 40; break;
        default: value = 0; break; // default to off
    }
    return send_command(CMD_OFFSET_OPERATION_MODE, value);
}

esp_err_t set_bivalent_control(bool state) {
    const uint8_t value = state ? CMD_VALUE_BIVALENT_CONTROL_ON : CMD_VALUE_BIVALENT_CONTROL_OFF;
    return send_command(CMD_OFFSET_BIVALENT_CONTROL, value);
}

esp_err_t set_bivalent_mode(uint8_t mode) {
    uint8_t value;
    switch (mode) {
        case 1: value = CMD_VALUE_BIVALENT_MODE_PARALLEL; break;
        case 2: value = CMD_VALUE_BIVALENT_MODE_ADV_PARALLEL; break;
        default: value = CMD_VALUE_BIVALENT_MODE_ALT; break;
    }
    return send_command(CMD_OFFSET_BIVALENT_MODE, value);
}

esp_err_t set_zones(uint8_t mode) {
    uint8_t value;
    switch (mode) {
        case 1: value = CMD_VALUE_ZONES_Z1_ON; break;
        case 2: value = CMD_VALUE_ZONES_Z2_ON; break;
        default: value = CMD_VALUE_ZONES_ALL_OFF; break;
    }
    return send_command(CMD_OFFSET_ZONES, value);
}

esp_err_t set_floor_heat_delta(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_FLOOR_HEAT_DELTA, temperature);
}

esp_err_t set_floor_cool_delta(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_FLOOR_COOL_DELTA, temperature);
}

esp_err_t set_dhw_heat_delta(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_DHW_HEAT_DELTA, temperature);
}

esp_err_t set_reset(bool state) {
    return send_command(CMD_OFFSET_RESET, state ? 1 : 0);
}

esp_err_t set_heater_delay_time(uint8_t time) {
    const uint8_t value = time + 1;
    return send_command(CMD_OFFSET_HEATER_DELAY_TIME, value);
}

esp_err_t set_heater_start_delta(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_HEATER_START_DELTA, temperature);
}

esp_err_t set_heater_stop_delta(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_HEATER_STOP_DELTA, temperature);
}

esp_err_t set_main_schedule(bool state) {
    const uint8_t value = state ? CMD_VALUE_MAIN_SCHEDULE_ON : CMD_VALUE_MAIN_SCHEDULE_OFF;
    return send_command(CMD_OFFSET_MAIN_SCHEDULE, value);
}

esp_err_t set_alt_external_sensor(bool state) {
    const uint8_t value = state ? CMD_VALUE_ALT_SENSOR_ON : CMD_VALUE_ALT_SENSOR_OFF;
    return send_command(CMD_OFFSET_ALT_EXTERNAL_SENSOR, value);
}

esp_err_t set_external_pad_heater(uint8_t mode) {
    uint8_t value;
    switch (mode) {
        case 1: value = CMD_VALUE_EXTERNAL_PAD_ON; break;
        case 2: value = CMD_VALUE_EXTERNAL_PAD_AUTO; break;
        default: value = CMD_VALUE_EXTERNAL_PAD_OFF; break;
    }
    return send_command(CMD_OFFSET_EXTERNAL_PAD_HEATER, value);
}

esp_err_t set_buffer_delta(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_BUFFER_DELTA, temperature);
}

esp_err_t set_buffer(bool state) {
    const uint8_t value = state ? CMD_VALUE_BUFFER_ON : CMD_VALUE_BUFFER_OFF;
    return send_command(CMD_OFFSET_BUFFER, value);
}

esp_err_t set_heating_off_outdoor_temp(int8_t temperature) {
    return temperature_send_command(CMD_OFFSET_HEATING_OFF_TEMP, temperature);
}

esp_err_t set_external_control(bool state) {
    const uint8_t value = state ? CMD_VALUE_EXTERNAL_CONTROL_ON : CMD_VALUE_EXTERNAL_CONTROL_OFF;
    return send_command(CMD_OFFSET_EXTERNAL_CONTROL, value);
}

esp_err_t set_external_heat_cool_control(bool state) {
    const uint8_t value = state ? CMD_VALUE_EXTERNAL_HEAT_COOL_ON : CMD_VALUE_EXTERNAL_HEAT_COOL_OFF;
    return send_command(CMD_OFFSET_EXTERNAL_HEAT_COOL, value);
}

esp_err_t set_external_error(bool state) {
    const uint8_t value = state ? CMD_VALUE_EXTERNAL_ERROR_ON : CMD_VALUE_EXTERNAL_ERROR_OFF;
    return send_command(CMD_OFFSET_EXTERNAL_ERROR, value);
}

esp_err_t set_external_compressor_control(bool state) {
    const uint8_t value = state ? CMD_VALUE_EXTERNAL_COMPRESSOR_ON : CMD_VALUE_EXTERNAL_COMPRESSOR_OFF;
    return send_command(CMD_OFFSET_EXTERNAL_COMPRESSOR, value);
}

/* @brief Set the value of a byte in the optional PCB query
 * @param val Value to set
 * @param base Base value
 * @param bit Bit to set
 * @return ESP_OK on success
 */
esp_err_t set_byte_6(uint8_t val, uint8_t base, uint8_t bit) {
    protocol_cmd_t cmd;
    cmd.len = PROTOCOL_OPT_WRITE_SIZE;
    memcpy(cmd.data, optional_pcb_query, PROTOCOL_OPT_WRITE_SIZE);
    cmd.data[OPT_OFFSET_BYTE_6] = (cmd.data[OPT_OFFSET_BYTE_6] & ~(base << bit)) | (val << bit);
    return protocol_send_command(&cmd);
}

esp_err_t set_byte_9(uint8_t val) {
    protocol_cmd_t cmd;
    cmd.len = PROTOCOL_OPT_WRITE_SIZE;
    memcpy(cmd.data, optional_pcb_query, PROTOCOL_OPT_WRITE_SIZE);
    cmd.data[OPT_OFFSET_BYTE_9] = val;
    return protocol_send_command(&cmd);
}

esp_err_t set_heat_cool_mode(bool state) {
    return set_byte_6(state, 0b1, 7);
}

esp_err_t set_compressor_state(bool state) {
    return set_byte_6(state, 0b1, 6);
}

esp_err_t set_smart_grid_mode(uint8_t mode) {
    if (mode < 4) {
        return set_byte_6(mode, 0b11, 4);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t set_external_thermostat_1_state(uint8_t mode) {
    if (mode < 4) {
        return set_byte_6(mode, 0b11, 2);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t set_external_thermostat_2_state(uint8_t mode) {
    if (mode < 4) {
        return set_byte_6(mode, 0b11, 0);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t set_demand_control(uint8_t mode) {
    protocol_cmd_t cmd;
    cmd.len = PROTOCOL_OPT_WRITE_SIZE;
    memcpy(cmd.data, optional_pcb_query, PROTOCOL_OPT_WRITE_SIZE);
    cmd.data[OPT_OFFSET_DEMAND_CONTROL] = mode;
    return protocol_send_command(&cmd);
}

esp_err_t set_xxx_temp(float temperature, uint8_t byte) {
    uint8_t value = temp2hex(temperature);
    protocol_cmd_t cmd;
    cmd.len = PROTOCOL_OPT_WRITE_SIZE;
    memcpy(cmd.data, optional_pcb_query, PROTOCOL_OPT_WRITE_SIZE);
    cmd.data[byte] = value;
    return protocol_send_command(&cmd);
}

esp_err_t set_pool_temp(float temperature) {
    return set_xxx_temp(temperature, OPT_OFFSET_POOL_TEMP);
}

esp_err_t set_buffer_temp(float temperature) {
    return set_xxx_temp(temperature, OPT_OFFSET_BUFFER_TEMP);
}

esp_err_t set_z1_room_temp(float temperature) {
    return set_xxx_temp(temperature, OPT_OFFSET_Z1_ROOM_TEMP);
}

esp_err_t set_z1_water_temp(float temperature) {
    return set_xxx_temp(temperature, OPT_OFFSET_Z1_WATER_TEMP);
}

esp_err_t set_z2_room_temp(float temperature) {
    return set_xxx_temp(temperature, OPT_OFFSET_Z2_ROOM_TEMP);
}

esp_err_t set_z2_water_temp(float temperature) {
    return set_xxx_temp(temperature, OPT_OFFSET_Z2_WATER_TEMP);
}

esp_err_t set_solar_temp(float temperature) {
    return set_xxx_temp(temperature, OPT_OFFSET_SOLAR_TEMP);
}
