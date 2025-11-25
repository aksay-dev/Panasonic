/**
 * @file decoder.c
 * @brief Heat pump data decoder implementation
 * @version 1.0.0
 * @date 2024
 * 
 * Based on HeishaMon decode.cpp implementation
 */

#include "decoder.h"
#include "protocol.h"
#include "modbus_params.h"
#include "esp_log.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdint.h>
#include <stdio.h>


static const char *TAG = "DECODER";
// Global decoded data structure (DEPRECATED - kept for compatibility during transition)
// hp_decoder_data_t g_decoded_data;

// Byte offsets (direct indices into g_protocol_rx.data) - removed, using OFFS_* below

// Field-oriented byte offsets (direct indices into g_protocol_rx.data)
#define OFFS_HEATPUMP_STATE                 4
#define OFFS_PUMP_FLOW_HL                   4 /* uses bytes 0-1 */
#define OFFS_FORCE_DHW_STATE                4
#define OFFS_QUIET_MODE_SCHEDULE            7
#define OFFS_OPERATING_MODE_STATE           6
#define OFFS_MAIN_INLET_TEMP                143
#define OFFS_MAIN_OUTLET_TEMP               144
#define OFFS_MAIN_TARGET_TEMP               153
#define OFFS_COMPRESSOR_FREQ                166
#define OFFS_DHW_TARGET_TEMP                42
#define OFFS_DHW_TEMP                       141
#define OFFS_MAIN_SCHEDULE_STATE            5
#define OFFS_OUTSIDE_TEMP                   142
#define OFFS_HEAT_POWER_PRODUCTION          194
#define OFFS_HEAT_POWER_CONSUMPTION         193
#define OFFS_POWERFUL_MODE_TIME             7
#define OFFS_QUIET_MODE_LEVEL               7
#define OFFS_HOLIDAY_MODE_STATE             5
#define OFFS_THREE_WAY_VALVE_STATE          111
#define OFFS_OUTSIDE_PIPE_TEMP              158
#define OFFS_DHW_HEAT_DELTA                 99
#define OFFS_HEAT_DELTA                     84
#define OFFS_COOL_DELTA                     94
#define OFFS_ERROR_STATE_STR                44
#define OFFS_DHW_HOLIDAY_SHIFT_TEMP         44
#define OFFS_DEFROSTING_STATE               111
#define OFFS_Z1_HEAT_REQUEST_TEMP           38
#define OFFS_Z1_COOL_REQUEST_TEMP           39
#define OFFS_Z1_HEAT_CURVE_TARGET_HIGH      75
#define OFFS_Z1_HEAT_CURVE_TARGET_LOW       76
#define OFFS_Z1_HEAT_CURVE_OUTSIDE_HIGH     78
#define OFFS_Z1_HEAT_CURVE_OUTSIDE_LOW      77
#define OFFS_ROOM_THERMOSTAT_TEMP           156
#define OFFS_Z2_HEAT_REQUEST_TEMP           40
#define OFFS_Z2_COOL_REQUEST_TEMP           41
#define OFFS_Z1_WATER_TEMP                  145
#define OFFS_Z2_WATER_TEMP                  146
#define OFFS_COOL_POWER_PRODUCTION          196
#define OFFS_COOL_POWER_CONSUMPTION         195
#define OFFS_DHW_POWER_PRODUCTION           198
#define OFFS_DHW_POWER_CONSUMPTION          197
#define OFFS_Z1_WATER_TARGET_TEMP           147
#define OFFS_Z2_WATER_TARGET_TEMP           148
#define OFFS_ROOM_HOLIDAY_SHIFT_TEMP        43
#define OFFS_BUFFER_TEMP                    149
#define OFFS_SOLAR_TEMP                     150
#define OFFS_POOL_TEMP                      151
#define OFFS_MAIN_HEX_OUTLET_TEMP           154
#define OFFS_DISCHARGE_TEMP                 155
#define OFFS_INSIDE_PIPE_TEMP               157
#define OFFS_DEFROST_TEMP                   159
#define OFFS_EVA_OUTLET_TEMP                160
#define OFFS_BYPASS_OUTLET_TEMP             161
#define OFFS_IPM_TEMP                       162
#define OFFS_Z1_TEMP                        139
#define OFFS_Z2_TEMP                        140
#define OFFS_DHW_HEATER_STATE               9
#define OFFS_ROOM_HEATER_STATE              9
#define OFFS_INTERNAL_HEATER_STATE          112
#define OFFS_EXTERNAL_HEATER_STATE          112
#define OFFS_FAN1_MOTOR_SPEED               173
#define OFFS_FAN2_MOTOR_SPEED               174
#define OFFS_HIGH_PRESSURE                  163
#define OFFS_PUMP_SPEED                     171
#define OFFS_LOW_PRESSURE                   164
#define OFFS_COMPRESSOR_CURRENT             165
#define OFFS_FORCE_HEATER_STATE             5
#define OFFS_STERILIZATION_STATE            117
#define OFFS_STERILIZATION_TEMP             100
#define OFFS_STERILIZATION_MAX_TIME         101
#define OFFS_Z1_COOL_CURVE_TARGET_HIGH      86
#define OFFS_Z1_COOL_CURVE_TARGET_LOW       87
#define OFFS_Z1_COOL_CURVE_OUTSIDE_HIGH     89
#define OFFS_Z1_COOL_CURVE_OUTSIDE_LOW      88
#define OFFS_HEATING_MODE                   28
#define OFFS_HEATING_OFF_OUTDOOR_TEMP       83
#define OFFS_HEATER_ON_OUTDOOR_TEMP         85
#define OFFS_HEAT_TO_COOL_TEMP              95
#define OFFS_COOL_TO_HEAT_TEMP              96
#define OFFS_COOLING_MODE                   28
#define OFFS_Z2_HEAT_CURVE_TARGET_HIGH      79
#define OFFS_Z2_HEAT_CURVE_TARGET_LOW       80
#define OFFS_Z2_HEAT_CURVE_OUTSIDE_HIGH     82
#define OFFS_Z2_HEAT_CURVE_OUTSIDE_LOW      81
#define OFFS_Z2_COOL_CURVE_TARGET_HIGH      90
#define OFFS_Z2_COOL_CURVE_TARGET_LOW       91
#define OFFS_Z2_COOL_CURVE_OUTSIDE_HIGH     93
#define OFFS_Z2_COOL_CURVE_OUTSIDE_LOW      92
#define OFFS_PUMP_DUTY                      172
#define OFFS_ZONES_STATE                    6
#define OFFS_MAX_PUMP_DUTY                  45
#define OFFS_HEATER_DELAY_TIME              104
#define OFFS_HEATER_START_DELTA             105
#define OFFS_HEATER_STOP_DELTA              106
#define OFFS_BUFFER_INSTALLED               24
#define OFFS_DHW_INSTALLED                  24
#define OFFS_SOLAR_MODE                     24
#define OFFS_SOLAR_ON_DELTA                 61
#define OFFS_SOLAR_OFF_DELTA                62
#define OFFS_SOLAR_FROST_PROTECTION         63
#define OFFS_SOLAR_HIGH_LIMIT               64
#define OFFS_PUMP_FLOWRATE_MODE             29
#define OFFS_LIQUID_TYPE                    20
#define OFFS_ALT_EXTERNAL_SENSOR            20
#define OFFS_ANTI_FREEZE_MODE               20
#define OFFS_OPTIONAL_PCB                   20
#define OFFS_Z1_SENSOR_SETTINGS             22
#define OFFS_Z2_SENSOR_SETTINGS             22
#define OFFS_BUFFER_TANK_DELTA              59
#define OFFS_EXTERNAL_PAD_HEATER            25
#define OFFS_WATER_PRESSURE                 125
#define OFFS_SECOND_INLET_TEMP              126
#define OFFS_ECONOMIZER_OUTLET_TEMP         127
#define OFFS_SECOND_ROOM_THERMOSTAT_TEMP    128
#define OFFS_EXTERNAL_CONTROL               23
#define OFFS_EXTERNAL_HEAT_COOL_CONTROL     23
#define OFFS_EXTERNAL_ERROR_SIGNAL          23
#define OFFS_EXTERNAL_COMPRESSOR_CONTROL    23
#define OFFS_Z2_PUMP_STATE                  116
#define OFFS_Z1_PUMP_STATE                  116
#define OFFS_TWOWAY_VALVE_STATE             116
#define OFFS_THREEWAY_VALVE_STATE2          116
#define OFFS_Z1_VALVE_PID                   177
#define OFFS_Z2_VALVE_PID                   178
#define OFFS_BIVALENT_CONTROL               26
#define OFFS_BIVALENT_MODE                  26
#define OFFS_BIVALENT_START_TEMP            65
#define OFFS_BIVALENT_ADV_HEAT              26
#define OFFS_BIVALENT_ADV_DHW               26
#define OFFS_BIVALENT_ADV_START_TEMP        66
#define OFFS_BIVALENT_ADV_STOP_TEMP         68
#define OFFS_BIVALENT_ADV_START_DELAY       67
#define OFFS_BIVALENT_ADV_STOP_DELAY        69
#define OFFS_BIVALENT_ADV_DHW_DELAY         70
#define OFFS_HP_MODEL_0                     129

// Extended data offsets (XTOP topics)
#define OFFS_XTOP_HEAT_POWER_CONSUMPTION_EXTRA    14
#define OFFS_XTOP_COOL_POWER_CONSUMPTION_EXTRA    16
#define OFFS_XTOP_DHW_POWER_CONSUMPTION_EXTRA     18
#define OFFS_XTOP_HEAT_POWER_PRODUCTION_EXTRA     20
#define OFFS_XTOP_COOL_POWER_PRODUCTION_EXTRA     22
#define OFFS_XTOP_DHW_POWER_PRODUCTION_EXTRA      24

#define OFFS_OPERATIONS_HOURS                     182
#define OFFS_OPERATIONS_COUNTER                   179
#define OFFS_ROOM_HEATER_OPERATIONS_HOURS         185
#define OFFS_DHW_HEATER_OPERATIONS_HOURS          188
#define OFFS_MAIN_INLET_FRACTIONAL_TEMP           118
#define OFFS_MAIN_OUTLET_FRACTIONAL_TEMP          118
#define OFFS_PUMP_FLOW                            170
#define OFFS_PUMP_FLOW_FRACTIONAL                 169
#define OFFS_ERROR_TYPE                           113
#define OFFS_ERROR_NUMBER                         114

// Optional PCB data offset (from data[4])
#define OFFS_OPT_PCB_DATA                         4

// Direct numeric decode functions
static int16_t getIntMinus128(uint8_t input) {
    return (int16_t)input - 128;
}

static int16_t getIntMinus1(uint8_t input) {
    return (int16_t)input - 1;
}

static int16_t getIntMinus1Div5(uint8_t input) {
    // return (((int16_t)input - 1) * 100 / 5); // Return as int16_t * 100
    return ((int16_t)input - 1) * 20;
}

static int16_t getIntMinus1Times10(uint8_t input) {
    return ((int16_t)input - 1) * 10;
}

static int16_t getIntMinus1Times50(uint8_t input) {
    return ((int16_t)input - 1) * 50;
}

static int16_t getBit7and8(uint8_t input) {
    return (int16_t)(input & 0b11) - 1;
}

static int16_t getBit1and2(uint8_t input) {
    return (int16_t)(input >> 6) - 1;
}

static int16_t getBit3and4and5(uint8_t input) {
    return (int16_t)((input >> 3) & 0b111) - 1;
}

static int16_t getBit3and4(uint8_t input) {
    return (int16_t)((input >> 4) & 0b11) - 1;
}

static int16_t getBit5and6(uint8_t input) {
    return (int16_t)((input >> 2) & 0b11) - 1;
}

static int16_t getRight3bits(uint8_t input) {
    return (int16_t)(input & 0b111) - 1;
}

static int16_t getValvePID(uint8_t input) {
    // return (((int16_t)input - 1) * 100 / 2 ); // Return as int16_t * 100
    return ((int16_t)input - 1) * 50;
}

static int16_t getIntMinus1Div50(uint8_t input) {
    return ((int16_t)input - 1) * 2; // Return as int16_t * 100
}

static int16_t getFirstByte(uint8_t input) {
    return (int16_t)(input >> 4) - 1;
}

static int16_t getSecondByte(uint8_t input) {
    return (int16_t)(input & 0b1111) - 1;
}

static int16_t getBit1(uint8_t input) {
    return (int16_t)(input >> 7);
}

static int16_t getPower(uint8_t input) {
    return ((int16_t)input - 1) * 200;
}

static int16_t getUint16(uint8_t addr) {
    return (int16_t)((g_protocol_rx.data[addr + 1] << 8) | g_protocol_rx.data[addr]) - 1;
}

static int16_t getPumpFlow() {
    // return (uint16_t)g_protocol_rx.data[170] * 100 + ((uint16_t)g_protocol_rx.data[169] - 1) * 100 / 256; // Return as l/min * 100
    return (int16_t)g_protocol_rx.data[OFFS_PUMP_FLOW] * 100 + ((int16_t)g_protocol_rx.data[OFFS_PUMP_FLOW_FRACTIONAL] - 1) * 100 / 256; // Return as l/min * 100
}

static int16_t getOpMode(uint8_t input) {
    switch ((int)(input & 0b111111)) {
        case 18: return 0;
        case 19: return 1;
        case 25: return 2;
        case 33: return 3;
        case 34: return 4;
        case 35: return 5;
        case 41: return 6;
        case 26: return 7;
        case 42: return 8;
        // default: return 255; // -1 as uint8_t
        default: return -1; // -1 as int16_t
    }
}

/**
 * @brief Decode main heat pump data
 * Now writes directly to Modbus input registers instead of intermediate structure
 */
esp_err_t decode_main_data(void) {
    ESP_LOGD(TAG, "Decoding main data");
    
    // Optimized direct decoding - write directly to Modbus registers
    
    // Temperatures (stored as int16_t * 100, e.g. 25.5°C = 2550)
    // Handle special cases with fractional parts
    int16_t main_inlet_temp = getIntMinus128(g_protocol_rx.data[OFFS_MAIN_INLET_TEMP]) * 100;
    int fractional = (int)(g_protocol_rx.data[OFFS_MAIN_INLET_FRACTIONAL_TEMP] & 0b111);
    if (fractional > 1 && fractional < 5) {
        main_inlet_temp += (fractional - 1) * 25;
    }
    mb_input_registers[MB_INPUT_MAIN_INLET_TEMP_CPY] = mb_input_registers[MB_INPUT_MAIN_INLET_TEMP] = main_inlet_temp;
    
    int16_t main_outlet_temp = getIntMinus128(g_protocol_rx.data[OFFS_MAIN_OUTLET_TEMP]) * 100;
    fractional = (int)((g_protocol_rx.data[OFFS_MAIN_OUTLET_FRACTIONAL_TEMP] >> 3) & 0b111);
    if (fractional > 1 && fractional < 5) {
        main_outlet_temp += (fractional - 1) * 25;
    }
    mb_input_registers[MB_INPUT_MAIN_OUTLET_TEMP_CPY] = mb_input_registers[MB_INPUT_MAIN_OUTLET_TEMP] = main_outlet_temp;
    
    mb_input_registers[MB_INPUT_MAIN_TARGET_TEMP_CPY] = mb_input_registers[MB_INPUT_MAIN_TARGET_TEMP] = 
        getIntMinus128(g_protocol_rx.data[OFFS_MAIN_TARGET_TEMP]);
    mb_input_registers[MB_INPUT_DHW_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_DHW_TEMP]);
    mb_input_registers[MB_INPUT_DHW_TARGET_TEMP_CPY] = mb_input_registers[MB_INPUT_DHW_TARGET_TEMP] = 
        getIntMinus128(g_protocol_rx.data[OFFS_DHW_TARGET_TEMP]);
    mb_input_registers[MB_INPUT_OUTSIDE_TEMP_CPY] = mb_input_registers[MB_INPUT_OUTSIDE_TEMP] = 
        getIntMinus128(g_protocol_rx.data[OFFS_OUTSIDE_TEMP]);
    mb_input_registers[MB_INPUT_ROOM_THERMOSTAT_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_ROOM_THERMOSTAT_TEMP]);
    mb_input_registers[MB_INPUT_BUFFER_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_BUFFER_TEMP]);
    mb_input_registers[MB_INPUT_SOLAR_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_SOLAR_TEMP]);
    mb_input_registers[MB_INPUT_POOL_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_POOL_TEMP]);
    
    // Power values
    mb_input_registers[MB_INPUT_HEAT_POWER_PRODUCTION] = getPower(g_protocol_rx.data[OFFS_HEAT_POWER_PRODUCTION]);
    mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION_CPY] = mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION] = 
        getPower(g_protocol_rx.data[OFFS_HEAT_POWER_CONSUMPTION]);
    mb_input_registers[MB_INPUT_COOL_POWER_PRODUCTION] = getPower(g_protocol_rx.data[OFFS_COOL_POWER_PRODUCTION]);
    mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION_CPY] = mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION] = 
        getPower(g_protocol_rx.data[OFFS_COOL_POWER_CONSUMPTION]);
    mb_input_registers[MB_INPUT_DHW_POWER_PRODUCTION] = getPower(g_protocol_rx.data[OFFS_DHW_POWER_PRODUCTION]);
    mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION_CPY] = mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION] = 
        getPower(g_protocol_rx.data[OFFS_DHW_POWER_CONSUMPTION]);
    
    // Operation states
    mb_input_registers[MB_INPUT_STATUS] = getBit7and8(g_protocol_rx.data[OFFS_HEATPUMP_STATE]);
    mb_input_registers[MB_INPUT_HEATPUMP_STATE_CPY] = mb_input_registers[MB_INPUT_HEATPUMP_STATE] = 
        getBit7and8(g_protocol_rx.data[OFFS_HEATPUMP_STATE]);
    mb_input_registers[MB_INPUT_FORCE_DHW_STATE_CPY] = mb_input_registers[MB_INPUT_FORCE_DHW_STATE] = 
        getBit1and2(g_protocol_rx.data[OFFS_FORCE_DHW_STATE]);
    mb_input_registers[MB_INPUT_OPERATING_MODE_STATE_CPY] = mb_input_registers[MB_INPUT_OPERATING_MODE_STATE] = 
        getOpMode(g_protocol_rx.data[OFFS_OPERATING_MODE_STATE]);
    mb_input_registers[MB_INPUT_QUIET_MODE_SCHEDULE] = getBit1and2(g_protocol_rx.data[OFFS_QUIET_MODE_SCHEDULE]);
    mb_input_registers[MB_INPUT_POWERFUL_MODE_TIME] = getRight3bits(g_protocol_rx.data[OFFS_POWERFUL_MODE_TIME]);
    mb_input_registers[MB_INPUT_QUIET_MODE_LEVEL] = getBit3and4and5(g_protocol_rx.data[OFFS_QUIET_MODE_LEVEL]);
    mb_input_registers[MB_INPUT_HOLIDAY_MODE_STATE] = getBit3and4(g_protocol_rx.data[OFFS_HOLIDAY_MODE_STATE]);
    mb_input_registers[MB_INPUT_THREE_WAY_VALVE_STATE_CPY] = mb_input_registers[MB_INPUT_THREE_WAY_VALVE_STATE] = 
        getBit7and8(g_protocol_rx.data[OFFS_THREE_WAY_VALVE_STATE]);
    mb_input_registers[MB_INPUT_DEFROSTING_STATE_CPY] = mb_input_registers[MB_INPUT_DEFROSTING_STATE] = 
        getBit5and6(g_protocol_rx.data[OFFS_DEFROSTING_STATE]);
    mb_input_registers[MB_INPUT_ZONES_STATE] = getBit1and2(g_protocol_rx.data[OFFS_ZONES_STATE]);
    
    // Technical parameters
    mb_input_registers[MB_INPUT_COMPRESSOR_FREQ_CPY] = mb_input_registers[MB_INPUT_COMPRESSOR_FREQ] = 
        getIntMinus1(g_protocol_rx.data[OFFS_COMPRESSOR_FREQ]);
    mb_input_registers[MB_INPUT_PUMP_FLOW_CPY] = mb_input_registers[MB_INPUT_PUMP_FLOW] = 
        getPumpFlow(); // Special case - needs full function
    mb_input_registers[MB_INPUT_OPERATIONS_HOURS_CPY] = mb_input_registers[MB_INPUT_OPERATIONS_HOURS] = 
        getUint16(OFFS_OPERATIONS_HOURS);
    mb_input_registers[MB_INPUT_OPERATIONS_COUNTER_CPY] = mb_input_registers[MB_INPUT_OPERATIONS_COUNTER] = 
        getUint16(OFFS_OPERATIONS_COUNTER);
    
    // Additional temperatures (stored as int8_t, write as int16_t)
    mb_input_registers[MB_INPUT_MAIN_HEX_OUTLET_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_MAIN_HEX_OUTLET_TEMP]);
    mb_input_registers[MB_INPUT_DISCHARGE_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_DISCHARGE_TEMP]);
    mb_input_registers[MB_INPUT_INSIDE_PIPE_TEMP_CPY] = mb_input_registers[MB_INPUT_INSIDE_PIPE_TEMP] = 
        getIntMinus128(g_protocol_rx.data[OFFS_INSIDE_PIPE_TEMP]);
    mb_input_registers[MB_INPUT_DEFROST_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_DEFROST_TEMP]);
    mb_input_registers[MB_INPUT_EVA_OUTLET_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_EVA_OUTLET_TEMP]);
    mb_input_registers[MB_INPUT_BYPASS_OUTLET_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_BYPASS_OUTLET_TEMP]);
    mb_input_registers[MB_INPUT_IPM_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_IPM_TEMP]);
    mb_input_registers[MB_INPUT_OUTSIDE_PIPE_TEMP_CPY] = mb_input_registers[MB_INPUT_OUTSIDE_PIPE_TEMP] = 
        getIntMinus128(g_protocol_rx.data[OFFS_OUTSIDE_PIPE_TEMP]);
    mb_input_registers[MB_INPUT_Z1_ROOM_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_TEMP]);
    mb_input_registers[MB_INPUT_Z2_ROOM_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_TEMP]);
    mb_input_registers[MB_INPUT_Z1_WATER_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_WATER_TEMP]);
    mb_input_registers[MB_INPUT_Z2_WATER_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_WATER_TEMP]);
    mb_input_registers[MB_INPUT_Z1_WATER_TARGET_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_WATER_TARGET_TEMP]);
    mb_input_registers[MB_INPUT_Z2_WATER_TARGET_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_WATER_TARGET_TEMP]);
    mb_input_registers[MB_INPUT_SECOND_INLET_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_SECOND_INLET_TEMP]);
    mb_input_registers[MB_INPUT_ECONOMIZER_OUTLET_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_ECONOMIZER_OUTLET_TEMP]);
    mb_input_registers[MB_INPUT_SECOND_ROOM_THERMO_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_SECOND_ROOM_THERMOSTAT_TEMP]);
    
    // Zone request temperatures (stored as int8_t, write as int16_t)
    mb_input_registers[MB_INPUT_Z1_HEAT_REQUEST_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_HEAT_REQUEST_TEMP]);
    mb_input_registers[MB_INPUT_Z1_COOL_REQUEST_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_COOL_REQUEST_TEMP]);
    mb_input_registers[MB_INPUT_Z2_HEAT_REQUEST_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_HEAT_REQUEST_TEMP]);
    mb_input_registers[MB_INPUT_Z2_COOL_REQUEST_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_COOL_REQUEST_TEMP]);
    
    // Zone curve settings (stored as int8_t, write as int16_t)
    mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_TARGET_HIGH] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_HEAT_CURVE_TARGET_HIGH]);
    mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_TARGET_LOW] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_HEAT_CURVE_TARGET_LOW]);
    mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_HIGH] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_HEAT_CURVE_OUTSIDE_HIGH]);
    mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_LOW] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_HEAT_CURVE_OUTSIDE_LOW]);
    mb_input_registers[MB_INPUT_Z1_COOL_CURVE_TARGET_HIGH] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_COOL_CURVE_TARGET_HIGH]);
    mb_input_registers[MB_INPUT_Z1_COOL_CURVE_TARGET_LOW] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_COOL_CURVE_TARGET_LOW]);
    mb_input_registers[MB_INPUT_Z1_COOL_CURVE_OUTSIDE_HIGH] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_COOL_CURVE_OUTSIDE_HIGH]);
    mb_input_registers[MB_INPUT_Z1_COOL_CURVE_OUTSIDE_LOW] = getIntMinus128(g_protocol_rx.data[OFFS_Z1_COOL_CURVE_OUTSIDE_LOW]);
    mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_TARGET_HIGH] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_HEAT_CURVE_TARGET_HIGH]);
    mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_TARGET_LOW] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_HEAT_CURVE_TARGET_LOW]);
    mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_HIGH] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_HEAT_CURVE_OUTSIDE_HIGH]);
    mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_LOW] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_HEAT_CURVE_OUTSIDE_LOW]);
    mb_input_registers[MB_INPUT_Z2_COOL_CURVE_TARGET_HIGH] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_COOL_CURVE_TARGET_HIGH]);
    mb_input_registers[MB_INPUT_Z2_COOL_CURVE_TARGET_LOW] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_COOL_CURVE_TARGET_LOW]);
    mb_input_registers[MB_INPUT_Z2_COOL_CURVE_OUTSIDE_HIGH] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_COOL_CURVE_OUTSIDE_HIGH]);
    mb_input_registers[MB_INPUT_Z2_COOL_CURVE_OUTSIDE_LOW] = getIntMinus128(g_protocol_rx.data[OFFS_Z2_COOL_CURVE_OUTSIDE_LOW]);
    
    // Additional operation states
    mb_input_registers[MB_INPUT_MAIN_SCHEDULE_STATE] = getBit1and2(g_protocol_rx.data[OFFS_MAIN_SCHEDULE_STATE]);
    
    // Technical parameters
    mb_input_registers[MB_INPUT_FAN1_MOTOR_SPEED] = getIntMinus1Times10(g_protocol_rx.data[OFFS_FAN1_MOTOR_SPEED]);
    mb_input_registers[MB_INPUT_FAN2_MOTOR_SPEED] = getIntMinus1Times10(g_protocol_rx.data[OFFS_FAN2_MOTOR_SPEED]);
    mb_input_registers[MB_INPUT_HIGH_PRESSURE] = getIntMinus1Div5(g_protocol_rx.data[OFFS_HIGH_PRESSURE]);
    mb_input_registers[MB_INPUT_PUMP_SPEED_CPY] = mb_input_registers[MB_INPUT_PUMP_SPEED] = 
        getIntMinus1Times50(g_protocol_rx.data[OFFS_PUMP_SPEED]);
    mb_input_registers[MB_INPUT_LOW_PRESSURE] = getIntMinus1Times50(g_protocol_rx.data[OFFS_LOW_PRESSURE]);
    mb_input_registers[MB_INPUT_COMPRESSOR_CURRENT_CPY] = mb_input_registers[MB_INPUT_COMPRESSOR_CURRENT] = 
        getIntMinus1Div5(g_protocol_rx.data[OFFS_COMPRESSOR_CURRENT]);
    mb_input_registers[MB_INPUT_PUMP_DUTY_CPY] = mb_input_registers[MB_INPUT_PUMP_DUTY] =
        getIntMinus1(g_protocol_rx.data[OFFS_PUMP_DUTY]);
    mb_input_registers[MB_INPUT_MAX_PUMP_DUTY] = getIntMinus1(g_protocol_rx.data[OFFS_MAX_PUMP_DUTY]);
    
    // Heater states
    mb_input_registers[MB_INPUT_DHW_HEATER_STATE] = getBit5and6(g_protocol_rx.data[OFFS_DHW_HEATER_STATE]);
    mb_input_registers[MB_INPUT_ROOM_HEATER_STATE] = getBit7and8(g_protocol_rx.data[OFFS_ROOM_HEATER_STATE]);
    mb_input_registers[MB_INPUT_INTERNAL_HEATER_STATE] = getBit7and8(g_protocol_rx.data[OFFS_INTERNAL_HEATER_STATE]);
    mb_input_registers[MB_INPUT_EXTERNAL_HEATER_STATE] = getBit5and6(g_protocol_rx.data[OFFS_EXTERNAL_HEATER_STATE]);
    mb_input_registers[MB_INPUT_FORCE_HEATER_STATE] = getBit5and6(g_protocol_rx.data[OFFS_FORCE_HEATER_STATE]);
    mb_input_registers[MB_INPUT_STERILIZATION_STATE] = getBit5and6(g_protocol_rx.data[OFFS_STERILIZATION_STATE]);
    mb_input_registers[MB_INPUT_STERILIZATION_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_STERILIZATION_TEMP]);
    mb_input_registers[MB_INPUT_STERILIZATION_MAX_TIME] = getIntMinus1(g_protocol_rx.data[OFFS_STERILIZATION_MAX_TIME]);
    
    // Deltas and shifts (stored as int16_t * 100)
    mb_input_registers[MB_INPUT_DHW_HEAT_DELTA] = getIntMinus128(g_protocol_rx.data[OFFS_DHW_HEAT_DELTA]);
    mb_input_registers[MB_INPUT_HEAT_DELTA] = getIntMinus128(g_protocol_rx.data[OFFS_HEAT_DELTA]);
    mb_input_registers[MB_INPUT_COOL_DELTA] = getIntMinus128(g_protocol_rx.data[OFFS_COOL_DELTA]);
    mb_input_registers[MB_INPUT_DHW_HOLIDAY_SHIFT_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_DHW_HOLIDAY_SHIFT_TEMP]);
    mb_input_registers[MB_INPUT_ROOM_HOLIDAY_SHIFT_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_ROOM_HOLIDAY_SHIFT_TEMP]);
    mb_input_registers[MB_INPUT_BUFFER_TANK_DELTA] = getIntMinus128(g_protocol_rx.data[OFFS_BUFFER_TANK_DELTA]);
    
    // Mode settings
    mb_input_registers[MB_INPUT_HEATING_MODE_CPY] = mb_input_registers[MB_INPUT_HEATING_MODE] =
        getBit7and8(g_protocol_rx.data[OFFS_HEATING_MODE]);
    mb_input_registers[MB_INPUT_HEATING_OFF_OUTDOOR_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_HEATING_OFF_OUTDOOR_TEMP]);
    mb_input_registers[MB_INPUT_HEATER_ON_OUTDOOR_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_HEATER_ON_OUTDOOR_TEMP]);
    mb_input_registers[MB_INPUT_HEAT_TO_COOL_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_HEAT_TO_COOL_TEMP]);
    mb_input_registers[MB_INPUT_COOL_TO_HEAT_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_COOL_TO_HEAT_TEMP]);
    mb_input_registers[MB_INPUT_COOLING_MODE_CPY] = mb_input_registers[MB_INPUT_COOLING_MODE] =
        getBit5and6(g_protocol_rx.data[OFFS_COOLING_MODE]);
    
    // Solar and buffer settings
    mb_input_registers[MB_INPUT_BUFFER_INSTALLED] = getBit5and6(g_protocol_rx.data[OFFS_BUFFER_INSTALLED]);
    mb_input_registers[MB_INPUT_DHW_INSTALLED] = getBit7and8(g_protocol_rx.data[OFFS_DHW_INSTALLED]);
    mb_input_registers[MB_INPUT_SOLAR_MODE] = getBit3and4(g_protocol_rx.data[OFFS_SOLAR_MODE]);
    mb_input_registers[MB_INPUT_SOLAR_ON_DELTA] = getIntMinus128(g_protocol_rx.data[OFFS_SOLAR_ON_DELTA]);
    mb_input_registers[MB_INPUT_SOLAR_OFF_DELTA] = getIntMinus128(g_protocol_rx.data[OFFS_SOLAR_OFF_DELTA]);
    mb_input_registers[MB_INPUT_SOLAR_FROST_PROTECTION] = getIntMinus128(g_protocol_rx.data[OFFS_SOLAR_FROST_PROTECTION]);
    mb_input_registers[MB_INPUT_SOLAR_HIGH_LIMIT] = getIntMinus128(g_protocol_rx.data[OFFS_SOLAR_HIGH_LIMIT]);
    
    // Pump and liquid settings
    mb_input_registers[MB_INPUT_PUMP_FLOWRATE_MODE] = getBit3and4(g_protocol_rx.data[OFFS_PUMP_FLOWRATE_MODE]);
    mb_input_registers[MB_INPUT_LIQUID_TYPE] = getBit1(g_protocol_rx.data[OFFS_LIQUID_TYPE]);
    mb_input_registers[MB_INPUT_ALT_EXTERNAL_SENSOR] = getBit3and4(g_protocol_rx.data[OFFS_ALT_EXTERNAL_SENSOR]);
    mb_input_registers[MB_INPUT_ANTI_FREEZE_MODE] = getBit5and6(g_protocol_rx.data[OFFS_ANTI_FREEZE_MODE]);
    mb_input_registers[MB_INPUT_OPTIONAL_PCB] = getBit7and8(g_protocol_rx.data[OFFS_OPTIONAL_PCB]);
    
    // Zone sensor settings
    mb_input_registers[MB_INPUT_Z1_SENSOR_SETTINGS] = getSecondByte(g_protocol_rx.data[OFFS_Z1_SENSOR_SETTINGS]);
    mb_input_registers[MB_INPUT_Z2_SENSOR_SETTINGS] = getFirstByte(g_protocol_rx.data[OFFS_Z2_SENSOR_SETTINGS]);
    
    // External controls
    mb_input_registers[MB_INPUT_EXTERNAL_PAD_HEATER] = getBit3and4(g_protocol_rx.data[OFFS_EXTERNAL_PAD_HEATER]);
    mb_input_registers[MB_INPUT_WATER_PRESSURE_CPY] = mb_input_registers[MB_INPUT_WATER_PRESSURE] = 
        getIntMinus1Div50(g_protocol_rx.data[OFFS_WATER_PRESSURE]);
    mb_input_registers[MB_INPUT_EXTERNAL_CONTROL_CPY] = mb_input_registers[MB_INPUT_EXTERNAL_CONTROL] = 
        getBit7and8(g_protocol_rx.data[OFFS_EXTERNAL_CONTROL]);
    mb_input_registers[MB_INPUT_EXTERNAL_HEAT_COOL_CONTROL] = getBit5and6(g_protocol_rx.data[OFFS_EXTERNAL_HEAT_COOL_CONTROL]);
    mb_input_registers[MB_INPUT_EXTERNAL_ERROR_SIGNAL_CPY] = mb_input_registers[MB_INPUT_EXTERNAL_ERROR_SIGNAL] = 
        getBit3and4(g_protocol_rx.data[OFFS_EXTERNAL_ERROR_SIGNAL]);
    mb_input_registers[MB_INPUT_EXTERNAL_COMPRESSOR_CONTROL] = getBit1and2(g_protocol_rx.data[OFFS_EXTERNAL_COMPRESSOR_CONTROL]);
    
    // Pump states
    mb_input_registers[MB_INPUT_Z2_PUMP_STATE] = getBit1and2(g_protocol_rx.data[OFFS_Z2_PUMP_STATE]);
    mb_input_registers[MB_INPUT_Z1_PUMP_STATE] = getBit3and4(g_protocol_rx.data[OFFS_Z1_PUMP_STATE]);
    mb_input_registers[MB_INPUT_TWO_WAY_VALVE_STATE_CPY] = mb_input_registers[MB_INPUT_TWO_WAY_VALVE_STATE] = 
        getBit5and6(g_protocol_rx.data[OFFS_TWOWAY_VALVE_STATE]);
    mb_input_registers[MB_INPUT_THREE_WAY_VALVE_STATE2_CPY] = mb_input_registers[MB_INPUT_THREE_WAY_VALVE_STATE2] = 
        getBit7and8(g_protocol_rx.data[OFFS_THREEWAY_VALVE_STATE2]);
    
    // Valve PID settings (stored as int16_t * 100)
    mb_input_registers[MB_INPUT_Z1_VALVE_PID] = getValvePID(g_protocol_rx.data[OFFS_Z1_VALVE_PID]);
    mb_input_registers[MB_INPUT_Z2_VALVE_PID] = getValvePID(g_protocol_rx.data[OFFS_Z2_VALVE_PID]);
    
    // Bivalent settings
    mb_input_registers[MB_INPUT_BIVALENT_CONTROL] = getBit7and8(g_protocol_rx.data[OFFS_BIVALENT_CONTROL]);
    mb_input_registers[MB_INPUT_BIVALENT_MODE] = getBit5and6(g_protocol_rx.data[OFFS_BIVALENT_MODE]);
    mb_input_registers[MB_INPUT_BIVALENT_START_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_BIVALENT_START_TEMP]);
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_HEAT] = getBit3and4(g_protocol_rx.data[OFFS_BIVALENT_ADV_HEAT]);
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_DHW] = getBit1and2(g_protocol_rx.data[OFFS_BIVALENT_ADV_DHW]);
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_START_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_BIVALENT_ADV_START_TEMP]);
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_STOP_TEMP] = getIntMinus128(g_protocol_rx.data[OFFS_BIVALENT_ADV_STOP_TEMP]);
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_START_DELAY] = getIntMinus1(g_protocol_rx.data[OFFS_BIVALENT_ADV_START_DELAY]);
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_STOP_DELAY] = getIntMinus1(g_protocol_rx.data[OFFS_BIVALENT_ADV_STOP_DELAY]);
    mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_DHW_DELAY] = getIntMinus1(g_protocol_rx.data[OFFS_BIVALENT_ADV_DHW_DELAY]);
    
    // Timing settings
    mb_input_registers[MB_INPUT_HEATER_DELAY_TIME] = getIntMinus1(g_protocol_rx.data[OFFS_HEATER_DELAY_TIME]);
    mb_input_registers[MB_INPUT_HEATER_START_DELTA] = getIntMinus128(g_protocol_rx.data[OFFS_HEATER_START_DELTA]);
    mb_input_registers[MB_INPUT_HEATER_STOP_DELTA] = getIntMinus128(g_protocol_rx.data[OFFS_HEATER_STOP_DELTA]);
    
    // Operation hours
    mb_input_registers[MB_INPUT_ROOM_HEATER_OPS_HOURS] = getUint16(OFFS_ROOM_HEATER_OPERATIONS_HOURS);
    mb_input_registers[MB_INPUT_DHW_HEATER_OPS_HOURS] = getUint16(OFFS_DHW_HEATER_OPERATIONS_HOURS);
    
    // Error and model (string topics) — write directly to Modbus registers
    {
        // Error string is at bytes 113-114 (Error_type and Error_number)
        int error_type = (int)(g_protocol_rx.data[OFFS_ERROR_TYPE]);
        int error_number = ((int)(g_protocol_rx.data[OFFS_ERROR_NUMBER])) - 17;
        if (error_type == 177) { // B1=F type error
            mb_input_registers[MB_INPUT_ERROR_TYPE_CPY] = mb_input_registers[MB_INPUT_ERROR_TYPE] = (int)'F';
            mb_input_registers[MB_INPUT_ERROR_NUMBER_CPY] = mb_input_registers[MB_INPUT_ERROR_NUMBER] = error_number;
        } else if (error_type == 161) { // A1=H type error
            mb_input_registers[MB_INPUT_ERROR_TYPE_CPY] = mb_input_registers[MB_INPUT_ERROR_TYPE] = (int)'H';
            mb_input_registers[MB_INPUT_ERROR_NUMBER_CPY] = mb_input_registers[MB_INPUT_ERROR_NUMBER] = error_number;
        } else {
            mb_input_registers[MB_INPUT_ERROR_TYPE_CPY] = mb_input_registers[MB_INPUT_ERROR_TYPE] = 0;
            mb_input_registers[MB_INPUT_ERROR_NUMBER_CPY] = mb_input_registers[MB_INPUT_ERROR_NUMBER] = 0;
        }
    }
    // Model string is at bytes 129-138 (10 bytes)
    {
        int data_offset = OFFS_HP_MODEL_0;
        for(int i = 0; i < 5; i++) {
            mb_input_registers[MB_INPUT_HP_MODEL_0 + i] = (g_protocol_rx.data[data_offset] << 8) | g_protocol_rx.data[data_offset + 1];
            data_offset += 2;
        }
    }
    
    ESP_LOGD(TAG, "Main data decoded successfully");
    return ESP_OK;
}

/**
 * @brief Decode extra heat pump data
 * Now writes directly to Modbus input registers instead of intermediate structure
 */
esp_err_t decode_extra_data(void) {
    ESP_LOGD(TAG, "Decoding extra data");
    
    // Decode directly into Modbus input registers (uint16_t -> int16_t, compatible)
    mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION_EXTRA] = getUint16(OFFS_XTOP_HEAT_POWER_CONSUMPTION_EXTRA);
    mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION_EXTRA] = getUint16(OFFS_XTOP_COOL_POWER_CONSUMPTION_EXTRA);
    mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION_EXTRA] = getUint16(OFFS_XTOP_DHW_POWER_CONSUMPTION_EXTRA);
    mb_input_registers[MB_INPUT_HEAT_POWER_PRODUCTION_EXTRA] = getUint16(OFFS_XTOP_HEAT_POWER_PRODUCTION_EXTRA);
    mb_input_registers[MB_INPUT_COOL_POWER_PRODUCTION_EXTRA] = getUint16(OFFS_XTOP_COOL_POWER_PRODUCTION_EXTRA);
    mb_input_registers[MB_INPUT_DHW_POWER_PRODUCTION_EXTRA] = getUint16(OFFS_XTOP_DHW_POWER_PRODUCTION_EXTRA);
    
    ESP_LOGD(TAG, "Extra data decoded successfully");
    return ESP_OK;
}

/**
 * @brief Decode optional PCB data
 * Now writes directly to Modbus input registers instead of intermediate structure
 */
esp_err_t decode_opt_data(void) {
    ESP_LOGD(TAG, "Decoding optional data");
    
    // Optional PCB data decoding from g_protocol_rx.data[4]
    uint8_t opt_data = g_protocol_rx.data[OFFS_OPT_PCB_DATA];
    
    // Decode directly into Modbus input registers (uint8_t -> int16_t)
    mb_input_registers[MB_INPUT_Z1_WATER_PUMP] = (int16_t)((opt_data >> 7) & 0x01);
    mb_input_registers[MB_INPUT_Z1_MIXING_VALVE] = (int16_t)((opt_data >> 5) & 0x03);
    mb_input_registers[MB_INPUT_Z2_WATER_PUMP] = (int16_t)((opt_data >> 4) & 0x01);
    mb_input_registers[MB_INPUT_Z2_MIXING_VALVE] = (int16_t)((opt_data >> 2) & 0x03);
    mb_input_registers[MB_INPUT_POOL_WATER_PUMP] = (int16_t)((opt_data >> 1) & 0x01);
    mb_input_registers[MB_INPUT_SOLAR_WATER_PUMP] = (int16_t)((opt_data >> 0) & 0x01);
    mb_input_registers[MB_INPUT_ALARM_STATE] = (int16_t)((opt_data >> 3) & 0x01);
    
    ESP_LOGD(TAG, "Optional data decoded successfully (written directly to Modbus registers)");
    return ESP_OK;
}

void log_main_data(void) {
    ESP_LOGI(TAG, "=== DECODED MAIN DATA ===");
    
    // Main temperatures
    ESP_LOGI(TAG, "main_inlet_temp: %d", mb_input_registers[MB_INPUT_MAIN_INLET_TEMP]);
    ESP_LOGI(TAG, "main_outlet_temp: %d", mb_input_registers[MB_INPUT_MAIN_OUTLET_TEMP]);
    ESP_LOGI(TAG, "main_target_temp: %d", mb_input_registers[MB_INPUT_MAIN_TARGET_TEMP]);
    ESP_LOGI(TAG, "dhw_temp: %d", mb_input_registers[MB_INPUT_DHW_TEMP]);
    ESP_LOGI(TAG, "dhw_target_temp: %d", mb_input_registers[MB_INPUT_DHW_TARGET_TEMP]);
    ESP_LOGI(TAG, "outside_temp: %d", mb_input_registers[MB_INPUT_OUTSIDE_TEMP]);
    ESP_LOGI(TAG, "room_thermostat_temp: %d", mb_input_registers[MB_INPUT_ROOM_THERMOSTAT_TEMP]);
    ESP_LOGI(TAG, "buffer_temp: %d", mb_input_registers[MB_INPUT_BUFFER_TEMP]);
    ESP_LOGI(TAG, "solar_temp: %d", mb_input_registers[MB_INPUT_SOLAR_TEMP]);
    ESP_LOGI(TAG, "pool_temp: %d", mb_input_registers[MB_INPUT_POOL_TEMP]);
    
    // Power data
    ESP_LOGI(TAG, "heat_power_production: %u", (uint16_t)mb_input_registers[MB_INPUT_HEAT_POWER_PRODUCTION]);
    ESP_LOGI(TAG, "heat_power_consumption: %u", (uint16_t)mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION]);
    ESP_LOGI(TAG, "cool_power_production: %u", (uint16_t)mb_input_registers[MB_INPUT_COOL_POWER_PRODUCTION]);
    ESP_LOGI(TAG, "cool_power_consumption: %u", (uint16_t)mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION]);
    ESP_LOGI(TAG, "dhw_power_production: %u", (uint16_t)mb_input_registers[MB_INPUT_DHW_POWER_PRODUCTION]);
    ESP_LOGI(TAG, "dhw_power_consumption: %u", (uint16_t)mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION]);
    
    // States
    ESP_LOGI(TAG, "heatpump_state: %u", (uint16_t)mb_input_registers[MB_INPUT_HEATPUMP_STATE]);
    ESP_LOGI(TAG, "force_dhw_state: %u", (uint16_t)mb_input_registers[MB_INPUT_FORCE_DHW_STATE]);
    ESP_LOGI(TAG, "operating_mode_state: %u", (uint16_t)mb_input_registers[MB_INPUT_OPERATING_MODE_STATE]);
    ESP_LOGI(TAG, "quiet_mode_schedule: %u", (uint16_t)mb_input_registers[MB_INPUT_QUIET_MODE_SCHEDULE]);
    ESP_LOGI(TAG, "powerful_mode_time: %u", (uint16_t)mb_input_registers[MB_INPUT_POWERFUL_MODE_TIME]);
    ESP_LOGI(TAG, "quiet_mode_level: %u", (uint16_t)mb_input_registers[MB_INPUT_QUIET_MODE_LEVEL]);
    ESP_LOGI(TAG, "holiday_mode_state: %u", (uint16_t)mb_input_registers[MB_INPUT_HOLIDAY_MODE_STATE]);
    ESP_LOGI(TAG, "three_way_valve_state: %u", (uint16_t)mb_input_registers[MB_INPUT_THREE_WAY_VALVE_STATE]);
    ESP_LOGI(TAG, "defrosting_state: %u", (uint16_t)mb_input_registers[MB_INPUT_DEFROSTING_STATE]);
    ESP_LOGI(TAG, "main_schedule_state: %u", (uint16_t)mb_input_registers[MB_INPUT_MAIN_SCHEDULE_STATE]);
    ESP_LOGI(TAG, "zones_state: %u", (uint16_t)mb_input_registers[MB_INPUT_ZONES_STATE]);
    
    // Technical parameters
    ESP_LOGI(TAG, "compressor_freq: %u", (uint16_t)mb_input_registers[MB_INPUT_COMPRESSOR_FREQ]);
    ESP_LOGI(TAG, "pump_flow: %d", mb_input_registers[MB_INPUT_PUMP_FLOW]);
    ESP_LOGI(TAG, "operations_hours: %u", (uint16_t)mb_input_registers[MB_INPUT_OPERATIONS_HOURS]);
    ESP_LOGI(TAG, "operations_counter: %u", (uint16_t)mb_input_registers[MB_INPUT_OPERATIONS_COUNTER]);
    ESP_LOGI(TAG, "fan1_motor_speed: %u", (uint16_t)mb_input_registers[MB_INPUT_FAN1_MOTOR_SPEED]);
    ESP_LOGI(TAG, "fan2_motor_speed: %u", (uint16_t)mb_input_registers[MB_INPUT_FAN2_MOTOR_SPEED]);
    ESP_LOGI(TAG, "high_pressure: %d", mb_input_registers[MB_INPUT_HIGH_PRESSURE]);
    ESP_LOGI(TAG, "pump_speed: %u", (uint16_t)mb_input_registers[MB_INPUT_PUMP_SPEED]);
    ESP_LOGI(TAG, "low_pressure: %d", mb_input_registers[MB_INPUT_LOW_PRESSURE]);
    ESP_LOGI(TAG, "compressor_current: %d", mb_input_registers[MB_INPUT_COMPRESSOR_CURRENT]);
    ESP_LOGI(TAG, "pump_duty: %u", (uint16_t)mb_input_registers[MB_INPUT_PUMP_DUTY]);
    ESP_LOGI(TAG, "max_pump_duty: %u", (uint16_t)mb_input_registers[MB_INPUT_MAX_PUMP_DUTY]);
    
    // Extra temperatures
    ESP_LOGI(TAG, "main_hex_outlet_temp: %d", mb_input_registers[MB_INPUT_MAIN_HEX_OUTLET_TEMP]);
    ESP_LOGI(TAG, "discharge_temp: %d", mb_input_registers[MB_INPUT_DISCHARGE_TEMP]);
    ESP_LOGI(TAG, "inside_pipe_temp: %d", mb_input_registers[MB_INPUT_INSIDE_PIPE_TEMP]);
    ESP_LOGI(TAG, "defrost_temp: %d", mb_input_registers[MB_INPUT_DEFROST_TEMP]);
    ESP_LOGI(TAG, "eva_outlet_temp: %d", mb_input_registers[MB_INPUT_EVA_OUTLET_TEMP]);
    ESP_LOGI(TAG, "bypass_outlet_temp: %d", mb_input_registers[MB_INPUT_BYPASS_OUTLET_TEMP]);
    ESP_LOGI(TAG, "ipm_temp: %d", mb_input_registers[MB_INPUT_IPM_TEMP]);
    ESP_LOGI(TAG, "outside_pipe_temp: %d", mb_input_registers[MB_INPUT_OUTSIDE_PIPE_TEMP]);
    ESP_LOGI(TAG, "z1_temp: %d", mb_input_registers[MB_INPUT_Z1_ROOM_TEMP]);
    ESP_LOGI(TAG, "z2_temp: %d", mb_input_registers[MB_INPUT_Z2_ROOM_TEMP]);
    
    // Water temperatures
    ESP_LOGI(TAG, "z1_water_temp: %d", mb_input_registers[MB_INPUT_Z1_WATER_TEMP]);
    ESP_LOGI(TAG, "z2_water_temp: %d", mb_input_registers[MB_INPUT_Z2_WATER_TEMP]);
    ESP_LOGI(TAG, "z1_water_target_temp: %d", mb_input_registers[MB_INPUT_Z1_WATER_TARGET_TEMP]);
    ESP_LOGI(TAG, "z2_water_target_temp: %d", mb_input_registers[MB_INPUT_Z2_WATER_TARGET_TEMP]);
    ESP_LOGI(TAG, "second_inlet_temp: %d", mb_input_registers[MB_INPUT_SECOND_INLET_TEMP]);
    ESP_LOGI(TAG, "economizer_outlet_temp: %d", mb_input_registers[MB_INPUT_ECONOMIZER_OUTLET_TEMP]);
    ESP_LOGI(TAG, "second_room_thermostat_temp: %d", mb_input_registers[MB_INPUT_SECOND_ROOM_THERMO_TEMP]);
    
    // Zone request temperatures
    ESP_LOGI(TAG, "z1_heat_request_temp: %d", mb_input_registers[MB_INPUT_Z1_HEAT_REQUEST_TEMP]);
    ESP_LOGI(TAG, "z1_cool_request_temp: %d", mb_input_registers[MB_INPUT_Z1_COOL_REQUEST_TEMP]);
    ESP_LOGI(TAG, "z2_heat_request_temp: %d", mb_input_registers[MB_INPUT_Z2_HEAT_REQUEST_TEMP]);
    ESP_LOGI(TAG, "z2_cool_request_temp: %d", mb_input_registers[MB_INPUT_Z2_COOL_REQUEST_TEMP]);
    
    // Zone 1 curves
    ESP_LOGI(TAG, "z1_heat_curve_target_high_temp: %d", mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_TARGET_HIGH]);
    ESP_LOGI(TAG, "z1_heat_curve_target_low_temp: %d", mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_TARGET_LOW]);
    ESP_LOGI(TAG, "z1_heat_curve_outside_high_temp: %d", mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_HIGH]);
    ESP_LOGI(TAG, "z1_heat_curve_outside_low_temp: %d", mb_input_registers[MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_LOW]);
    ESP_LOGI(TAG, "z1_cool_curve_target_high_temp: %d", mb_input_registers[MB_INPUT_Z1_COOL_CURVE_TARGET_HIGH]);
    ESP_LOGI(TAG, "z1_cool_curve_target_low_temp: %d", mb_input_registers[MB_INPUT_Z1_COOL_CURVE_TARGET_LOW]);
    ESP_LOGI(TAG, "z1_cool_curve_outside_high_temp: %d", mb_input_registers[MB_INPUT_Z1_COOL_CURVE_OUTSIDE_HIGH]);
    ESP_LOGI(TAG, "z1_cool_curve_outside_low_temp: %d", mb_input_registers[MB_INPUT_Z1_COOL_CURVE_OUTSIDE_LOW]);
    
    // Zone 2 curves
    ESP_LOGI(TAG, "z2_heat_curve_target_high_temp: %d", mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_TARGET_HIGH]);
    ESP_LOGI(TAG, "z2_heat_curve_target_low_temp: %d", mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_TARGET_LOW]);
    ESP_LOGI(TAG, "z2_heat_curve_outside_high_temp: %d", mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_HIGH]);
    ESP_LOGI(TAG, "z2_heat_curve_outside_low_temp: %d", mb_input_registers[MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_LOW]);
    ESP_LOGI(TAG, "z2_cool_curve_target_high_temp: %d", mb_input_registers[MB_INPUT_Z2_COOL_CURVE_TARGET_HIGH]);
    ESP_LOGI(TAG, "z2_cool_curve_target_low_temp: %d", mb_input_registers[MB_INPUT_Z2_COOL_CURVE_TARGET_LOW]);
    ESP_LOGI(TAG, "z2_cool_curve_outside_high_temp: %d", mb_input_registers[MB_INPUT_Z2_COOL_CURVE_OUTSIDE_HIGH]);
    ESP_LOGI(TAG, "z2_cool_curve_outside_low_temp: %d", mb_input_registers[MB_INPUT_Z2_COOL_CURVE_OUTSIDE_LOW]);
    
    // Heaters
    ESP_LOGI(TAG, "dhw_heater_state: %u", mb_input_registers[MB_INPUT_DHW_HEATER_STATE]);
    ESP_LOGI(TAG, "room_heater_state: %u", mb_input_registers[MB_INPUT_ROOM_HEATER_STATE]);
    ESP_LOGI(TAG, "internal_heater_state: %u", mb_input_registers[MB_INPUT_INTERNAL_HEATER_STATE]);
    ESP_LOGI(TAG, "external_heater_state: %u", mb_input_registers[MB_INPUT_EXTERNAL_HEATER_STATE]);
    ESP_LOGI(TAG, "force_heater_state: %u", mb_input_registers[MB_INPUT_FORCE_HEATER_STATE]);
    ESP_LOGI(TAG, "sterilization_state: %u", mb_input_registers[MB_INPUT_STERILIZATION_STATE]);
    ESP_LOGI(TAG, "sterilization_temp: %d", mb_input_registers[MB_INPUT_STERILIZATION_TEMP]);
    ESP_LOGI(TAG, "sterilization_max_time: %u", mb_input_registers[MB_INPUT_STERILIZATION_MAX_TIME]);
    
    // Deltas
    ESP_LOGI(TAG, "dhw_heat_delta: %d", mb_input_registers[MB_INPUT_DHW_HEAT_DELTA]);
    ESP_LOGI(TAG, "heat_delta: %d", mb_input_registers[MB_INPUT_HEAT_DELTA]);
    ESP_LOGI(TAG, "cool_delta: %d", mb_input_registers[MB_INPUT_COOL_DELTA]);
    ESP_LOGI(TAG, "dhw_holiday_shift_temp: %d", mb_input_registers[MB_INPUT_DHW_HOLIDAY_SHIFT_TEMP]);
    ESP_LOGI(TAG, "room_holiday_shift_temp: %d", mb_input_registers[MB_INPUT_ROOM_HOLIDAY_SHIFT_TEMP]);
    ESP_LOGI(TAG, "buffer_tank_delta: %d", mb_input_registers[MB_INPUT_BUFFER_TANK_DELTA]);
    
    // Modes
    ESP_LOGI(TAG, "heating_mode: %u", mb_input_registers[MB_INPUT_HEATING_MODE]);
    ESP_LOGI(TAG, "heating_off_outdoor_temp: %d", mb_input_registers[MB_INPUT_HEATING_OFF_OUTDOOR_TEMP]);
    ESP_LOGI(TAG, "heater_on_outdoor_temp: %d", mb_input_registers[MB_INPUT_HEATER_ON_OUTDOOR_TEMP]);
    ESP_LOGI(TAG, "heat_to_cool_temp: %d", mb_input_registers[MB_INPUT_HEAT_TO_COOL_TEMP]);
    ESP_LOGI(TAG, "cool_to_heat_temp: %d", mb_input_registers[MB_INPUT_COOL_TO_HEAT_TEMP]);
    ESP_LOGI(TAG, "cooling_mode: %u", mb_input_registers[MB_INPUT_COOLING_MODE]);
    
    // Solar/Buffer
    ESP_LOGI(TAG, "buffer_installed: %u", mb_input_registers[MB_INPUT_BUFFER_INSTALLED]);
    ESP_LOGI(TAG, "dhw_installed: %u", mb_input_registers[MB_INPUT_DHW_INSTALLED]);
    ESP_LOGI(TAG, "solar_mode: %u", mb_input_registers[MB_INPUT_SOLAR_MODE]);
    ESP_LOGI(TAG, "solar_on_delta: %d", mb_input_registers[MB_INPUT_SOLAR_ON_DELTA]);
    ESP_LOGI(TAG, "solar_off_delta: %d", mb_input_registers[MB_INPUT_SOLAR_OFF_DELTA]);
    ESP_LOGI(TAG, "solar_frost_protection: %d", mb_input_registers[MB_INPUT_SOLAR_FROST_PROTECTION]);
    ESP_LOGI(TAG, "solar_high_limit: %d", mb_input_registers[MB_INPUT_SOLAR_HIGH_LIMIT]);
    
    // Pump/Liquid
    ESP_LOGI(TAG, "pump_flowrate_mode: %u", mb_input_registers[MB_INPUT_PUMP_FLOWRATE_MODE]);
    ESP_LOGI(TAG, "liquid_type: %u", mb_input_registers[MB_INPUT_LIQUID_TYPE]);
    ESP_LOGI(TAG, "alt_external_sensor: %u", mb_input_registers[MB_INPUT_ALT_EXTERNAL_SENSOR]);
    ESP_LOGI(TAG, "anti_freeze_mode: %u", mb_input_registers[MB_INPUT_ANTI_FREEZE_MODE]);
    ESP_LOGI(TAG, "optional_pcb: %u", mb_input_registers[MB_INPUT_OPTIONAL_PCB]);
    
    // Zone sensors
    ESP_LOGI(TAG, "z1_sensor_settings: %u", mb_input_registers[MB_INPUT_Z1_SENSOR_SETTINGS]);
    ESP_LOGI(TAG, "z2_sensor_settings: %u", mb_input_registers[MB_INPUT_Z2_SENSOR_SETTINGS]);
    
    // External
    ESP_LOGI(TAG, "external_pad_heater: %u", mb_input_registers[MB_INPUT_EXTERNAL_PAD_HEATER]);
    ESP_LOGI(TAG, "water_pressure: %d", mb_input_registers[MB_INPUT_WATER_PRESSURE]);
    ESP_LOGI(TAG, "external_control: %u", mb_input_registers[MB_INPUT_EXTERNAL_CONTROL]);
    ESP_LOGI(TAG, "external_heat_cool_control: %u", mb_input_registers[MB_INPUT_EXTERNAL_HEAT_COOL_CONTROL]);
    ESP_LOGI(TAG, "external_error_signal: %u", mb_input_registers[MB_INPUT_EXTERNAL_ERROR_SIGNAL]);
    ESP_LOGI(TAG, "external_compressor_control: %u", mb_input_registers[MB_INPUT_EXTERNAL_COMPRESSOR_CONTROL]);
    
    // Pumps
    ESP_LOGI(TAG, "z2_pump_state: %u", mb_input_registers[MB_INPUT_Z2_PUMP_STATE]);
    ESP_LOGI(TAG, "z1_pump_state: %u", mb_input_registers[MB_INPUT_Z1_PUMP_STATE]);
    ESP_LOGI(TAG, "two_way_valve_state: %u", mb_input_registers[MB_INPUT_TWO_WAY_VALVE_STATE]);
    ESP_LOGI(TAG, "three_way_valve_state2: %u", mb_input_registers[MB_INPUT_THREE_WAY_VALVE_STATE2]);
    
    // PID
    ESP_LOGI(TAG, "z1_valve_pid: %d", mb_input_registers[MB_INPUT_Z1_VALVE_PID]);
    ESP_LOGI(TAG, "z2_valve_pid: %d", mb_input_registers[MB_INPUT_Z2_VALVE_PID]);
    
    // Bivalent
    ESP_LOGI(TAG, "bivalent_control: %u", mb_input_registers[MB_INPUT_BIVALENT_CONTROL]);
    ESP_LOGI(TAG, "bivalent_mode: %u", mb_input_registers[MB_INPUT_BIVALENT_MODE]);
    ESP_LOGI(TAG, "bivalent_start_temp: %d", mb_input_registers[MB_INPUT_BIVALENT_START_TEMP]);
    ESP_LOGI(TAG, "bivalent_advanced_heat: %u", mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_HEAT]);
    ESP_LOGI(TAG, "bivalent_advanced_dhw: %u", mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_DHW]);
    ESP_LOGI(TAG, "bivalent_advanced_start_temp: %d", mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_START_TEMP]);
    ESP_LOGI(TAG, "bivalent_advanced_stop_temp: %d", mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_STOP_TEMP]);
    ESP_LOGI(TAG, "bivalent_advanced_start_delay: %u", mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_START_DELAY]);
    ESP_LOGI(TAG, "bivalent_advanced_stop_delay: %u", mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_STOP_DELAY]);
    ESP_LOGI(TAG, "bivalent_advanced_dhw_delay: %u", mb_input_registers[MB_INPUT_BIVALENT_ADVANCED_DHW_DELAY]);
    
    // Heater timing settings
    ESP_LOGI(TAG, "heater_delay_time: %u", mb_input_registers[MB_INPUT_HEATER_DELAY_TIME]);
    ESP_LOGI(TAG, "heater_start_delta: %d", mb_input_registers[MB_INPUT_HEATER_START_DELTA]);
    ESP_LOGI(TAG, "heater_stop_delta: %d", mb_input_registers[MB_INPUT_HEATER_STOP_DELTA]);
    
    // Hours
    ESP_LOGI(TAG, "room_heater_operations_hours: %u", mb_input_registers[MB_INPUT_ROOM_HEATER_OPS_HOURS]);
    ESP_LOGI(TAG, "dhw_heater_operations_hours: %u", mb_input_registers[MB_INPUT_DHW_HEATER_OPS_HOURS]);
    ESP_LOGI(TAG, "error_type: '%c'", (char)mb_input_registers[MB_INPUT_ERROR_TYPE]);
    ESP_LOGI(TAG, "error_number: %d", mb_input_registers[MB_INPUT_ERROR_NUMBER]);

    {
        char model_str[30];
        for (size_t i = 0; i < 5; i++) {
            uint16_t reg = (uint16_t)mb_input_registers[MB_INPUT_HP_MODEL_0 + i];
            sprintf(model_str + i * 6, "%02X %02X ", reg >> 8, reg & 0xFF);
        }
        model_str[29] = '\0';
        ESP_LOGI(TAG, "heat_pump_model: '%s'", model_str);
    }
    
    ESP_LOGI(TAG, "=== END DECODED MAIN DATA ===");
}

void log_extra_data(void) {
    ESP_LOGI(TAG, "=== DECODED EXTRA DATA ===");
    ESP_LOGI(TAG, "heat_power_consumption_extra: %u", mb_input_registers[MB_INPUT_HEAT_POWER_CONSUMPTION_EXTRA]);
    ESP_LOGI(TAG, "cool_power_consumption_extra: %u", mb_input_registers[MB_INPUT_COOL_POWER_CONSUMPTION_EXTRA]);
    ESP_LOGI(TAG, "dhw_power_consumption_extra: %u", mb_input_registers[MB_INPUT_DHW_POWER_CONSUMPTION_EXTRA]);
    ESP_LOGI(TAG, "heat_power_production_extra: %u", mb_input_registers[MB_INPUT_HEAT_POWER_PRODUCTION_EXTRA]);
    ESP_LOGI(TAG, "cool_power_production_extra: %u", mb_input_registers[MB_INPUT_COOL_POWER_PRODUCTION_EXTRA]);
    ESP_LOGI(TAG, "dhw_power_production_extra: %u", mb_input_registers[MB_INPUT_DHW_POWER_PRODUCTION_EXTRA]);
    ESP_LOGI(TAG, "=== END DECODED EXTRA DATA ===");
}

void log_opt_data(void) {
    ESP_LOGI(TAG, "=== DECODED OPT DATA ===");
    ESP_LOGI(TAG, "z1_water_pump: %u", mb_input_registers[MB_INPUT_Z1_WATER_PUMP]);
    ESP_LOGI(TAG, "z1_mixing_valve: %u", mb_input_registers[MB_INPUT_Z1_MIXING_VALVE]);
    ESP_LOGI(TAG, "z2_water_pump: %u", mb_input_registers[MB_INPUT_Z2_WATER_PUMP]);
    ESP_LOGI(TAG, "z2_mixing_valve: %u", mb_input_registers[MB_INPUT_Z2_MIXING_VALVE]);
    ESP_LOGI(TAG, "pool_water_pump: %u", mb_input_registers[MB_INPUT_POOL_WATER_PUMP]);
    ESP_LOGI(TAG, "solar_water_pump: %u", mb_input_registers[MB_INPUT_SOLAR_WATER_PUMP]);
    ESP_LOGI(TAG, "alarm_state: %u", mb_input_registers[MB_INPUT_ALARM_STATE]);
    ESP_LOGI(TAG, "=== END DECODED OPT DATA ===");
}