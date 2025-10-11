/**
 * @file decoder.c
 * @brief Heat pump data decoder implementation
 * @version 1.0.0
 * @date 2024
 * 
 * Based on HeishaMon decode.cpp implementation
 */

#include "include/decoder.h"
#include "include/protocol.h"
#include "esp_log.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "esp_timer.h"
#include <stdint.h>

static const char *TAG = "DECODER_NEW";
// Global decoded data structure
hp_decoder_data_t g_decoded_data;

// Byte offsets (direct indices into g_protocol_rx_buffer) - removed, using OFFS_* below

// Field-oriented byte offsets (direct indices into g_protocol_rx_buffer)
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

// Extended data offsets (XTOP topics)
#define OFFS_XTOP_HEAT_POWER_CONSUMPTION_EXTRA    14
#define OFFS_XTOP_COOL_POWER_CONSUMPTION_EXTRA    16
#define OFFS_XTOP_DHW_POWER_CONSUMPTION_EXTRA     18
#define OFFS_XTOP_HEAT_POWER_PRODUCTION_EXTRA     20
#define OFFS_XTOP_COOL_POWER_PRODUCTION_EXTRA     22
#define OFFS_XTOP_DHW_POWER_PRODUCTION_EXTRA      24

// Optional PCB data offset (from data[4])
#define OFFS_OPT_PCB_DATA                         4
// Direct numeric decode functions (optimized - no string conversion)
static int16_t getIntMinus128(uint8_t input) {
    return (int16_t)input - 128;
}

static int16_t getIntMinus1(uint8_t input) {
    return (int16_t)input - 1;
}

static int16_t getIntMinus1Div5(uint8_t input) {
    return (((int16_t)input - 1) * 100 / 5); // Return as int16_t * 100
}

static int16_t getIntMinus1Times10(uint8_t input) {
    return (int16_t)((input - 1) * 10);
}

static int16_t getIntMinus1Times50(uint8_t input) {
    return (int16_t)((input - 1) * 50);
}

static uint8_t getBit7and8(uint8_t input) {
    return (input & 0b11) - 1;
}

static uint8_t getBit1and2(uint8_t input) {
    return (input >> 6) - 1;
}

static uint8_t getBit3and4and5(uint8_t input) {
    return ((input >> 3) & 0b111) - 1;
}

static uint8_t getBit3and4(uint8_t input) {
    return ((input >> 4) & 0b11) - 1;
}

static uint8_t getBit5and6(uint8_t input) {
    return ((input >> 2) & 0b11) - 1;
}

static uint8_t getRight3bits(uint8_t input) {
    return (input & 0b111) - 1;
}

static int16_t getValvePID(uint8_t input) {
    return (((int16_t)input - 1) * 100 / 2 ); // Return as int16_t * 100
}

static int16_t getIntMinus1Div50(uint8_t input) {
    return (((int16_t)input - 1) * 2); // Return as int16_t * 100
}

static uint8_t getFirstByte(uint8_t input) {
    return (input >> 4) - 1;
}

static uint8_t getSecondByte(uint8_t input) {
    return (input & 0b1111) - 1;
}

static uint8_t getBit1(uint8_t input) {
    return (input >> 7);
}

static uint16_t getPower(uint8_t input) {
    return (uint16_t)((input - 1) * 200);
}

static uint16_t getUint16(uint8_t addr) {
    return ((g_protocol_rx_buffer[addr + 1] << 8) | g_protocol_rx_buffer[addr]) - 1;
}

// static uint16_t getPumpFlow() {
//     // HeishaMon algorithm: PumpFlow1 + PumpFlow2
//     int PumpFlow1 = (int)g_protocol_rx_buffer[170];
//     float PumpFlow2 = (((float)g_protocol_rx_buffer[169] - 1) / 256);
//     float PumpFlow = PumpFlow1 + PumpFlow2;
//     return (uint16_t)(PumpFlow * 100); // Return as l/min * 100
// }

static uint16_t getPumpFlow() {
    return (uint16_t)g_protocol_rx_buffer[170] * 100 + ((uint16_t)g_protocol_rx_buffer[169] - 1) * 100 / 256; // Return as l/min * 100
}

static uint8_t getOpMode(uint8_t input) {
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
        default: return 255; // -1 as uint8_t
    }
}

/**
 * @brief Initialize the decoder module
 */
esp_err_t decoder_init(void) {
    ESP_LOGI(TAG, "Initializing decoder module");
    
    // Initialize decoded data structure
    memset(&g_decoded_data, 0, sizeof(hp_decoder_data_t));
    return ESP_OK;
}


/**
 * @brief Decode main heat pump data
 */
esp_err_t decode_main_data(void) {
    ESP_LOGD(TAG, "Decoding main data");
    
    // Optimized direct decoding using function tables (like HeishaMon)
    
    // Temperatures (stored as int16_t * 100, e.g. 25.5°C = 2550)
    // Handle special cases with fractional parts
    uint8_t input_byte = g_protocol_rx_buffer[OFFS_MAIN_INLET_TEMP];
    g_decoded_data.main_inlet_temp = getIntMinus128(input_byte) * 100;
    int fractional = (int)(g_protocol_rx_buffer[118] & 0b111);
    if (fractional > 1 && fractional < 5) {
        g_decoded_data.main_inlet_temp += (fractional - 1) * 25;
    }
    
    input_byte = g_protocol_rx_buffer[OFFS_MAIN_OUTLET_TEMP];
    g_decoded_data.main_outlet_temp = getIntMinus128(input_byte) * 100;
    fractional = (int)((g_protocol_rx_buffer[118] >> 3) & 0b111);
    if (fractional > 1 && fractional < 5) {
        g_decoded_data.main_outlet_temp += (fractional - 1) * 25;
    }
    
    g_decoded_data.main_target_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_MAIN_TARGET_TEMP]) * 100;
    g_decoded_data.dhw_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_DHW_TEMP]) * 100;
    g_decoded_data.dhw_target_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_DHW_TARGET_TEMP]) * 100;
    g_decoded_data.outside_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_OUTSIDE_TEMP]) * 100;
    g_decoded_data.room_thermostat_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_ROOM_THERMOSTAT_TEMP]) * 100;
    g_decoded_data.buffer_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_BUFFER_TEMP]) * 100;
    g_decoded_data.solar_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_SOLAR_TEMP]) * 100;
    g_decoded_data.pool_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_POOL_TEMP]) * 100;
    
    // Power values
    g_decoded_data.heat_power_production = getPower(g_protocol_rx_buffer[OFFS_HEAT_POWER_PRODUCTION]);
    g_decoded_data.heat_power_consumption = getPower(g_protocol_rx_buffer[OFFS_HEAT_POWER_CONSUMPTION]);
    g_decoded_data.cool_power_production = getPower(g_protocol_rx_buffer[OFFS_COOL_POWER_PRODUCTION]);
    g_decoded_data.cool_power_consumption = getPower(g_protocol_rx_buffer[OFFS_COOL_POWER_CONSUMPTION]);
    g_decoded_data.dhw_power_production = getPower(g_protocol_rx_buffer[OFFS_DHW_POWER_PRODUCTION]);
    g_decoded_data.dhw_power_consumption = getPower(g_protocol_rx_buffer[OFFS_DHW_POWER_CONSUMPTION]);
    
    // Operation states
    g_decoded_data.heatpump_state = getBit7and8(g_protocol_rx_buffer[OFFS_HEATPUMP_STATE]);
    g_decoded_data.force_dhw_state = getBit1and2(g_protocol_rx_buffer[OFFS_FORCE_DHW_STATE]);
    g_decoded_data.operating_mode_state = getOpMode(g_protocol_rx_buffer[OFFS_OPERATING_MODE_STATE]);
    g_decoded_data.quiet_mode_schedule = getBit1and2(g_protocol_rx_buffer[OFFS_QUIET_MODE_SCHEDULE]);
    g_decoded_data.powerful_mode_time = getRight3bits(g_protocol_rx_buffer[OFFS_POWERFUL_MODE_TIME]);
    g_decoded_data.quiet_mode_level = getBit3and4and5(g_protocol_rx_buffer[OFFS_QUIET_MODE_LEVEL]);
    g_decoded_data.holiday_mode_state = getBit3and4(g_protocol_rx_buffer[OFFS_HOLIDAY_MODE_STATE]);
    g_decoded_data.three_way_valve_state = getBit7and8(g_protocol_rx_buffer[OFFS_THREE_WAY_VALVE_STATE]);
    g_decoded_data.defrosting_state = getBit5and6(g_protocol_rx_buffer[OFFS_DEFROSTING_STATE]);
    g_decoded_data.zones_state = getBit1and2(g_protocol_rx_buffer[OFFS_ZONES_STATE]);
    
    // Technical parameters - using universal decoder
    g_decoded_data.compressor_freq = getIntMinus1(g_protocol_rx_buffer[OFFS_COMPRESSOR_FREQ]);
    g_decoded_data.pump_flow = getPumpFlow(); // Special case - needs full function
    g_decoded_data.operations_hours = (uint16_t)(((g_protocol_rx_buffer[183] << 8) | g_protocol_rx_buffer[182]) - 1);
    g_decoded_data.operations_counter = (uint16_t)(((g_protocol_rx_buffer[180] << 8) | g_protocol_rx_buffer[179]) - 1);
    
    // Additional temperatures (stored as int16_t * 100) — direct decoding
    g_decoded_data.main_hex_outlet_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_MAIN_HEX_OUTLET_TEMP]) * 100;
    g_decoded_data.discharge_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_DISCHARGE_TEMP]) * 100;
    g_decoded_data.inside_pipe_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_INSIDE_PIPE_TEMP]) * 100;
    g_decoded_data.defrost_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_DEFROST_TEMP]) * 100;
    g_decoded_data.eva_outlet_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_EVA_OUTLET_TEMP]) * 100;
    g_decoded_data.bypass_outlet_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_BYPASS_OUTLET_TEMP]) * 100;
    g_decoded_data.ipm_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_IPM_TEMP]) * 100;
    g_decoded_data.outside_pipe_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_OUTSIDE_PIPE_TEMP]) * 100;
    g_decoded_data.z1_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_TEMP]) * 100;
    g_decoded_data.z2_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_TEMP]) * 100;
    g_decoded_data.z1_water_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_WATER_TEMP]) * 100;
    g_decoded_data.z2_water_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_WATER_TEMP]) * 100;
    g_decoded_data.z1_water_target_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_WATER_TARGET_TEMP]) * 100;
    g_decoded_data.z2_water_target_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_WATER_TARGET_TEMP]) * 100;
    g_decoded_data.second_inlet_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_SECOND_INLET_TEMP]) * 100;
    g_decoded_data.economizer_outlet_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_ECONOMIZER_OUTLET_TEMP]) * 100;
    g_decoded_data.second_room_thermostat_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_SECOND_ROOM_THERMOSTAT_TEMP]) * 100;
    
    
    // Zone request temperatures (stored as int16_t * 100) — direct decoding
    g_decoded_data.z1_heat_request_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_HEAT_REQUEST_TEMP]) * 100;
    g_decoded_data.z1_cool_request_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_COOL_REQUEST_TEMP]) * 100;
    g_decoded_data.z2_heat_request_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_HEAT_REQUEST_TEMP]) * 100;
    g_decoded_data.z2_cool_request_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_COOL_REQUEST_TEMP]) * 100;
    
    // Zone curve settings (stored as int16_t * 100) — direct decoding
    g_decoded_data.z1_heat_curve_target_high_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_HEAT_CURVE_TARGET_HIGH]) * 100;
    g_decoded_data.z1_heat_curve_target_low_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_HEAT_CURVE_TARGET_LOW]) * 100;
    g_decoded_data.z1_heat_curve_outside_high_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_HEAT_CURVE_OUTSIDE_HIGH]) * 100;
    g_decoded_data.z1_heat_curve_outside_low_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_HEAT_CURVE_OUTSIDE_LOW]) * 100;
    g_decoded_data.z1_cool_curve_target_high_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_COOL_CURVE_TARGET_HIGH]) * 100;
    g_decoded_data.z1_cool_curve_target_low_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_COOL_CURVE_TARGET_LOW]) * 100;
    g_decoded_data.z1_cool_curve_outside_high_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_COOL_CURVE_OUTSIDE_HIGH]) * 100;
    g_decoded_data.z1_cool_curve_outside_low_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z1_COOL_CURVE_OUTSIDE_LOW]) * 100;
    g_decoded_data.z2_heat_curve_target_high_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_HEAT_CURVE_TARGET_HIGH]) * 100;
    g_decoded_data.z2_heat_curve_target_low_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_HEAT_CURVE_TARGET_LOW]) * 100;
    g_decoded_data.z2_heat_curve_outside_high_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_HEAT_CURVE_OUTSIDE_HIGH]) * 100;
    g_decoded_data.z2_heat_curve_outside_low_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_HEAT_CURVE_OUTSIDE_LOW]) * 100;
    g_decoded_data.z2_cool_curve_target_high_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_COOL_CURVE_TARGET_HIGH]) * 100;
    g_decoded_data.z2_cool_curve_target_low_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_COOL_CURVE_TARGET_LOW]) * 100;
    g_decoded_data.z2_cool_curve_outside_high_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_COOL_CURVE_OUTSIDE_HIGH]) * 100;
    g_decoded_data.z2_cool_curve_outside_low_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_Z2_COOL_CURVE_OUTSIDE_LOW]) * 100;
    
    // Additional operation states — direct decoding
    g_decoded_data.main_schedule_state = getBit1and2(g_protocol_rx_buffer[OFFS_MAIN_SCHEDULE_STATE]);
    
    // Technical parameters — direct decoding
    g_decoded_data.fan1_motor_speed = getIntMinus1Times10(g_protocol_rx_buffer[OFFS_FAN1_MOTOR_SPEED]);
    g_decoded_data.fan2_motor_speed = getIntMinus1Times10(g_protocol_rx_buffer[OFFS_FAN2_MOTOR_SPEED]);
    g_decoded_data.high_pressure = getIntMinus1Div5(g_protocol_rx_buffer[OFFS_HIGH_PRESSURE]);
    g_decoded_data.pump_speed = getIntMinus1Times50(g_protocol_rx_buffer[OFFS_PUMP_SPEED]);
    g_decoded_data.low_pressure = getIntMinus1Times50(g_protocol_rx_buffer[OFFS_LOW_PRESSURE]);
    g_decoded_data.compressor_current = getIntMinus1Div5(g_protocol_rx_buffer[OFFS_COMPRESSOR_CURRENT]);
    g_decoded_data.pump_duty = getIntMinus1(g_protocol_rx_buffer[OFFS_PUMP_DUTY]);
    g_decoded_data.max_pump_duty = getIntMinus1(g_protocol_rx_buffer[OFFS_MAX_PUMP_DUTY]);
    
    // Heater states — direct decoding
    g_decoded_data.dhw_heater_state = getBit5and6(g_protocol_rx_buffer[OFFS_DHW_HEATER_STATE]);
    g_decoded_data.room_heater_state = getBit7and8(g_protocol_rx_buffer[OFFS_ROOM_HEATER_STATE]);
    g_decoded_data.internal_heater_state = getBit7and8(g_protocol_rx_buffer[OFFS_INTERNAL_HEATER_STATE]);
    g_decoded_data.external_heater_state = getBit5and6(g_protocol_rx_buffer[OFFS_EXTERNAL_HEATER_STATE]);
    g_decoded_data.force_heater_state = getBit5and6(g_protocol_rx_buffer[OFFS_FORCE_HEATER_STATE]);
    g_decoded_data.sterilization_state = getBit5and6(g_protocol_rx_buffer[OFFS_STERILIZATION_STATE]);
    g_decoded_data.sterilization_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_STERILIZATION_TEMP]) * 100;
    g_decoded_data.sterilization_max_time = getIntMinus1(g_protocol_rx_buffer[OFFS_STERILIZATION_MAX_TIME]);
    
    // Deltas and shifts (stored as int16_t * 100) — direct decoding
    g_decoded_data.dhw_heat_delta = getIntMinus128(g_protocol_rx_buffer[OFFS_DHW_HEAT_DELTA]) * 100;
    g_decoded_data.heat_delta = getIntMinus128(g_protocol_rx_buffer[OFFS_HEAT_DELTA]) * 100;
    g_decoded_data.cool_delta = getIntMinus128(g_protocol_rx_buffer[OFFS_COOL_DELTA]) * 100;
    g_decoded_data.dhw_holiday_shift_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_DHW_HOLIDAY_SHIFT_TEMP]) * 100;
    g_decoded_data.room_holiday_shift_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_ROOM_HOLIDAY_SHIFT_TEMP]) * 100;
    g_decoded_data.buffer_tank_delta = getIntMinus128(g_protocol_rx_buffer[OFFS_BUFFER_TANK_DELTA]) * 100;
    
    // Mode settings — direct decoding
    g_decoded_data.heating_mode = getBit7and8(g_protocol_rx_buffer[OFFS_HEATING_MODE]);
    g_decoded_data.heating_off_outdoor_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_HEATING_OFF_OUTDOOR_TEMP]) * 100;
    g_decoded_data.heater_on_outdoor_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_HEATER_ON_OUTDOOR_TEMP]) * 100;
    g_decoded_data.heat_to_cool_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_HEAT_TO_COOL_TEMP]) * 100;
    g_decoded_data.cool_to_heat_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_COOL_TO_HEAT_TEMP]) * 100;
    g_decoded_data.cooling_mode = getBit5and6(g_protocol_rx_buffer[OFFS_COOLING_MODE]);
    
    // Solar and buffer settings — direct decoding
    g_decoded_data.buffer_installed = getBit5and6(g_protocol_rx_buffer[OFFS_BUFFER_INSTALLED]);
    g_decoded_data.dhw_installed = getBit7and8(g_protocol_rx_buffer[OFFS_DHW_INSTALLED]);
    g_decoded_data.solar_mode = getBit3and4(g_protocol_rx_buffer[OFFS_SOLAR_MODE]);
    g_decoded_data.solar_on_delta = getIntMinus128(g_protocol_rx_buffer[OFFS_SOLAR_ON_DELTA]) * 100;
    g_decoded_data.solar_off_delta = getIntMinus128(g_protocol_rx_buffer[OFFS_SOLAR_OFF_DELTA]) * 100;
    g_decoded_data.solar_frost_protection = getIntMinus128(g_protocol_rx_buffer[OFFS_SOLAR_FROST_PROTECTION]) * 100;
    g_decoded_data.solar_high_limit = getIntMinus128(g_protocol_rx_buffer[OFFS_SOLAR_HIGH_LIMIT]) * 100;
    
    // Pump and liquid settings — direct decoding
    g_decoded_data.pump_flowrate_mode = getBit3and4(g_protocol_rx_buffer[OFFS_PUMP_FLOWRATE_MODE]);
    g_decoded_data.liquid_type = getBit1(g_protocol_rx_buffer[OFFS_LIQUID_TYPE]);
    g_decoded_data.alt_external_sensor = getBit3and4(g_protocol_rx_buffer[OFFS_ALT_EXTERNAL_SENSOR]);
    g_decoded_data.anti_freeze_mode = getBit5and6(g_protocol_rx_buffer[OFFS_ANTI_FREEZE_MODE]);
    g_decoded_data.optional_pcb = getBit7and8(g_protocol_rx_buffer[OFFS_OPTIONAL_PCB]);
    
    // Zone sensor settings — direct decoding
    g_decoded_data.z1_sensor_settings = getSecondByte(g_protocol_rx_buffer[OFFS_Z1_SENSOR_SETTINGS]);
    g_decoded_data.z2_sensor_settings = getFirstByte(g_protocol_rx_buffer[OFFS_Z2_SENSOR_SETTINGS]);
    
    // External controls — direct decoding
    g_decoded_data.external_pad_heater = getBit3and4(g_protocol_rx_buffer[OFFS_EXTERNAL_PAD_HEATER]);
    g_decoded_data.water_pressure = getIntMinus1Div50(g_protocol_rx_buffer[OFFS_WATER_PRESSURE]);
    g_decoded_data.external_control = getBit7and8(g_protocol_rx_buffer[OFFS_EXTERNAL_CONTROL]);
    g_decoded_data.external_heat_cool_control = getBit5and6(g_protocol_rx_buffer[OFFS_EXTERNAL_HEAT_COOL_CONTROL]);
    g_decoded_data.external_error_signal = getBit3and4(g_protocol_rx_buffer[OFFS_EXTERNAL_ERROR_SIGNAL]);
    g_decoded_data.external_compressor_control = getBit1and2(g_protocol_rx_buffer[OFFS_EXTERNAL_COMPRESSOR_CONTROL]);
    
    // Pump states — direct decoding
    g_decoded_data.z2_pump_state = getBit1and2(g_protocol_rx_buffer[OFFS_Z2_PUMP_STATE]);
    g_decoded_data.z1_pump_state = getBit3and4(g_protocol_rx_buffer[OFFS_Z1_PUMP_STATE]);
    g_decoded_data.two_way_valve_state = getBit5and6(g_protocol_rx_buffer[OFFS_TWOWAY_VALVE_STATE]);
    g_decoded_data.three_way_valve_state2 = getBit7and8(g_protocol_rx_buffer[OFFS_THREEWAY_VALVE_STATE2]);
    
    // Valve PID settings (stored as int16_t * 100) — direct decoding
    g_decoded_data.z1_valve_pid = getValvePID(g_protocol_rx_buffer[OFFS_Z1_VALVE_PID]);
    g_decoded_data.z2_valve_pid = getValvePID(g_protocol_rx_buffer[OFFS_Z2_VALVE_PID]);
    
    // Bivalent settings — direct decoding
    g_decoded_data.bivalent_control = getBit7and8(g_protocol_rx_buffer[OFFS_BIVALENT_CONTROL]);
    g_decoded_data.bivalent_mode = getBit5and6(g_protocol_rx_buffer[OFFS_BIVALENT_MODE]);
    g_decoded_data.bivalent_start_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_BIVALENT_START_TEMP]) * 100;
    g_decoded_data.bivalent_advanced_heat = getBit3and4(g_protocol_rx_buffer[OFFS_BIVALENT_ADV_HEAT]);
    g_decoded_data.bivalent_advanced_dhw = getBit1and2(g_protocol_rx_buffer[OFFS_BIVALENT_ADV_DHW]);
    g_decoded_data.bivalent_advanced_start_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_BIVALENT_ADV_START_TEMP]) * 100;
    g_decoded_data.bivalent_advanced_stop_temp = getIntMinus128(g_protocol_rx_buffer[OFFS_BIVALENT_ADV_STOP_TEMP]) * 100;
    g_decoded_data.bivalent_advanced_start_delay = getIntMinus1(g_protocol_rx_buffer[OFFS_BIVALENT_ADV_START_DELAY]);
    g_decoded_data.bivalent_advanced_stop_delay = getIntMinus1(g_protocol_rx_buffer[OFFS_BIVALENT_ADV_STOP_DELAY]);
    g_decoded_data.bivalent_advanced_dhw_delay = getIntMinus1(g_protocol_rx_buffer[OFFS_BIVALENT_ADV_DHW_DELAY]);
    
    // Timing settings
    g_decoded_data.heater_delay_time = getIntMinus1(g_protocol_rx_buffer[OFFS_HEATER_DELAY_TIME]);
    g_decoded_data.heater_start_delta = getIntMinus128(g_protocol_rx_buffer[OFFS_HEATER_START_DELTA]) * 100;
    g_decoded_data.heater_stop_delta = getIntMinus128(g_protocol_rx_buffer[OFFS_HEATER_STOP_DELTA]) * 100;
    
    // Operation hours
    g_decoded_data.room_heater_operations_hours = ((g_protocol_rx_buffer[186] << 8) | g_protocol_rx_buffer[185]) - 1;
    g_decoded_data.dhw_heater_operations_hours = ((g_protocol_rx_buffer[189] << 8) | g_protocol_rx_buffer[188]) - 1;
    
    // Error and model (string topics) — copy as C-strings from buffer starting at topic index
    {
        // Error string is at bytes 113-114 (Error_type and Error_number)
        int Error_type = (int)(g_protocol_rx_buffer[113]);
        int Error_number = ((int)(g_protocol_rx_buffer[114])) - 17;
        if (Error_type == 177) { // B1=F type error
            snprintf(g_decoded_data.error_state, sizeof(g_decoded_data.error_state), "F%02X", Error_number);
        } else if (Error_type == 161) { // A1=H type error
            snprintf(g_decoded_data.error_state, sizeof(g_decoded_data.error_state), "H%02X", Error_number);
        } else {
            strcpy(g_decoded_data.error_state, "No error");
        }
    }
    {
        // Model string is at bytes 129-138 (10 bytes)
        uint8_t model[10] = { g_protocol_rx_buffer[129], g_protocol_rx_buffer[130], g_protocol_rx_buffer[131], 
                             g_protocol_rx_buffer[132], g_protocol_rx_buffer[133], g_protocol_rx_buffer[134], 
                             g_protocol_rx_buffer[135], g_protocol_rx_buffer[136], g_protocol_rx_buffer[137], 
                             g_protocol_rx_buffer[138] };
        char modelResult[31];
        for (size_t i = 0; i < 10; ++i) {
            sprintf(&modelResult[i*3], "%02X ", model[i]);
        }
        modelResult[29] = '\0';
        strncpy(g_decoded_data.heat_pump_model, modelResult, sizeof(g_decoded_data.heat_pump_model) - 1);
        g_decoded_data.heat_pump_model[sizeof(g_decoded_data.heat_pump_model) - 1] = '\0';
    }
    
    g_decoded_data.data_valid = true;
    g_decoded_data.last_update_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    ESP_LOGD(TAG, "Main data decoded successfully");
    return ESP_OK;
}

/**
 * @brief Decode extra heat pump data
 */
esp_err_t decode_extra_data(void) {
    ESP_LOGD(TAG, "Decoding extra data");
    
    // Extended data decoding using meaningful offsets
    g_decoded_data.heat_power_consumption_extra = getUint16(OFFS_XTOP_HEAT_POWER_CONSUMPTION_EXTRA);
    g_decoded_data.cool_power_consumption_extra = getUint16(OFFS_XTOP_COOL_POWER_CONSUMPTION_EXTRA);
    g_decoded_data.dhw_power_consumption_extra = getUint16(OFFS_XTOP_DHW_POWER_CONSUMPTION_EXTRA);
    g_decoded_data.heat_power_production_extra = getUint16(OFFS_XTOP_HEAT_POWER_PRODUCTION_EXTRA);
    g_decoded_data.cool_power_production_extra = getUint16(OFFS_XTOP_COOL_POWER_PRODUCTION_EXTRA);
    g_decoded_data.dhw_power_production_extra = getUint16(OFFS_XTOP_DHW_POWER_PRODUCTION_EXTRA);
    
    ESP_LOGD(TAG, "Extra data decoded successfully");
    return ESP_OK;
}

/**
 * @brief Decode optional PCB data
 */
esp_err_t decode_opt_data(void) {
    ESP_LOGD(TAG, "Decoding optional data");
    
    // Optional PCB data decoding from g_protocol_rx_buffer[4]
    uint8_t opt_data = g_protocol_rx_buffer[OFFS_OPT_PCB_DATA];
    
    g_decoded_data.z1_water_pump = (opt_data >> 7) & 0x01;
    g_decoded_data.z1_mixing_valve = (opt_data >> 5) & 0x03;
    g_decoded_data.z2_water_pump = (opt_data >> 4) & 0x01;
    g_decoded_data.z2_mixing_valve = (opt_data >> 2) & 0x03;
    g_decoded_data.pool_water_pump = (opt_data >> 1) & 0x01;
    g_decoded_data.solar_water_pump = (opt_data >> 0) & 0x01;
    g_decoded_data.alarm_state = (opt_data >> 3) & 0x01;
    
    ESP_LOGD(TAG, "Optional data decoded successfully");
    return ESP_OK;
}

void log_all(void) {
    if (!g_decoded_data.data_valid) {
        ESP_LOGW(TAG, "Data not valid, skipping log");
        return;
    }
    
    ESP_LOGI(TAG, "=== DECODED DATA ===");
    
    // Main temperatures
    ESP_LOGI(TAG, "main_inlet_temp: %d", g_decoded_data.main_inlet_temp);
    ESP_LOGI(TAG, "main_outlet_temp: %d", g_decoded_data.main_outlet_temp);
    ESP_LOGI(TAG, "main_target_temp: %d", g_decoded_data.main_target_temp);
    ESP_LOGI(TAG, "dhw_temp: %d", g_decoded_data.dhw_temp);
    ESP_LOGI(TAG, "dhw_target_temp: %d", g_decoded_data.dhw_target_temp);
    ESP_LOGI(TAG, "outside_temp: %d", g_decoded_data.outside_temp);
    ESP_LOGI(TAG, "room_thermostat_temp: %d", g_decoded_data.room_thermostat_temp);
    ESP_LOGI(TAG, "buffer_temp: %d", g_decoded_data.buffer_temp);
    ESP_LOGI(TAG, "solar_temp: %d", g_decoded_data.solar_temp);
    ESP_LOGI(TAG, "pool_temp: %d", g_decoded_data.pool_temp);
    
    // Power data
    ESP_LOGI(TAG, "heat_power_production: %u", g_decoded_data.heat_power_production);
    ESP_LOGI(TAG, "heat_power_consumption: %u", g_decoded_data.heat_power_consumption);
    ESP_LOGI(TAG, "cool_power_production: %u", g_decoded_data.cool_power_production);
    ESP_LOGI(TAG, "cool_power_consumption: %u", g_decoded_data.cool_power_consumption);
    ESP_LOGI(TAG, "dhw_power_production: %u", g_decoded_data.dhw_power_production);
    ESP_LOGI(TAG, "dhw_power_consumption: %u", g_decoded_data.dhw_power_consumption);
    
    // States
    ESP_LOGI(TAG, "heatpump_state: %u", g_decoded_data.heatpump_state);
    ESP_LOGI(TAG, "force_dhw_state: %u", g_decoded_data.force_dhw_state);
    ESP_LOGI(TAG, "operating_mode_state: %u", g_decoded_data.operating_mode_state);
    ESP_LOGI(TAG, "quiet_mode_schedule: %u", g_decoded_data.quiet_mode_schedule);
    ESP_LOGI(TAG, "powerful_mode_time: %u", g_decoded_data.powerful_mode_time);
    ESP_LOGI(TAG, "quiet_mode_level: %u", g_decoded_data.quiet_mode_level);
    ESP_LOGI(TAG, "holiday_mode_state: %u", g_decoded_data.holiday_mode_state);
    ESP_LOGI(TAG, "three_way_valve_state: %u", g_decoded_data.three_way_valve_state);
    ESP_LOGI(TAG, "defrosting_state: %u", g_decoded_data.defrosting_state);
    ESP_LOGI(TAG, "main_schedule_state: %u", g_decoded_data.main_schedule_state);
    ESP_LOGI(TAG, "zones_state: %u", g_decoded_data.zones_state);
    
    // Technical parameters
    ESP_LOGI(TAG, "compressor_freq: %u", g_decoded_data.compressor_freq);
    ESP_LOGI(TAG, "pump_flow: %d", g_decoded_data.pump_flow);
    ESP_LOGI(TAG, "operations_hours: %u", g_decoded_data.operations_hours);
    ESP_LOGI(TAG, "operations_counter: %u", g_decoded_data.operations_counter);
    ESP_LOGI(TAG, "fan1_motor_speed: %u", g_decoded_data.fan1_motor_speed);
    ESP_LOGI(TAG, "fan2_motor_speed: %u", g_decoded_data.fan2_motor_speed);
    ESP_LOGI(TAG, "high_pressure: %d", g_decoded_data.high_pressure);
    ESP_LOGI(TAG, "pump_speed: %u", g_decoded_data.pump_speed);
    ESP_LOGI(TAG, "low_pressure: %d", g_decoded_data.low_pressure);
    ESP_LOGI(TAG, "compressor_current: %d", g_decoded_data.compressor_current);
    ESP_LOGI(TAG, "pump_duty: %u", g_decoded_data.pump_duty);
    ESP_LOGI(TAG, "max_pump_duty: %u", g_decoded_data.max_pump_duty);
    
    // Extra temperatures
    ESP_LOGI(TAG, "main_hex_outlet_temp: %d", g_decoded_data.main_hex_outlet_temp);
    ESP_LOGI(TAG, "discharge_temp: %d", g_decoded_data.discharge_temp);
    ESP_LOGI(TAG, "inside_pipe_temp: %d", g_decoded_data.inside_pipe_temp);
    ESP_LOGI(TAG, "defrost_temp: %d", g_decoded_data.defrost_temp);
    ESP_LOGI(TAG, "eva_outlet_temp: %d", g_decoded_data.eva_outlet_temp);
    ESP_LOGI(TAG, "bypass_outlet_temp: %d", g_decoded_data.bypass_outlet_temp);
    ESP_LOGI(TAG, "ipm_temp: %d", g_decoded_data.ipm_temp);
    ESP_LOGI(TAG, "outside_pipe_temp: %d", g_decoded_data.outside_pipe_temp);
    ESP_LOGI(TAG, "z1_temp: %d", g_decoded_data.z1_temp);
    ESP_LOGI(TAG, "z2_temp: %d", g_decoded_data.z2_temp);
    
    // Water temperatures
    ESP_LOGI(TAG, "z1_water_temp: %d", g_decoded_data.z1_water_temp);
    ESP_LOGI(TAG, "z2_water_temp: %d", g_decoded_data.z2_water_temp);
    ESP_LOGI(TAG, "z1_water_target_temp: %d", g_decoded_data.z1_water_target_temp);
    ESP_LOGI(TAG, "z2_water_target_temp: %d", g_decoded_data.z2_water_target_temp);
    ESP_LOGI(TAG, "second_inlet_temp: %d", g_decoded_data.second_inlet_temp);
    ESP_LOGI(TAG, "economizer_outlet_temp: %d", g_decoded_data.economizer_outlet_temp);
    ESP_LOGI(TAG, "second_room_thermostat_temp: %d", g_decoded_data.second_room_thermostat_temp);
    
    // Zone 1 curves
    ESP_LOGI(TAG, "z1_heat_curve_target_high_temp: %d", g_decoded_data.z1_heat_curve_target_high_temp);
    ESP_LOGI(TAG, "z1_heat_curve_target_low_temp: %d", g_decoded_data.z1_heat_curve_target_low_temp);
    ESP_LOGI(TAG, "z1_heat_curve_outside_high_temp: %d", g_decoded_data.z1_heat_curve_outside_high_temp);
    ESP_LOGI(TAG, "z1_heat_curve_outside_low_temp: %d", g_decoded_data.z1_heat_curve_outside_low_temp);
    ESP_LOGI(TAG, "z1_cool_curve_target_high_temp: %d", g_decoded_data.z1_cool_curve_target_high_temp);
    ESP_LOGI(TAG, "z1_cool_curve_target_low_temp: %d", g_decoded_data.z1_cool_curve_target_low_temp);
    ESP_LOGI(TAG, "z1_cool_curve_outside_high_temp: %d", g_decoded_data.z1_cool_curve_outside_high_temp);
    ESP_LOGI(TAG, "z1_cool_curve_outside_low_temp: %d", g_decoded_data.z1_cool_curve_outside_low_temp);
    
    // Zone 2 curves
    ESP_LOGI(TAG, "z2_heat_curve_target_high_temp: %d", g_decoded_data.z2_heat_curve_target_high_temp);
    ESP_LOGI(TAG, "z2_heat_curve_target_low_temp: %d", g_decoded_data.z2_heat_curve_target_low_temp);
    ESP_LOGI(TAG, "z2_heat_curve_outside_high_temp: %d", g_decoded_data.z2_heat_curve_outside_high_temp);
    ESP_LOGI(TAG, "z2_heat_curve_outside_low_temp: %d", g_decoded_data.z2_heat_curve_outside_low_temp);
    ESP_LOGI(TAG, "z2_cool_curve_target_high_temp: %d", g_decoded_data.z2_cool_curve_target_high_temp);
    ESP_LOGI(TAG, "z2_cool_curve_target_low_temp: %d", g_decoded_data.z2_cool_curve_target_low_temp);
    ESP_LOGI(TAG, "z2_cool_curve_outside_high_temp: %d", g_decoded_data.z2_cool_curve_outside_high_temp);
    ESP_LOGI(TAG, "z2_cool_curve_outside_low_temp: %d", g_decoded_data.z2_cool_curve_outside_low_temp);
    
    // Heaters
    ESP_LOGI(TAG, "dhw_heater_state: %u", g_decoded_data.dhw_heater_state);
    ESP_LOGI(TAG, "room_heater_state: %u", g_decoded_data.room_heater_state);
    ESP_LOGI(TAG, "internal_heater_state: %u", g_decoded_data.internal_heater_state);
    ESP_LOGI(TAG, "external_heater_state: %u", g_decoded_data.external_heater_state);
    ESP_LOGI(TAG, "force_heater_state: %u", g_decoded_data.force_heater_state);
    ESP_LOGI(TAG, "sterilization_state: %u", g_decoded_data.sterilization_state);
    ESP_LOGI(TAG, "sterilization_temp: %d", g_decoded_data.sterilization_temp);
    ESP_LOGI(TAG, "sterilization_max_time: %u", g_decoded_data.sterilization_max_time);
    
    // Deltas
    ESP_LOGI(TAG, "dhw_heat_delta: %d", g_decoded_data.dhw_heat_delta);
    ESP_LOGI(TAG, "heat_delta: %d", g_decoded_data.heat_delta);
    ESP_LOGI(TAG, "cool_delta: %d", g_decoded_data.cool_delta);
    ESP_LOGI(TAG, "dhw_holiday_shift_temp: %d", g_decoded_data.dhw_holiday_shift_temp);
    ESP_LOGI(TAG, "room_holiday_shift_temp: %d", g_decoded_data.room_holiday_shift_temp);
    ESP_LOGI(TAG, "buffer_tank_delta: %d", g_decoded_data.buffer_tank_delta);
    
    // Modes
    ESP_LOGI(TAG, "heating_mode: %u", g_decoded_data.heating_mode);
    ESP_LOGI(TAG, "heating_off_outdoor_temp: %d", g_decoded_data.heating_off_outdoor_temp);
    ESP_LOGI(TAG, "heater_on_outdoor_temp: %d", g_decoded_data.heater_on_outdoor_temp);
    ESP_LOGI(TAG, "heat_to_cool_temp: %d", g_decoded_data.heat_to_cool_temp);
    ESP_LOGI(TAG, "cool_to_heat_temp: %d", g_decoded_data.cool_to_heat_temp);
    ESP_LOGI(TAG, "cooling_mode: %u", g_decoded_data.cooling_mode);
    
    // Solar/Buffer
    ESP_LOGI(TAG, "buffer_installed: %u", g_decoded_data.buffer_installed);
    ESP_LOGI(TAG, "dhw_installed: %u", g_decoded_data.dhw_installed);
    ESP_LOGI(TAG, "solar_mode: %u", g_decoded_data.solar_mode);
    ESP_LOGI(TAG, "solar_on_delta: %d", g_decoded_data.solar_on_delta);
    ESP_LOGI(TAG, "solar_off_delta: %d", g_decoded_data.solar_off_delta);
    ESP_LOGI(TAG, "solar_frost_protection: %d", g_decoded_data.solar_frost_protection);
    ESP_LOGI(TAG, "solar_high_limit: %d", g_decoded_data.solar_high_limit);
    
    // Pump/Liquid
    ESP_LOGI(TAG, "pump_flowrate_mode: %u", g_decoded_data.pump_flowrate_mode);
    ESP_LOGI(TAG, "liquid_type: %u", g_decoded_data.liquid_type);
    ESP_LOGI(TAG, "alt_external_sensor: %u", g_decoded_data.alt_external_sensor);
    ESP_LOGI(TAG, "anti_freeze_mode: %u", g_decoded_data.anti_freeze_mode);
    ESP_LOGI(TAG, "optional_pcb: %u", g_decoded_data.optional_pcb);
    
    // Zone sensors
    ESP_LOGI(TAG, "z1_sensor_settings: %u", g_decoded_data.z1_sensor_settings);
    ESP_LOGI(TAG, "z2_sensor_settings: %u", g_decoded_data.z2_sensor_settings);
    
    // External
    ESP_LOGI(TAG, "external_pad_heater: %u", g_decoded_data.external_pad_heater);
    ESP_LOGI(TAG, "water_pressure: %d", g_decoded_data.water_pressure);
    ESP_LOGI(TAG, "external_control: %u", g_decoded_data.external_control);
    ESP_LOGI(TAG, "external_heat_cool_control: %u", g_decoded_data.external_heat_cool_control);
    ESP_LOGI(TAG, "external_error_signal: %u", g_decoded_data.external_error_signal);
    ESP_LOGI(TAG, "external_compressor_control: %u", g_decoded_data.external_compressor_control);
    
    // Pumps
    ESP_LOGI(TAG, "z2_pump_state: %u", g_decoded_data.z2_pump_state);
    ESP_LOGI(TAG, "z1_pump_state: %u", g_decoded_data.z1_pump_state);
    ESP_LOGI(TAG, "two_way_valve_state: %u", g_decoded_data.two_way_valve_state);
    ESP_LOGI(TAG, "three_way_valve_state2: %u", g_decoded_data.three_way_valve_state2);
    
    // PID
    ESP_LOGI(TAG, "z1_valve_pid: %d", g_decoded_data.z1_valve_pid);
    ESP_LOGI(TAG, "z2_valve_pid: %d", g_decoded_data.z2_valve_pid);
    
    // Bivalent
    ESP_LOGI(TAG, "bivalent_control: %u", g_decoded_data.bivalent_control);
    ESP_LOGI(TAG, "bivalent_mode: %u", g_decoded_data.bivalent_mode);
    ESP_LOGI(TAG, "bivalent_start_temp: %d", g_decoded_data.bivalent_start_temp);
    ESP_LOGI(TAG, "bivalent_advanced_heat: %u", g_decoded_data.bivalent_advanced_heat);
    ESP_LOGI(TAG, "bivalent_advanced_dhw: %u", g_decoded_data.bivalent_advanced_dhw);
    ESP_LOGI(TAG, "bivalent_advanced_start_temp: %d", g_decoded_data.bivalent_advanced_start_temp);
    ESP_LOGI(TAG, "bivalent_advanced_stop_temp: %d", g_decoded_data.bivalent_advanced_stop_temp);
    ESP_LOGI(TAG, "bivalent_advanced_start_delay: %u", g_decoded_data.bivalent_advanced_start_delay);
    ESP_LOGI(TAG, "bivalent_advanced_stop_delay: %u", g_decoded_data.bivalent_advanced_stop_delay);
    ESP_LOGI(TAG, "bivalent_advanced_dhw_delay: %u", g_decoded_data.bivalent_advanced_dhw_delay);
    
    // Hours
    ESP_LOGI(TAG, "room_heater_operations_hours: %u", g_decoded_data.room_heater_operations_hours);
    ESP_LOGI(TAG, "dhw_heater_operations_hours: %u", g_decoded_data.dhw_heater_operations_hours);
    
    // Strings
    ESP_LOGI(TAG, "error_state: '%s'", g_decoded_data.error_state);
    ESP_LOGI(TAG, "heat_pump_model: '%s'", g_decoded_data.heat_pump_model);
    
    ESP_LOGI(TAG, "=== END DECODED DATA ===");
}
