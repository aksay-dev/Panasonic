/**
 * @file modbus_params.h
 * @brief Modbus register mapping and parameters - FULL decoder data
 * @version 2.0.0
 * @date 2025
 */

#ifndef MODBUS_PARAMS_H
#define MODBUS_PARAMS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// INPUT REGISTERS (Read-only) - 0x0000-0x01FF
// ============================================================================
#define MB_REG_INPUT_START 0x0000

// System information (0x0000-0x000F)
#define MB_INPUT_STATUS             0x0003
#define MB_INPUT_EXTENDED_DATA      0x0004  // Если 1, то данные расширенные

// ============================================================================
// Basic temperatures (0x0010-0x002F)
// ============================================================================
// int16_t types (stored as * 100)
#define MB_INPUT_MAIN_INLET_TEMP       0x0010
#define MB_INPUT_MAIN_OUTLET_TEMP      0x0011

// int8_t types (stored as whole degrees)
#define MB_INPUT_MAIN_TARGET_TEMP      0x0012
#define MB_INPUT_DHW_TEMP              0x0013
#define MB_INPUT_DHW_TARGET_TEMP       0x0014
#define MB_INPUT_OUTSIDE_TEMP          0x0015
#define MB_INPUT_ROOM_THERMOSTAT_TEMP  0x0016
#define MB_INPUT_BUFFER_TEMP           0x0017
#define MB_INPUT_SOLAR_TEMP            0x0018
#define MB_INPUT_POOL_TEMP             0x0019

// Additional temperatures (0x0020-0x003F)
#define MB_INPUT_MAIN_HEX_OUTLET_TEMP  0x0020
#define MB_INPUT_DISCHARGE_TEMP        0x0021
#define MB_INPUT_INSIDE_PIPE_TEMP      0x0022
#define MB_INPUT_DEFROST_TEMP          0x0023
#define MB_INPUT_EVA_OUTLET_TEMP       0x0024
#define MB_INPUT_BYPASS_OUTLET_TEMP    0x0025
#define MB_INPUT_IPM_TEMP              0x0026
#define MB_INPUT_OUTSIDE_PIPE_TEMP     0x0027
#define MB_INPUT_Z1_ROOM_TEMP          0x0028
#define MB_INPUT_Z2_ROOM_TEMP          0x0029
#define MB_INPUT_Z1_WATER_TEMP         0x002A
#define MB_INPUT_Z2_WATER_TEMP         0x002B
#define MB_INPUT_Z1_WATER_TARGET_TEMP  0x002C
#define MB_INPUT_Z2_WATER_TARGET_TEMP  0x002D
#define MB_INPUT_SECOND_INLET_TEMP     0x002E
#define MB_INPUT_ECONOMIZER_OUTLET_TEMP 0x002F
#define MB_INPUT_SECOND_ROOM_THERMO_TEMP 0x0030

// ============================================================================
// Zone temperature requests (0x0040-0x004F)
// ============================================================================
#define MB_INPUT_Z1_HEAT_REQUEST_TEMP  0x0040
#define MB_INPUT_Z1_COOL_REQUEST_TEMP  0x0041
#define MB_INPUT_Z2_HEAT_REQUEST_TEMP  0x0042
#define MB_INPUT_Z2_COOL_REQUEST_TEMP  0x0043

// ============================================================================
// Zone 1 heating curve (0x0050-0x005F)
// ============================================================================
#define MB_INPUT_Z1_HEAT_CURVE_TARGET_HIGH  0x0050
#define MB_INPUT_Z1_HEAT_CURVE_TARGET_LOW   0x0051
#define MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_HIGH 0x0052
#define MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_LOW  0x0053

// Zone 1 cooling curve (0x0054-0x005F)
#define MB_INPUT_Z1_COOL_CURVE_TARGET_HIGH  0x0054
#define MB_INPUT_Z1_COOL_CURVE_TARGET_LOW   0x0055
#define MB_INPUT_Z1_COOL_CURVE_OUTSIDE_HIGH 0x0056
#define MB_INPUT_Z1_COOL_CURVE_OUTSIDE_LOW  0x0057

// ============================================================================
// Zone 2 heating curve (0x0060-0x006F)
// ============================================================================
#define MB_INPUT_Z2_HEAT_CURVE_TARGET_HIGH  0x0060
#define MB_INPUT_Z2_HEAT_CURVE_TARGET_LOW   0x0061
#define MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_HIGH 0x0062
#define MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_LOW  0x0063

// Zone 2 cooling curve (0x0064-0x006F)
#define MB_INPUT_Z2_COOL_CURVE_TARGET_HIGH  0x0064
#define MB_INPUT_Z2_COOL_CURVE_TARGET_LOW   0x0065
#define MB_INPUT_Z2_COOL_CURVE_OUTSIDE_HIGH 0x0066
#define MB_INPUT_Z2_COOL_CURVE_OUTSIDE_LOW  0x0067

// ============================================================================
// Power and energy (0x0070-0x007F)
// ============================================================================
#define MB_INPUT_HEAT_POWER_PRODUCTION   0x0070
#define MB_INPUT_HEAT_POWER_CONSUMPTION  0x0071
#define MB_INPUT_COOL_POWER_PRODUCTION   0x0072
#define MB_INPUT_COOL_POWER_CONSUMPTION  0x0073
#define MB_INPUT_DHW_POWER_PRODUCTION    0x0074
#define MB_INPUT_DHW_POWER_CONSUMPTION   0x0075

// Extra power data (XTOP0-5)
#define MB_INPUT_HEAT_POWER_CONSUMPTION_EXTRA 0x0076
#define MB_INPUT_COOL_POWER_CONSUMPTION_EXTRA 0x0077
#define MB_INPUT_DHW_POWER_CONSUMPTION_EXTRA  0x0078
#define MB_INPUT_HEAT_POWER_PRODUCTION_EXTRA  0x0079
#define MB_INPUT_COOL_POWER_PRODUCTION_EXTRA  0x007A
#define MB_INPUT_DHW_POWER_PRODUCTION_EXTRA   0x007B

// ============================================================================
// Technical parameters (0x0080-0x009F)
// ============================================================================
#define MB_INPUT_COMPRESSOR_FREQ       0x0080
#define MB_INPUT_PUMP_FLOW             0x0081
#define MB_INPUT_OPERATIONS_HOURS      0x0082
#define MB_INPUT_OPERATIONS_COUNTER    0x0083
#define MB_INPUT_FAN1_MOTOR_SPEED      0x0084
#define MB_INPUT_FAN2_MOTOR_SPEED      0x0085
#define MB_INPUT_HIGH_PRESSURE         0x0086
#define MB_INPUT_PUMP_SPEED            0x0087
#define MB_INPUT_LOW_PRESSURE          0x0088
#define MB_INPUT_COMPRESSOR_CURRENT    0x0089
#define MB_INPUT_PUMP_DUTY             0x008A
#define MB_INPUT_MAX_PUMP_DUTY         0x008B

// ============================================================================
// Operation states (0x00A0-0x00AF)
// ============================================================================
#define MB_INPUT_HEATPUMP_STATE        0x00A0
#define MB_INPUT_FORCE_DHW_STATE       0x00A1
#define MB_INPUT_OPERATING_MODE_STATE  0x00A2
#define MB_INPUT_QUIET_MODE_SCHEDULE   0x00A3
#define MB_INPUT_POWERFUL_MODE_TIME    0x00A4
#define MB_INPUT_QUIET_MODE_LEVEL      0x00A5
#define MB_INPUT_HOLIDAY_MODE_STATE    0x00A6
#define MB_INPUT_THREE_WAY_VALVE_STATE 0x00A7
#define MB_INPUT_DEFROSTING_STATE      0x00A8
#define MB_INPUT_MAIN_SCHEDULE_STATE   0x00A9
#define MB_INPUT_ZONES_STATE           0x00AA

// ============================================================================
// Heaters and sterilization (0x00B0-0x00BF)
// ============================================================================
#define MB_INPUT_DHW_HEATER_STATE      0x00B0
#define MB_INPUT_ROOM_HEATER_STATE     0x00B1
#define MB_INPUT_INTERNAL_HEATER_STATE 0x00B2
#define MB_INPUT_EXTERNAL_HEATER_STATE 0x00B3
#define MB_INPUT_FORCE_HEATER_STATE    0x00B4
#define MB_INPUT_STERILIZATION_STATE   0x00B5
#define MB_INPUT_STERILIZATION_TEMP    0x00B6
#define MB_INPUT_STERILIZATION_MAX_TIME 0x00B7

// ============================================================================
// Deltas and shifts (0x00C0-0x00CF) - all int16 (*100)
// ============================================================================
#define MB_INPUT_DHW_HEAT_DELTA        0x00C0
#define MB_INPUT_HEAT_DELTA            0x00C1
#define MB_INPUT_COOL_DELTA            0x00C2
#define MB_INPUT_DHW_HOLIDAY_SHIFT_TEMP 0x00C3
#define MB_INPUT_ROOM_HOLIDAY_SHIFT_TEMP 0x00C4
#define MB_INPUT_BUFFER_TANK_DELTA     0x00C5

// ============================================================================
// Heating/Cooling mode settings (0x00D0-0x00DF)
// ============================================================================
#define MB_INPUT_HEATING_MODE          0x00D0
#define MB_INPUT_HEATING_OFF_OUTDOOR_TEMP 0x00D1
#define MB_INPUT_HEATER_ON_OUTDOOR_TEMP   0x00D2
#define MB_INPUT_HEAT_TO_COOL_TEMP     0x00D3
#define MB_INPUT_COOL_TO_HEAT_TEMP     0x00D4
#define MB_INPUT_COOLING_MODE          0x00D5

// ============================================================================
// Solar and buffer settings (0x00E0-0x00EF)
// ============================================================================
#define MB_INPUT_BUFFER_INSTALLED      0x00E0
#define MB_INPUT_DHW_INSTALLED         0x00E1
#define MB_INPUT_SOLAR_MODE            0x00E2
#define MB_INPUT_SOLAR_ON_DELTA        0x00E3
#define MB_INPUT_SOLAR_OFF_DELTA       0x00E4
#define MB_INPUT_SOLAR_FROST_PROTECTION 0x00E5
#define MB_INPUT_SOLAR_HIGH_LIMIT      0x00E6

// ============================================================================
// Pump and liquid settings (0x00F0-0x00FF)
// ============================================================================
#define MB_INPUT_PUMP_FLOWRATE_MODE    0x00F0
#define MB_INPUT_LIQUID_TYPE           0x00F1
#define MB_INPUT_ALT_EXTERNAL_SENSOR   0x00F2
#define MB_INPUT_ANTI_FREEZE_MODE      0x00F3
#define MB_INPUT_OPTIONAL_PCB          0x00F4
#define MB_INPUT_Z1_SENSOR_SETTINGS    0x00F5
#define MB_INPUT_Z2_SENSOR_SETTINGS    0x00F6

// ============================================================================
// External controls (0x0100-0x010F)
// ============================================================================
#define MB_INPUT_EXTERNAL_PAD_HEATER   0x0100
#define MB_INPUT_WATER_PRESSURE        0x0101
#define MB_INPUT_EXTERNAL_CONTROL      0x0102
#define MB_INPUT_EXTERNAL_HEAT_COOL_CONTROL 0x0103
#define MB_INPUT_EXTERNAL_ERROR_SIGNAL 0x0104
#define MB_INPUT_EXTERNAL_COMPRESSOR_CONTROL 0x0105

// ============================================================================
// Pump and valve states (0x0110-0x011F)
// ============================================================================
#define MB_INPUT_Z2_PUMP_STATE         0x0110
#define MB_INPUT_Z1_PUMP_STATE         0x0111
#define MB_INPUT_TWO_WAY_VALVE_STATE   0x0112
#define MB_INPUT_THREE_WAY_VALVE_STATE2 0x0113
#define MB_INPUT_Z1_VALVE_PID          0x0114
#define MB_INPUT_Z2_VALVE_PID          0x0115

// ============================================================================
// Bivalent settings (0x0120-0x012F)
// ============================================================================
#define MB_INPUT_BIVALENT_CONTROL      0x0120
#define MB_INPUT_BIVALENT_MODE         0x0121
#define MB_INPUT_BIVALENT_START_TEMP   0x0122
#define MB_INPUT_BIVALENT_ADVANCED_HEAT 0x0123
#define MB_INPUT_BIVALENT_ADVANCED_DHW 0x0124
#define MB_INPUT_BIVALENT_ADVANCED_START_TEMP 0x0125
#define MB_INPUT_BIVALENT_ADVANCED_STOP_TEMP  0x0126
#define MB_INPUT_BIVALENT_ADVANCED_START_DELAY 0x0127
#define MB_INPUT_BIVALENT_ADVANCED_STOP_DELAY  0x0128
#define MB_INPUT_BIVALENT_ADVANCED_DHW_DELAY   0x0129

// ============================================================================
// Heater timing settings (0x0130-0x013F)
// ============================================================================
#define MB_INPUT_HEATER_DELAY_TIME     0x0130
#define MB_INPUT_HEATER_START_DELTA    0x0131
#define MB_INPUT_HEATER_STOP_DELTA     0x0132

// ============================================================================
// Error info (0x0140-0x0141)
// ============================================================================
// Error state
#define MB_INPUT_ERROR_TYPE         0x0140
#define MB_INPUT_ERROR_NUMBER       0x0141

// ============================================================================
// Heat pump model (0x0150-0x0154) - 5 registers (10 bytes)
// ============================================================================
#define MB_INPUT_HP_MODEL_0            0x0150
#define MB_INPUT_HP_MODEL_1            0x0151
#define MB_INPUT_HP_MODEL_2            0x0152
#define MB_INPUT_HP_MODEL_3            0x0153
#define MB_INPUT_HP_MODEL_4            0x0154

// ============================================================================
// Operation hours (0x0158-0x015F)
// ============================================================================
#define MB_INPUT_ROOM_HEATER_OPS_HOURS 0x0158
#define MB_INPUT_DHW_HEATER_OPS_HOURS  0x0159

// ============================================================================
// Optional PCB data (0x0160-0x016F)
// ============================================================================
#define MB_INPUT_Z1_WATER_PUMP         0x0160
#define MB_INPUT_Z1_MIXING_VALVE       0x0161
#define MB_INPUT_Z2_WATER_PUMP         0x0162
#define MB_INPUT_Z2_MIXING_VALVE       0x0163
#define MB_INPUT_POOL_WATER_PUMP       0x0164
#define MB_INPUT_SOLAR_WATER_PUMP      0x0165
#define MB_INPUT_ALARM_STATE           0x0166

// Registers copy fo fast reading
// Basic temperatures
#define MB_INPUT_MAIN_INLET_TEMP_CPY            0x0170
#define MB_INPUT_MAIN_OUTLET_TEMP_CPY           0x0171
#define MB_INPUT_MAIN_TARGET_TEMP_CPY           0x0172
#define MB_INPUT_DHW_TARGET_TEMP_CPY            0x0173
#define MB_INPUT_OUTSIDE_TEMP_CPY               0x0174
// Additional temperatures
#define MB_INPUT_INSIDE_PIPE_TEMP_CPY           0x0175
#define MB_INPUT_OUTSIDE_PIPE_TEMP_CPY          0x0176
// Power and energy
#define MB_INPUT_HEAT_POWER_CONSUMPTION_CPY     0x0177
#define MB_INPUT_COOL_POWER_CONSUMPTION_CPY     0x0178
#define MB_INPUT_DHW_POWER_CONSUMPTION_CPY      0x0179
// ==================================================================
// Technical parameters
// ==================================================================
#define MB_INPUT_COMPRESSOR_FREQ_CPY            0x017A
#define MB_INPUT_PUMP_FLOW_CPY                  0x017B
#define MB_INPUT_OPERATIONS_HOURS_CPY           0x017C
#define MB_INPUT_OPERATIONS_COUNTER_CPY         0x017D
#define MB_INPUT_PUMP_SPEED_CPY                 0x017E
#define MB_INPUT_COMPRESSOR_CURRENT_CPY         0x017F
#define MB_INPUT_PUMP_DUTY_CPY                  0x0180
// ============================================================================
// Operation states
// ============================================================================
#define MB_INPUT_HEATPUMP_STATE_CPY             0x0181
#define MB_INPUT_FORCE_DHW_STATE_CPY            0x0182
#define MB_INPUT_OPERATING_MODE_STATE_CPY       0x0183
#define MB_INPUT_THREE_WAY_VALVE_STATE_CPY      0x0184
#define MB_INPUT_DEFROSTING_STATE_CPY           0x0185
// ==================================================================
// Heating/Cooling mode settings
// ==================================================================
#define MB_INPUT_HEATING_MODE_CPY               0x0186
#define MB_INPUT_COOLING_MODE_CPY               0x0187
// ==================================================================
// External controls
// ==================================================================
#define MB_INPUT_WATER_PRESSURE_CPY             0x0188
#define MB_INPUT_EXTERNAL_CONTROL_CPY           0x0189
#define MB_INPUT_EXTERNAL_ERROR_SIGNAL_CPY      0x018A
// ==================================================================
// Pump and valve states
// ==================================================================
#define MB_INPUT_TWO_WAY_VALVE_STATE_CPY        0x018B
#define MB_INPUT_THREE_WAY_VALVE_STATE2_CPY     0x018C
// ==================================================================
// Error info
// ==================================================================
// Error state
#define MB_INPUT_ERROR_TYPE_CPY         0x018D
#define MB_INPUT_ERROR_NUMBER_CPY       0x018E

// ADC analog inputs (0x0190-0x0192)
#define MB_INPUT_ADC_AIN                0x0190  // GPIO32
#define MB_INPUT_ADC_NTC1               0x0191  // GPIO34
#define MB_INPUT_ADC_NTC2               0x0192  // GPIO35

// DS18B20 temperature sensor (0x0193)
#define MB_INPUT_DS18B20_TEMP           0x0193  // GPIO22, temperature in °C * 100

// Total input registers
#define MB_REG_INPUT_COUNT             0x0194  // 404 registers (0x0000-0x0193)

// ============================================================================
// HOLDING REGISTERS (Read/Write) - 0x1000-0x103F
// ============================================================================

#define MB_REG_HOLDING_START            0x1000

// Control commands (0x1000-0x100F)
#define MB_HOLDING_SET_HEATPUMP         0x1000
#define MB_HOLDING_SET_PUMP             0x1001
#define MB_HOLDING_SET_MAX_PUMP_DUTY    0x1002
#define MB_HOLDING_SET_QUIET_MODE       0x1003
#define MB_HOLDING_SET_POWERFUL_MODE    0x1004
#define MB_HOLDING_SET_OPERATION_MODE   0x1005
#define MB_HOLDING_SET_HOLIDAY_MODE     0x1006
#define MB_HOLDING_SET_FORCE_DHW        0x1007
#define MB_HOLDING_SET_FORCE_DEFROST    0x1008
#define MB_HOLDING_SET_FORCE_STERILIZATION 0x1009
#define MB_HOLDING_SET_MAIN_SCHEDULE    0x100A
#define MB_HOLDING_SET_RESET            0x100B
#define MB_HOLDING_SET_ZONES            0x100C

// External controls (0x100D-0x100F)
#define MB_HOLDING_SET_EXTERNAL_CONTROL            0x100D
#define MB_HOLDING_SET_EXTERNAL_ERROR              0x100E
#define MB_HOLDING_SET_EXTERNAL_COMPRESSOR_CONTROL 0x100F

// Additional controls (0x1010-0x1015)
#define MB_HOLDING_SET_EXTERNAL_HEAT_COOL_CONTROL  0x1010
#define MB_HOLDING_SET_BIVALENT_CONTROL            0x1011
#define MB_HOLDING_SET_BIVALENT_MODE               0x1012
#define MB_HOLDING_SET_ALT_EXTERNAL_SENSOR         0x1013
#define MB_HOLDING_SET_EXTERNAL_PAD_HEATER         0x1014
#define MB_HOLDING_SET_BUFFER                      0x1015

// Temperature setpoints (0x1020-0x1024) - all in °C (int8)
#define MB_HOLDING_SET_Z1_HEAT_TEMP     0x1020
#define MB_HOLDING_SET_Z1_COOL_TEMP     0x1021
#define MB_HOLDING_SET_Z2_HEAT_TEMP     0x1022
#define MB_HOLDING_SET_Z2_COOL_TEMP     0x1023
#define MB_HOLDING_SET_DHW_TEMP         0x1024

// Deltas and timing (0x1030-0x1036)
#define MB_HOLDING_SET_BUFFER_DELTA         0x1030
#define MB_HOLDING_SET_FLOOR_HEAT_DELTA     0x1031
#define MB_HOLDING_SET_FLOOR_COOL_DELTA     0x1032
#define MB_HOLDING_SET_DHW_HEAT_DELTA       0x1033
#define MB_HOLDING_SET_HEATER_START_DELTA   0x1034
#define MB_HOLDING_SET_HEATER_STOP_DELTA    0x1035
#define MB_HOLDING_SET_HEATER_DELAY_TIME    0x1036

// Bivalent temperatures (advanced, °C *100 ints in decoder, but commands take int8)
#define MB_HOLDING_SET_BIVALENT_START_TEMP      0x1037
#define MB_HOLDING_SET_BIVALENT_AP_START_TEMP   0x1038
#define MB_HOLDING_SET_BIVALENT_AP_STOP_TEMP    0x1039

// Optional temperatures (float in commands, we pass int16 raw °C) 0x1040-0x1046
#define MB_HOLDING_SET_POOL_TEMP            0x1040
#define MB_HOLDING_SET_BUFFER_TEMP          0x1041
#define MB_HOLDING_SET_Z1_ROOM_TEMP         0x1042
#define MB_HOLDING_SET_Z1_WATER_TEMP        0x1043
#define MB_HOLDING_SET_Z2_ROOM_TEMP         0x1044
#define MB_HOLDING_SET_Z2_WATER_TEMP        0x1045
#define MB_HOLDING_SET_SOLAR_TEMP           0x1046

// Optional controls (0x1050-0x1055)
#define MB_HOLDING_SET_HEAT_COOL_MODE       0x1050
#define MB_HOLDING_SET_COMPRESSOR_STATE     0x1051
#define MB_HOLDING_SET_SMART_GRID_MODE      0x1052
#define MB_HOLDING_SET_EXT_THERMOSTAT_1     0x1053
#define MB_HOLDING_SET_EXT_THERMOSTAT_2     0x1054
#define MB_HOLDING_SET_DEMAND_CONTROL       0x1055

// Curves block (write data then trigger apply)
#define MB_HOLDING_CURVES_START             0x1060  // 16 registers (32 bytes)
#define MB_HOLDING_CURVES_REGS              16
#define MB_HOLDING_CURVES_APPLY             0x1070  // write any value to apply

// Modbus serial configuration (0x1080-0x1084)
// Изменения сохраняются в NVS и применяются при следующей перезагрузке
#define MB_HOLDING_SET_MODBUS_BAUD          0x1080  // uint16, baudrate (1200-57600)
#define MB_HOLDING_SET_MODBUS_PARITY        0x1081  // 0=None, 1=Even, 2=Odd
#define MB_HOLDING_SET_MODBUS_STOP_BITS     0x1082  // 1 or 2 stop bits
#define MB_HOLDING_SET_MODBUS_DATA_BITS     0x1083  // 7 or 8 data bits
#define MB_HOLDING_SET_MODBUS_SLAVE_ID      0x1084  // 1-247

#define MB_HOLDING_OPT_PCB_AVAILABLE        0x1090  // == 1 - Есть опциональная плата, включить обработку
#define MB_HOLDING_SET_MQTT_PUBLISH         0x1091  // 1= включить публикацию в MQTT

// Update total count to cover up to last defined register (0x1090)
// Using 0xA0 (160) for safety margin
#define MB_REG_HOLDING_COUNT            0x00A0  // covers 0x1000-0x109F (160 registers)

// ============================================================================
// Register data structures
// ============================================================================

extern int16_t mb_input_registers[MB_REG_INPUT_COUNT];
extern int16_t mb_holding_registers[MB_REG_HOLDING_COUNT];

// ============================================================================
// Function prototypes
// ============================================================================

/**
 * @brief Initialize Modbus parameter structures
 * @return ESP_OK on success
 */
esp_err_t modbus_params_init(void);

/**
 * @brief Process holding register writes (execute commands)
 * @param reg_addr Register address that was written
 * @return ESP_OK on success
 */
esp_err_t modbus_params_process_holding_write(uint16_t reg_addr);

/**
 * @brief Sync holding registers with current serial configuration
 */
void modbus_params_sync_serial_registers(void);

/**
 * @brief Sync holding registers with current decoded heat pump data
 * This allows reading current values (temperatures, deltas, etc.) from holding registers
 */
void modbus_params_sync_holding_from_input(void);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_PARAMS_H
