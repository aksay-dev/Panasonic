/**
 * @file modbus_params.h
 * @brief Modbus register mapping and parameters - FULL decoder data
 * @version 2.0.0
 * @date 2025
 */

#ifndef MODBUS_PARAMS_H
#define MODBUS_PARAMS_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// INPUT REGISTERS (Read-only) - 0x0000-0x01FF
// ============================================================================
#define MB_REG_INPUT_START 0x0000

// System information (0x0000-0x000F)
#define MB_INPUT_UPTIME_LOW         0x0001
#define MB_INPUT_UPTIME_HIGH        0x0002
#define MB_INPUT_STATUS             0x0003

// ============================================================================
// Basic temperatures (0x0010-0x002F)
// ============================================================================
// int16_t types (stored as * 100)
#define MB_INPUT_MAIN_INLET_TEMP       0x0010  // TOP5, int16
#define MB_INPUT_MAIN_OUTLET_TEMP      0x0011  // TOP6, int16

// int8_t types (stored as whole degrees)
#define MB_INPUT_MAIN_TARGET_TEMP      0x0012  // TOP7, int8
#define MB_INPUT_DHW_TEMP              0x0013  // TOP10, int8
#define MB_INPUT_DHW_TARGET_TEMP       0x0014  // TOP9, int8
#define MB_INPUT_OUTSIDE_TEMP          0x0015  // TOP14, int8
#define MB_INPUT_ROOM_THERMOSTAT_TEMP  0x0016  // TOP33, int8
#define MB_INPUT_BUFFER_TEMP           0x0017  // TOP46, int8
#define MB_INPUT_SOLAR_TEMP            0x0018  // TOP47, int8
#define MB_INPUT_POOL_TEMP             0x0019  // TOP48, int8

// Additional temperatures (0x0020-0x003F)
#define MB_INPUT_MAIN_HEX_OUTLET_TEMP  0x0020  // TOP49, int8 (condensor)
#define MB_INPUT_DISCHARGE_TEMP        0x0021  // TOP50, int8 (compressor)
#define MB_INPUT_INSIDE_PIPE_TEMP      0x0022  // TOP51, int8
#define MB_INPUT_DEFROST_TEMP          0x0023  // TOP52, int8
#define MB_INPUT_EVA_OUTLET_TEMP       0x0024  // TOP53, int8 (evaporator)
#define MB_INPUT_BYPASS_OUTLET_TEMP    0x0025  // TOP54, int8
#define MB_INPUT_IPM_TEMP              0x0026  // TOP55, int8 (inverter)
#define MB_INPUT_OUTSIDE_PIPE_TEMP     0x0027  // TOP21, int8 (outdoor coil)
#define MB_INPUT_Z1_ROOM_TEMP          0x0028  // TOP56, int8
#define MB_INPUT_Z2_ROOM_TEMP          0x0029  // TOP57, int8
#define MB_INPUT_Z1_WATER_TEMP         0x002A  // TOP36, int8
#define MB_INPUT_Z2_WATER_TEMP         0x002B  // TOP37, int8
#define MB_INPUT_Z1_WATER_TARGET_TEMP  0x002C  // TOP42, int8
#define MB_INPUT_Z2_WATER_TARGET_TEMP  0x002D  // TOP43, int8
#define MB_INPUT_SECOND_INLET_TEMP     0x002E  // TOP116, int8
#define MB_INPUT_ECONOMIZER_OUTLET_TEMP 0x002F // TOP117, int8
#define MB_INPUT_SECOND_ROOM_THERMO_TEMP 0x0030 // TOP118, int8

// ============================================================================
// Zone temperature requests (0x0040-0x004F)
// ============================================================================
#define MB_INPUT_Z1_HEAT_REQUEST_TEMP  0x0040  // TOP27, int8
#define MB_INPUT_Z1_COOL_REQUEST_TEMP  0x0041  // TOP28, int8
#define MB_INPUT_Z2_HEAT_REQUEST_TEMP  0x0042  // TOP34, int8
#define MB_INPUT_Z2_COOL_REQUEST_TEMP  0x0043  // TOP35, int8

// ============================================================================
// Zone 1 heating curve (0x0050-0x005F)
// ============================================================================
#define MB_INPUT_Z1_HEAT_CURVE_TARGET_HIGH  0x0050  // TOP29, int8
#define MB_INPUT_Z1_HEAT_CURVE_TARGET_LOW   0x0051  // TOP30, int8
#define MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_HIGH 0x0052  // TOP31, int8
#define MB_INPUT_Z1_HEAT_CURVE_OUTSIDE_LOW  0x0053  // TOP32, int8

// Zone 1 cooling curve (0x0054-0x005F)
#define MB_INPUT_Z1_COOL_CURVE_TARGET_HIGH  0x0054  // TOP72, int8
#define MB_INPUT_Z1_COOL_CURVE_TARGET_LOW   0x0055  // TOP73, int8
#define MB_INPUT_Z1_COOL_CURVE_OUTSIDE_HIGH 0x0056  // TOP74, int8
#define MB_INPUT_Z1_COOL_CURVE_OUTSIDE_LOW  0x0057  // TOP75, int8

// ============================================================================
// Zone 2 heating curve (0x0060-0x006F)
// ============================================================================
#define MB_INPUT_Z2_HEAT_CURVE_TARGET_HIGH  0x0060  // TOP82, int8
#define MB_INPUT_Z2_HEAT_CURVE_TARGET_LOW   0x0061  // TOP83, int8
#define MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_HIGH 0x0062  // TOP84, int8
#define MB_INPUT_Z2_HEAT_CURVE_OUTSIDE_LOW  0x0063  // TOP85, int8

// Zone 2 cooling curve (0x0064-0x006F)
#define MB_INPUT_Z2_COOL_CURVE_TARGET_HIGH  0x0064  // TOP86, int8
#define MB_INPUT_Z2_COOL_CURVE_TARGET_LOW   0x0065  // TOP87, int8
#define MB_INPUT_Z2_COOL_CURVE_OUTSIDE_HIGH 0x0066  // TOP88, int8
#define MB_INPUT_Z2_COOL_CURVE_OUTSIDE_LOW  0x0067  // TOP89, int8

// ============================================================================
// Power and energy (0x0070-0x007F)
// ============================================================================
#define MB_INPUT_HEAT_POWER_PRODUCTION   0x0070  // TOP15, uint16
#define MB_INPUT_HEAT_POWER_CONSUMPTION  0x0071  // TOP16, uint16
#define MB_INPUT_COOL_POWER_PRODUCTION   0x0072  // TOP38, uint16
#define MB_INPUT_COOL_POWER_CONSUMPTION  0x0073  // TOP39, uint16
#define MB_INPUT_DHW_POWER_PRODUCTION    0x0074  // TOP40, uint16
#define MB_INPUT_DHW_POWER_CONSUMPTION   0x0075  // TOP41, uint16

// Extra power data (XTOP0-5)
#define MB_INPUT_HEAT_POWER_CONSUMPTION_EXTRA 0x0076  // XTOP0, uint16
#define MB_INPUT_COOL_POWER_CONSUMPTION_EXTRA 0x0077  // XTOP1, uint16
#define MB_INPUT_DHW_POWER_CONSUMPTION_EXTRA  0x0078  // XTOP2, uint16
#define MB_INPUT_HEAT_POWER_PRODUCTION_EXTRA  0x0079  // XTOP3, uint16
#define MB_INPUT_COOL_POWER_PRODUCTION_EXTRA  0x007A  // XTOP4, uint16
#define MB_INPUT_DHW_POWER_PRODUCTION_EXTRA   0x007B  // XTOP5, uint16

// ============================================================================
// Technical parameters (0x0080-0x009F)
// ============================================================================
#define MB_INPUT_COMPRESSOR_FREQ       0x0080  // TOP8, uint16
#define MB_INPUT_PUMP_FLOW             0x0081  // TOP1, int16 (*100)
#define MB_INPUT_OPERATIONS_HOURS      0x0082  // TOP11, uint16
#define MB_INPUT_OPERATIONS_COUNTER    0x0083  // TOP12, uint16
#define MB_INPUT_FAN1_MOTOR_SPEED      0x0084  // TOP62, uint16
#define MB_INPUT_FAN2_MOTOR_SPEED      0x0085  // TOP63, uint16
#define MB_INPUT_HIGH_PRESSURE         0x0086  // TOP64, int16 (*100)
#define MB_INPUT_PUMP_SPEED            0x0087  // TOP65, uint16
#define MB_INPUT_LOW_PRESSURE          0x0088  // TOP66, int16 (*100)
#define MB_INPUT_COMPRESSOR_CURRENT    0x0089  // TOP67, int16 (*100)
#define MB_INPUT_PUMP_DUTY             0x008A  // TOP93, uint8
#define MB_INPUT_MAX_PUMP_DUTY         0x008B  // TOP95, uint8

// ============================================================================
// Operation states (0x00A0-0x00AF)
// ============================================================================
#define MB_INPUT_HEATPUMP_STATE        0x00A0  // TOP0, uint8
#define MB_INPUT_FORCE_DHW_STATE       0x00A1  // TOP2, uint8
#define MB_INPUT_OPERATING_MODE_STATE  0x00A2  // TOP4, uint8
#define MB_INPUT_QUIET_MODE_SCHEDULE   0x00A3  // TOP3, uint8
#define MB_INPUT_POWERFUL_MODE_TIME    0x00A4  // TOP17, uint8
#define MB_INPUT_QUIET_MODE_LEVEL      0x00A5  // TOP18, uint8
#define MB_INPUT_HOLIDAY_MODE_STATE    0x00A6  // TOP19, uint8
#define MB_INPUT_THREE_WAY_VALVE_STATE 0x00A7  // TOP20, uint8
#define MB_INPUT_DEFROSTING_STATE      0x00A8  // TOP26, uint8
#define MB_INPUT_MAIN_SCHEDULE_STATE   0x00A9  // TOP13, uint8
#define MB_INPUT_ZONES_STATE           0x00AA  // TOP94, uint8

// ============================================================================
// Heaters and sterilization (0x00B0-0x00BF)
// ============================================================================
#define MB_INPUT_DHW_HEATER_STATE      0x00B0  // TOP58, uint8
#define MB_INPUT_ROOM_HEATER_STATE     0x00B1  // TOP59, uint8
#define MB_INPUT_INTERNAL_HEATER_STATE 0x00B2  // TOP60, uint8
#define MB_INPUT_EXTERNAL_HEATER_STATE 0x00B3  // TOP61, uint8
#define MB_INPUT_FORCE_HEATER_STATE    0x00B4  // TOP68, uint8
#define MB_INPUT_STERILIZATION_STATE   0x00B5  // TOP69, uint8
#define MB_INPUT_STERILIZATION_TEMP    0x00B6  // TOP70, int16 (*100)
#define MB_INPUT_STERILIZATION_MAX_TIME 0x00B7 // TOP71, uint8

// ============================================================================
// Deltas and shifts (0x00C0-0x00CF) - all int16 (*100)
// ============================================================================
#define MB_INPUT_DHW_HEAT_DELTA        0x00C0  // TOP22, int16 (*100)
#define MB_INPUT_HEAT_DELTA            0x00C1  // TOP23, int16 (*100)
#define MB_INPUT_COOL_DELTA            0x00C2  // TOP24, int16 (*100)
#define MB_INPUT_DHW_HOLIDAY_SHIFT_TEMP 0x00C3 // TOP25, int16 (*100)
#define MB_INPUT_ROOM_HOLIDAY_SHIFT_TEMP 0x00C4 // TOP45, int16 (*100)
#define MB_INPUT_BUFFER_TANK_DELTA     0x00C5  // TOP113, int16 (*100)

// ============================================================================
// Heating/Cooling mode settings (0x00D0-0x00DF)
// ============================================================================
#define MB_INPUT_HEATING_MODE          0x00D0  // TOP76, uint8
#define MB_INPUT_HEATING_OFF_OUTDOOR_TEMP 0x00D1 // TOP77, int16 (*100)
#define MB_INPUT_HEATER_ON_OUTDOOR_TEMP   0x00D2 // TOP78, int16 (*100)
#define MB_INPUT_HEAT_TO_COOL_TEMP     0x00D3  // TOP79, int16 (*100)
#define MB_INPUT_COOL_TO_HEAT_TEMP     0x00D4  // TOP80, int16 (*100)
#define MB_INPUT_COOLING_MODE          0x00D5  // TOP81, uint8

// ============================================================================
// Solar and buffer settings (0x00E0-0x00EF)
// ============================================================================
#define MB_INPUT_BUFFER_INSTALLED      0x00E0  // TOP99, uint8
#define MB_INPUT_DHW_INSTALLED         0x00E1  // TOP100, uint8
#define MB_INPUT_SOLAR_MODE            0x00E2  // TOP101, uint8
#define MB_INPUT_SOLAR_ON_DELTA        0x00E3  // TOP102, int16 (*100)
#define MB_INPUT_SOLAR_OFF_DELTA       0x00E4  // TOP103, int16 (*100)
#define MB_INPUT_SOLAR_FROST_PROTECTION 0x00E5 // TOP104, int16 (*100)
#define MB_INPUT_SOLAR_HIGH_LIMIT      0x00E6  // TOP105, int16 (*100)

// ============================================================================
// Pump and liquid settings (0x00F0-0x00FF)
// ============================================================================
#define MB_INPUT_PUMP_FLOWRATE_MODE    0x00F0  // TOP106, uint8
#define MB_INPUT_LIQUID_TYPE           0x00F1  // TOP107, uint8
#define MB_INPUT_ALT_EXTERNAL_SENSOR   0x00F2  // TOP108, uint8
#define MB_INPUT_ANTI_FREEZE_MODE      0x00F3  // TOP109, uint8
#define MB_INPUT_OPTIONAL_PCB          0x00F4  // TOP110, uint8
#define MB_INPUT_Z1_SENSOR_SETTINGS    0x00F5  // TOP111, uint8
#define MB_INPUT_Z2_SENSOR_SETTINGS    0x00F6  // TOP112, uint8

// ============================================================================
// External controls (0x0100-0x010F)
// ============================================================================
#define MB_INPUT_EXTERNAL_PAD_HEATER   0x0100  // TOP114, uint8
#define MB_INPUT_WATER_PRESSURE        0x0101  // TOP115, int16 (*100)
#define MB_INPUT_EXTERNAL_CONTROL      0x0102  // TOP119, uint8
#define MB_INPUT_EXTERNAL_HEAT_COOL_CONTROL 0x0103 // TOP120, uint8
#define MB_INPUT_EXTERNAL_ERROR_SIGNAL 0x0104  // TOP121, uint8
#define MB_INPUT_EXTERNAL_COMPRESSOR_CONTROL 0x0105 // TOP122, uint8

// ============================================================================
// Pump and valve states (0x0110-0x011F)
// ============================================================================
#define MB_INPUT_Z2_PUMP_STATE         0x0110  // TOP123, uint8
#define MB_INPUT_Z1_PUMP_STATE         0x0111  // TOP124, uint8
#define MB_INPUT_TWO_WAY_VALVE_STATE   0x0112  // TOP125, uint8
#define MB_INPUT_THREE_WAY_VALVE_STATE2 0x0113 // TOP126, uint8
#define MB_INPUT_Z1_VALVE_PID          0x0114  // TOP127, int16 (*100)
#define MB_INPUT_Z2_VALVE_PID          0x0115  // TOP128, int16 (*100)

// ============================================================================
// Bivalent settings (0x0120-0x012F)
// ============================================================================
#define MB_INPUT_BIVALENT_CONTROL      0x0120  // TOP129, uint8
#define MB_INPUT_BIVALENT_MODE         0x0121  // TOP130, uint8
#define MB_INPUT_BIVALENT_START_TEMP   0x0122  // TOP131, int16 (*100)
#define MB_INPUT_BIVALENT_ADVANCED_HEAT 0x0123 // TOP132, uint8
#define MB_INPUT_BIVALENT_ADVANCED_DHW 0x0124  // TOP133, uint8
#define MB_INPUT_BIVALENT_ADVANCED_START_TEMP 0x0125 // TOP134, int16 (*100)
#define MB_INPUT_BIVALENT_ADVANCED_STOP_TEMP  0x0126 // TOP135, int16 (*100)
#define MB_INPUT_BIVALENT_ADVANCED_START_DELAY 0x0127 // TOP136, uint8
#define MB_INPUT_BIVALENT_ADVANCED_STOP_DELAY  0x0128 // TOP137, uint8
#define MB_INPUT_BIVALENT_ADVANCED_DHW_DELAY   0x0129 // TOP138, uint8

// ============================================================================
// Heater timing settings (0x0130-0x013F)
// ============================================================================
#define MB_INPUT_HEATER_DELAY_TIME     0x0130  // TOP96, uint8
#define MB_INPUT_HEATER_START_DELTA    0x0131  // TOP97, int16 (*100)
#define MB_INPUT_HEATER_STOP_DELTA     0x0132  // TOP98, int16 (*100)

// ============================================================================
// Error and model info (0x0140-0x014F)
// ============================================================================
// Error state: 8 registers (16 bytes for char[16])
#define MB_INPUT_ERROR_STATE_0         0x0140  // TOP44[0-1]
#define MB_INPUT_ERROR_STATE_1         0x0141  // TOP44[2-3]
#define MB_INPUT_ERROR_STATE_2         0x0142  // TOP44[4-5]
#define MB_INPUT_ERROR_STATE_3         0x0143  // TOP44[6-7]
#define MB_INPUT_ERROR_STATE_4         0x0144  // TOP44[8-9]
#define MB_INPUT_ERROR_STATE_5         0x0145  // TOP44[10-11]
#define MB_INPUT_ERROR_STATE_6         0x0146  // TOP44[12-13]
#define MB_INPUT_ERROR_STATE_7         0x0147  // TOP44[14-15]

// ============================================================================
// Heat pump model (0x0148-0x0157) - 16 registers (32 bytes for char[32])
// ============================================================================
#define MB_INPUT_HP_MODEL_0            0x0148  // TOP92[0-1]
#define MB_INPUT_HP_MODEL_1            0x0149  // TOP92[2-3]
#define MB_INPUT_HP_MODEL_2            0x014A  // TOP92[4-5]
#define MB_INPUT_HP_MODEL_3            0x014B  // TOP92[6-7]
#define MB_INPUT_HP_MODEL_4            0x014C  // TOP92[8-9]
#define MB_INPUT_HP_MODEL_5            0x014D  // TOP92[10-11]
#define MB_INPUT_HP_MODEL_6            0x014E  // TOP92[12-13]
#define MB_INPUT_HP_MODEL_7            0x014F  // TOP92[14-15]
#define MB_INPUT_HP_MODEL_8            0x0150  // TOP92[16-17]
#define MB_INPUT_HP_MODEL_9            0x0151  // TOP92[18-19]
#define MB_INPUT_HP_MODEL_10           0x0152  // TOP92[20-21]
#define MB_INPUT_HP_MODEL_11           0x0153  // TOP92[22-23]
#define MB_INPUT_HP_MODEL_12           0x0154  // TOP92[24-25]
#define MB_INPUT_HP_MODEL_13           0x0155  // TOP92[26-27]
#define MB_INPUT_HP_MODEL_14           0x0156  // TOP92[28-29]
#define MB_INPUT_HP_MODEL_15           0x0157  // TOP92[30-31]

// ============================================================================
// Operation hours (0x0158-0x015F)
// ============================================================================
#define MB_INPUT_ROOM_HEATER_OPS_HOURS 0x0158  // TOP90, uint16
#define MB_INPUT_DHW_HEATER_OPS_HOURS  0x0159  // TOP91, uint16

// ============================================================================
// Optional PCB data (0x0160-0x016F)
// ============================================================================
#define MB_INPUT_Z1_WATER_PUMP         0x0160  // OPT0, uint8
#define MB_INPUT_Z1_MIXING_VALVE       0x0161  // OPT1, uint8
#define MB_INPUT_Z2_WATER_PUMP         0x0162  // OPT2, uint8
#define MB_INPUT_Z2_MIXING_VALVE       0x0163  // OPT3, uint8
#define MB_INPUT_POOL_WATER_PUMP       0x0164  // OPT4, uint8
#define MB_INPUT_SOLAR_WATER_PUMP      0x0165  // OPT5, uint8
#define MB_INPUT_ALARM_STATE           0x0166  // OPT6, uint8

// Total input registers
#define MB_REG_INPUT_COUNT             0x0170  // 368 registers (0x0000-0x016F)

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

// Update total count to cover up to last defined register
#define MB_REG_HOLDING_COUNT            0x0080  // headroom to cover new ranges

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
 * @brief Update input registers from decoded heat pump data
 * @return ESP_OK on success
 */
esp_err_t modbus_params_update_inputs(void);

/**
 * @brief Process holding register writes (execute commands)
 * @param reg_addr Register address that was written
 * @return ESP_OK on success
 */
esp_err_t modbus_params_process_holding_write(uint16_t reg_addr);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_PARAMS_H
