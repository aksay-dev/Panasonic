/**
 * @file modbus_params.h
 * @brief Modbus register mapping and parameters
 * @version 1.0.0
 * @date 2025
 */

#ifndef MODBUS_PARAMS_H
#define MODBUS_PARAMS_H

#include <stdint.h>
#include "esp_modbus_slave.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// INPUT REGISTERS (Read-only) - 0x0000-0x00FF
// ============================================================================

// System information (0x0000-0x000F)
#define MB_REG_INPUT_START          0x0000
#define MB_INPUT_FREE_MEMORY        0x0003
#define MB_INPUT_UPTIME_LOW         0x0004
#define MB_INPUT_UPTIME_HIGH        0x0005
#define MB_INPUT_STATUS             0x0006
#define MB_INPUT_LAST_UPDATE_LOW    0x0007
#define MB_INPUT_LAST_UPDATE_HIGH   0x0008

// Temperatures (0x0010-0x001F) - all in °C*100
#define MB_INPUT_MAIN_INLET_TEMP       0x0010
#define MB_INPUT_MAIN_OUTLET_TEMP      0x0011
#define MB_INPUT_DHW_TEMP              0x0012
#define MB_INPUT_OUTSIDE_TEMP          0x0013
#define MB_INPUT_BUFFER_TEMP           0x0014
#define MB_INPUT_SOLAR_TEMP            0x0015
#define MB_INPUT_POOL_TEMP             0x0016
#define MB_INPUT_Z1_ROOM_TEMP          0x0017
#define MB_INPUT_Z1_WATER_TEMP         0x0018
#define MB_INPUT_Z2_ROOM_TEMP          0x0019
#define MB_INPUT_Z2_WATER_TEMP         0x001A
#define MB_INPUT_COMPRESSOR_TEMP       0x001B
#define MB_INPUT_EVAPORATOR_TEMP       0x001C
#define MB_INPUT_CONDENSOR_TEMP        0x001D
#define MB_INPUT_INVERTER_TEMP         0x001E
#define MB_INPUT_OUTDOOR_COIL_TEMP     0x001F

// Power and energy (0x0020-0x002D)
#define MB_INPUT_HEAT_POWER_PRODUCTION   0x0020
#define MB_INPUT_HEAT_POWER_CONSUMPTION  0x0021
#define MB_INPUT_COOL_POWER_PRODUCTION   0x0022
#define MB_INPUT_COOL_POWER_CONSUMPTION  0x0023
#define MB_INPUT_DHW_POWER_PRODUCTION    0x0024
#define MB_INPUT_DHW_POWER_CONSUMPTION   0x0025
#define MB_INPUT_COMPRESSOR_FREQ         0x0029
#define MB_INPUT_PUMP_SPEED              0x002A
#define MB_INPUT_PUMP_FLOW               0x002B
#define MB_INPUT_PUMP_DUTY               0x002C
#define MB_INPUT_MAX_PUMP_DUTY           0x002D

// States and modes (0x0030-0x003F)
#define MB_INPUT_HEATPUMP_STATE         0x0030
#define MB_INPUT_PUMP_STATE             0x0031
#define MB_INPUT_OPERATION_MODE         0x0032
#define MB_INPUT_QUIET_MODE             0x0033
#define MB_INPUT_POWERFUL_MODE          0x0034
#define MB_INPUT_HOLIDAY_MODE           0x0035
#define MB_INPUT_FORCE_DHW              0x0036
#define MB_INPUT_FORCE_DEFROST          0x0037
#define MB_INPUT_DEFROSTING_STATE       0x0039
#define MB_INPUT_THREEWAY_VALVE_STATE   0x003A
#define MB_INPUT_ZONES_STATE            0x003B
#define MB_INPUT_MAIN_SCHEDULE_STATE    0x003C
#define MB_INPUT_DHW_HEATER_STATE       0x003D
#define MB_INPUT_ROOM_HEATER_STATE      0x003E
#define MB_INPUT_INTERNAL_HEATER_STATE  0x003F

// Zone pumps (0x0040-0x0046)
#define MB_INPUT_Z1_WATER_PUMP          0x0040
#define MB_INPUT_Z2_WATER_PUMP          0x0041
#define MB_INPUT_POOL_WATER_PUMP        0x0042
#define MB_INPUT_SOLAR_WATER_PUMP       0x0043
#define MB_INPUT_Z1_PUMP_STATE          0x0044
#define MB_INPUT_Z2_PUMP_STATE          0x0045
#define MB_INPUT_PUMP_FLOWRATE_MODE     0x0046

#define MB_REG_INPUT_COUNT              0x0050  // Total input registers

// ============================================================================
// HOLDING REGISTERS (Read/Write) - 0x1000-0x102F
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

// Temperature setpoints (0x1020-0x1028) - all in °C*100
#define MB_HOLDING_SET_Z1_HEAT_TEMP     0x1020
#define MB_HOLDING_SET_Z1_COOL_TEMP     0x1021
#define MB_HOLDING_SET_Z2_HEAT_TEMP     0x1022
#define MB_HOLDING_SET_Z2_COOL_TEMP     0x1023
#define MB_HOLDING_SET_DHW_TEMP         0x1024

#define MB_REG_HOLDING_COUNT            0x0030  // Total holding registers

// ============================================================================
// Register data structures
// ============================================================================

extern uint16_t mb_input_registers[MB_REG_INPUT_COUNT];
extern uint16_t mb_holding_registers[MB_REG_HOLDING_COUNT];

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

