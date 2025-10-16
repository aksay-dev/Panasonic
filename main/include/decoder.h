/**
 * @file decoder.h
 * @brief Heat pump data decoder header
 * @version 1.0.0
 * @date 2024
 * 
 * Based on HeishaMon decode.h/c implementation
 */

#ifndef DECODER_NEW_H
#define DECODER_NEW_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Main decoded data structure
typedef struct {
    // Basic temperatures main_inlet_temp & main_outlet_temp (stored as int16_t * 100, e.g. 25.5°C = 2550)
    int16_t main_inlet_temp;           // TOP5
    int16_t main_outlet_temp;          // TOP6
    // Basic temperatures (stored as int8_t, e.g. 25°C = 25)
    int8_t main_target_temp;          // TOP7
    int8_t dhw_temp;                  // TOP10
    int8_t dhw_target_temp;           // TOP9
    int8_t outside_temp;              // TOP14
    int8_t room_thermostat_temp;      // TOP33
    int8_t buffer_temp;               // TOP46
    int8_t solar_temp;                // TOP47
    int8_t pool_temp;                 // TOP48
    int8_t main_hex_outlet_temp;      // TOP49
    int8_t discharge_temp;            // TOP50
    int8_t inside_pipe_temp;          // TOP51
    int8_t defrost_temp;              // TOP52
    int8_t eva_outlet_temp;           // TOP53
    int8_t bypass_outlet_temp;        // TOP54
    int8_t ipm_temp;                  // TOP55
    int8_t outside_pipe_temp;         // TOP21
    int8_t z1_temp;                   // TOP56
    int8_t z2_temp;                   // TOP57
    int8_t z1_water_temp;             // TOP36
    int8_t z2_water_temp;             // TOP37
    int8_t z1_water_target_temp;      // TOP42
    int8_t z2_water_target_temp;      // TOP43
    int8_t second_inlet_temp;         // TOP116
    int8_t economizer_outlet_temp;    // TOP117
    int8_t second_room_thermostat_temp; // TOP118

    // Zone temperatures and settings (stored as int8_t, e.g. 25°C = 25)
    int8_t z1_heat_request_temp;      // TOP27
    int8_t z1_cool_request_temp;      // TOP28
    int8_t z2_heat_request_temp;      // TOP34
    int8_t z2_cool_request_temp;      // TOP35
    int8_t z1_heat_curve_target_high_temp;  // TOP29
    int8_t z1_heat_curve_target_low_temp;   // TOP30
    int8_t z1_heat_curve_outside_high_temp; // TOP31
    int8_t z1_heat_curve_outside_low_temp;  // TOP32
    int8_t z1_cool_curve_target_high_temp;  // TOP72
    int8_t z1_cool_curve_target_low_temp;   // TOP73
    int8_t z1_cool_curve_outside_high_temp; // TOP74
    int8_t z1_cool_curve_outside_low_temp;  // TOP75
    int8_t z2_heat_curve_target_high_temp;  // TOP82
    int8_t z2_heat_curve_target_low_temp;   // TOP83
    int8_t z2_heat_curve_outside_high_temp; // TOP84
    int8_t z2_heat_curve_outside_low_temp;  // TOP85
    int8_t z2_cool_curve_target_high_temp;  // TOP86
    int8_t z2_cool_curve_target_low_temp;   // TOP87
    int8_t z2_cool_curve_outside_high_temp; // TOP88
    int8_t z2_cool_curve_outside_low_temp;  // TOP89

    // Power and energy
    uint16_t heat_power_production;  // TOP15
    uint16_t heat_power_consumption; // TOP16
    uint16_t cool_power_production;  // TOP38
    uint16_t cool_power_consumption; // TOP39
    uint16_t dhw_power_production;   // TOP40
    uint16_t dhw_power_consumption;  // TOP41

    // Operation states
    uint8_t heatpump_state;          // TOP0
    uint8_t force_dhw_state;         // TOP2
    uint8_t operating_mode_state;    // TOP4
    uint8_t quiet_mode_schedule;     // TOP3
    uint8_t powerful_mode_time;      // TOP17
    uint8_t quiet_mode_level;        // TOP18
    uint8_t holiday_mode_state;      // TOP19
    uint8_t three_way_valve_state;   // TOP20
    uint8_t defrosting_state;        // TOP26
    uint8_t main_schedule_state;     // TOP13
    uint8_t zones_state;             // TOP94

    // Technical parameters
    uint16_t compressor_freq;        // TOP8
    int16_t pump_flow;               // TOP1 (л/мин * 100)
    uint16_t operations_hours;       // TOP11
    uint16_t operations_counter;     // TOP12
    uint16_t fan1_motor_speed;       // TOP62
    uint16_t fan2_motor_speed;       // TOP63
    int16_t high_pressure;           // TOP64 (бар * 100)
    uint16_t pump_speed;             // TOP65
    int16_t low_pressure;            // TOP66 (бар * 100)
    int16_t compressor_current;      // TOP67 (А * 100)
    uint8_t pump_duty;               // TOP93
    uint8_t max_pump_duty;           // TOP95

    // Heaters and states
    uint8_t dhw_heater_state;        // TOP58
    uint8_t room_heater_state;       // TOP59
    uint8_t internal_heater_state;   // TOP60
    uint8_t external_heater_state;   // TOP61
    uint8_t force_heater_state;      // TOP68
    uint8_t sterilization_state;     // TOP69
    int16_t sterilization_temp;      // TOP70 (°C * 100)
    uint8_t sterilization_max_time;  // TOP71

    // Deltas and shifts (stored as int16_t * 100)
    int16_t dhw_heat_delta;          // TOP22
    int16_t heat_delta;              // TOP23
    int16_t cool_delta;              // TOP24
    int16_t dhw_holiday_shift_temp;  // TOP25
    int16_t room_holiday_shift_temp; // TOP45
    int16_t buffer_tank_delta;       // TOP113

    // Mode settings
    uint8_t heating_mode;            // TOP76
    int16_t heating_off_outdoor_temp; // TOP77 (°C * 100)
    int16_t heater_on_outdoor_temp;  // TOP78 (°C * 100)
    int16_t heat_to_cool_temp;       // TOP79 (°C * 100)
    int16_t cool_to_heat_temp;       // TOP80 (°C * 100)
    uint8_t cooling_mode;            // TOP81

    // Solar and buffer settings
    uint8_t buffer_installed;        // TOP99
    uint8_t dhw_installed;           // TOP100
    uint8_t solar_mode;              // TOP101
    int16_t solar_on_delta;          // TOP102 (°C * 100)
    int16_t solar_off_delta;         // TOP103 (°C * 100)
    int16_t solar_frost_protection;  // TOP104 (°C * 100)
    int16_t solar_high_limit;        // TOP105 (°C * 100)

    // Pump and liquid settings
    uint8_t pump_flowrate_mode;      // TOP106
    uint8_t liquid_type;             // TOP107
    uint8_t alt_external_sensor;     // TOP108
    uint8_t anti_freeze_mode;        // TOP109
    uint8_t optional_pcb;            // TOP110

    // Zone sensor settings
    uint8_t z1_sensor_settings;      // TOP111
    uint8_t z2_sensor_settings;      // TOP112

    // External controls
    uint8_t external_pad_heater;     // TOP114
    int16_t water_pressure;          // TOP115 (бар * 100)
    uint8_t external_control;        // TOP119
    uint8_t external_heat_cool_control; // TOP120
    uint8_t external_error_signal;   // TOP121
    uint8_t external_compressor_control; // TOP122

    // Pump states
    uint8_t z2_pump_state;           // TOP123
    uint8_t z1_pump_state;           // TOP124
    uint8_t two_way_valve_state;     // TOP125
    uint8_t three_way_valve_state2;  // TOP126

    // Valve PID settings (stored as int16_t * 100)
    int16_t z1_valve_pid;            // TOP127
    int16_t z2_valve_pid;            // TOP128

    // Bivalent settings
    uint8_t bivalent_control;        // TOP129
    uint8_t bivalent_mode;           // TOP130
    int16_t bivalent_start_temp;     // TOP131 (°C * 100)
    uint8_t bivalent_advanced_heat;  // TOP132
    uint8_t bivalent_advanced_dhw;   // TOP133
    int16_t bivalent_advanced_start_temp; // TOP134 (°C * 100)
    int16_t bivalent_advanced_stop_temp;  // TOP135 (°C * 100)
    uint8_t bivalent_advanced_start_delay; // TOP136
    uint8_t bivalent_advanced_stop_delay;  // TOP137
    uint8_t bivalent_advanced_dhw_delay;   // TOP138

    // Timing settings
    uint8_t heater_delay_time;       // TOP96
    int16_t heater_start_delta;      // TOP97 (°C * 100)
    int16_t heater_stop_delta;       // TOP98 (°C * 100)

    // Error and model info
    char error_state[16];            // TOP44
    char heat_pump_model[32];        // TOP92

    // Operation hours
    uint16_t room_heater_operations_hours; // TOP90
    uint16_t dhw_heater_operations_hours;  // TOP91

    // Extra data (XTOP)
    uint16_t heat_power_consumption_extra; // XTOP0
    uint16_t cool_power_consumption_extra; // XTOP1
    uint16_t dhw_power_consumption_extra;  // XTOP2
    uint16_t heat_power_production_extra;  // XTOP3
    uint16_t cool_power_production_extra;  // XTOP4
    uint16_t dhw_power_production_extra;   // XTOP5

    // Optional PCB data (OPT)
    uint8_t z1_water_pump;           // OPT0
    uint8_t z1_mixing_valve;         // OPT1
    uint8_t z2_water_pump;           // OPT2
    uint8_t z2_mixing_valve;         // OPT3
    uint8_t pool_water_pump;         // OPT4
    uint8_t solar_water_pump;        // OPT5
    uint8_t alarm_state;             // OPT6

    bool data_valid;                 // Indicates if data is valid
    uint32_t last_update_time;       // Last update timestamp
} hp_decoder_data_t;

// Global decoded data structure
extern hp_decoder_data_t g_decoded_data;

/**
 * @brief Initialize the decoder module
 * @return ESP_OK on success
 */
esp_err_t decoder_init(void);

/**
 * @brief Decode main heat pump data
 * @return ESP_OK on success
 */
esp_err_t decode_main_data(void);

/**
 * @brief Decode extra heat pump data
 * @return ESP_OK on success
 */
esp_err_t decode_extra_data(void);

/**
 * @brief Decode optional PCB data
 * @return ESP_OK on success
 */
esp_err_t decode_opt_data(void);

void log_main_data(void);
void log_extra_data(void);
void log_opt_data(void);

#ifdef __cplusplus
}
#endif

#endif // DECODER_NEW_H
