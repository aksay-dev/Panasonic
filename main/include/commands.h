/**
 * @file commands.h
 * @brief Heat pump control commands header
 * @version 0.1.0
 * @date 2025
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct {
//     uint8_t zone1_heat_target_high;
//     uint8_t zone1_heat_target_low;
//     uint8_t zone1_heat_outside_low;
//     uint8_t zone1_heat_outside_high;
//     uint8_t zone2_heat_target_high;
//     uint8_t zone2_heat_target_low;
//     uint8_t zone2_heat_outside_low;
//     uint8_t zone2_heat_outside_high;
//     uint8_t zone1_cool_target_high;
//     uint8_t zone1_cool_target_low;
//     uint8_t zone1_cool_outside_low;
//     uint8_t zone1_cool_outside_high;
//     uint8_t zone2_cool_target_high;
//     uint8_t zone2_cool_target_low;
//     uint8_t zone2_cool_outside_low;
//     uint8_t zone2_cool_outside_high;
// } curves_t;

// Command functions
esp_err_t set_heatpump_state(const bool state);
esp_err_t set_pump(const bool state);
esp_err_t set_max_pump_duty(const uint8_t duty);
esp_err_t set_quiet_mode(const uint8_t mode);
esp_err_t set_z1_heat_request_temperature(const int8_t temperature);
esp_err_t set_z1_cool_request_temperature(const int8_t temperature);
esp_err_t set_z2_heat_request_temperature(const int8_t temperature);
esp_err_t set_z2_cool_request_temperature(const int8_t temperature);
esp_err_t set_force_DHW(const bool state);
esp_err_t set_force_defrost(const bool state);
esp_err_t set_force_sterilization(const bool state);
esp_err_t set_holiday_mode(const bool state);
esp_err_t set_powerful_mode(const uint8_t mode);
esp_err_t set_operation_mode(const uint8_t mode);
esp_err_t set_DHW_temp(const int8_t temperature);
esp_err_t set_curves(const uint8_t *curves);
esp_err_t set_zones(const uint8_t mode);
esp_err_t set_floor_heat_delta(const int8_t temperature);
esp_err_t set_floor_cool_delta(const int8_t temperature);
esp_err_t set_dhw_heat_delta(const int8_t temperature);
esp_err_t set_reset(const bool state);
esp_err_t set_heater_delay_time(const uint8_t time);
esp_err_t set_heater_start_delta(const int8_t temperature);
esp_err_t set_heater_stop_delta(const int8_t temperature);
esp_err_t set_main_schedule(const bool state);
esp_err_t set_alt_external_sensor(const bool state);
esp_err_t set_external_pad_heater(const uint8_t mode);
esp_err_t set_buffer_delta(const int8_t temperature);
esp_err_t set_buffer(const bool state);
esp_err_t set_heating_off_outdoor_temp(const int8_t temperature);
esp_err_t set_bivalent_control(const bool state); 
esp_err_t set_bivalent_mode(const uint8_t mode);
esp_err_t set_bivalent_start_temp(const int8_t temperature);
esp_err_t set_bivalent_ap_start_temp(const int8_t temperature);
esp_err_t set_bivalent_ap_stop_temp(const int8_t temperature);
esp_err_t set_external_control(const bool state);
esp_err_t set_external_error(const bool state);
esp_err_t set_external_compressor_control(const bool state);
esp_err_t set_external_heat_cool_control(const bool state);

// Optional PCB command functions
esp_err_t set_heat_cool_mode(const bool state);
esp_err_t set_compressor_state(const bool state);
esp_err_t set_smart_grid_mode(const uint8_t mode);
esp_err_t set_external_thermostat_1_state(const uint8_t mode);
esp_err_t set_external_thermostat_2_state(const uint8_t mode);
esp_err_t set_demand_control(const uint8_t mode);
esp_err_t set_pool_temp(const float temperature);
esp_err_t set_buffer_temp(const float temperature);
esp_err_t set_z1_room_temp(const float temperature);
esp_err_t set_z1_water_temp(const float temperature);
esp_err_t set_z2_room_temp(const float temperature);
esp_err_t set_z2_water_temp(const float temperature);
esp_err_t set_solar_temp(const float temperature);

// Temperature conversion utility
uint32_t temp2hex(float temp);

#ifdef __cplusplus
}
#endif

#endif // COMMANDS_H
