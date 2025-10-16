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

// Command data sizes
// #define COMMANDS_DATA_SIZE 203
// #define COMMANDS_OPT_DATA_SIZE 20

// MQTT topic names
#define MQTT_TOPIC_VALUES "main"
#define MQTT_TOPIC_XVALUES "extra"
#define MQTT_TOPIC_COMMANDS "commands"
#define MQTT_TOPIC_PCBVALUES "optional"
#define MQTT_TOPIC_1WIRE "1wire"
#define MQTT_TOPIC_S0 "s0"
#define MQTT_TOPIC_LOGT "log"
#define MQTT_TOPIC_WILL "LWT"
#define MQTT_TOPIC_IP "ip"
#define MQTT_TOPIC_SEND_RAW "SendRawValue"

// Command function type
typedef uint32_t (*command_func_t)(const char *msg, uint8_t *cmd, char *log_msg);

// Optional command function type
typedef uint32_t (*opt_command_func_t)(const char *msg, char *log_msg);

// Command structure
typedef struct {
    char name[29];
    command_func_t func;
} cmd_struct_t;

// Optional command structure
typedef struct {
    char name[28];
    opt_command_func_t func;
} opt_cmd_struct_t;

// Command functions
esp_err_t set_heatpump_state(bool state);
esp_err_t set_pump(bool state);
uint32_t set_max_pump_duty(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_quiet_mode(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_z1_heat_request_temperature(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_z1_cool_request_temperature(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_z2_heat_request_temperature(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_z2_cool_request_temperature(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_force_DHW(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_force_defrost(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_force_sterilization(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_holiday_mode(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_powerful_mode(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_operation_mode(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_DHW_temp(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_curves(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_zones(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_floor_heat_delta(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_floor_cool_delta(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_dhw_heat_delta(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_reset(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_heater_delay_time(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_heater_start_delta(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_heater_stop_delta(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_main_schedule(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_alt_external_sensor(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_external_pad_heater(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_buffer_delta(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_buffer(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_heatingoffoutdoortemp(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_bivalent_control(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_bivalent_mode(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_bivalent_start_temp(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_bivalent_ap_start_temp(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_bivalent_ap_stop_temp(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_external_control(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_external_error(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_external_compressor_control(const char *msg, uint8_t *cmd, char *log_msg);
uint32_t set_external_heat_cool_control(const char *msg, uint8_t *cmd, char *log_msg);

// Optional PCB command functions
uint32_t set_heat_cool_mode(const char *msg, char *log_msg);
uint32_t set_compressor_state(const char *msg, char *log_msg);
uint32_t set_smart_grid_mode(const char *msg, char *log_msg);
uint32_t set_external_thermostat_1_state(const char *msg, char *log_msg);
uint32_t set_external_thermostat_2_state(const char *msg, char *log_msg);
uint32_t set_demand_control(const char *msg, char *log_msg);
uint32_t set_pool_temp(const char *msg, char *log_msg);
uint32_t set_buffer_temp(const char *msg, char *log_msg);
uint32_t set_z1_room_temp(const char *msg, char *log_msg);
uint32_t set_z1_water_temp(const char *msg, char *log_msg);
uint32_t set_z2_room_temp(const char *msg, char *log_msg);
uint32_t set_z2_water_temp(const char *msg, char *log_msg);
uint32_t set_solar_temp(const char *msg, char *log_msg);
uint32_t set_byte_9(const char *msg, char *log_msg);

// Command arrays
extern const cmd_struct_t commands[];
extern const opt_cmd_struct_t optional_commands[];

// Command processing functions
esp_err_t send_heatpump_command(const char* topic, const char *msg, 
                               esp_err_t (*send_command)(const uint8_t*, size_t), 
                               void (*log_message)(const char*), 
                               bool optional_pcb);

// Optional PCB save/load functions
bool save_optional_pcb(const uint8_t *command, size_t length);
bool load_optional_pcb(uint8_t *command, size_t length);

// Temperature conversion utility
uint32_t temp2hex(float temp);

#ifdef __cplusplus
}
#endif

#endif // COMMANDS_H
