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
#include <string.h>
#include <stdlib.h>
#include <math.h>

esp_err_t set_heatpump_state(bool state) {
    protocol_cmd_t cmd = {0};
    cmd.type = PROTOCOL_CMD_WRITE;
    cmd.data[0] = PROTOCOL_PKT_WRITE;
    cmd.data[1] = 0x6c;
    cmd.data[2] = 0x01;
    cmd.data[3] = PROTOCOL_DATA_MAIN;
    cmd.data[4] = state ? 2 : 1; // 2: On, 1: Off
    return protocol_send_command(&cmd);
}

esp_err_t set_pump(bool state) {
    protocol_cmd_t cmd = {0};
    cmd.type = PROTOCOL_CMD_WRITE;
    cmd.data_size = sizeof(panasonic_query);
    memcpy(cmd.data, panasonic_query, sizeof(panasonic_query));
    cmd.data[0] = PROTOCOL_PKT_WRITE;
    cmd.data[1] = 0x6c;
    cmd.data[2] = 0x01;
    cmd.data[3] = PROTOCOL_DATA_MAIN;
    cmd.data[4] = state ? 2 : 1; // 2: On, 1: Off
    return protocol_send_command(&cmd);
}
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
