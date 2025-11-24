/**
 * @file modbus_params.c
 * @brief Modbus register mapping and data synchronization - FULL decoder data
 * @version 2.0.0
 * @date 2025
 */

#include "include/modbus_params.h"
#include "include/commands.h"
#include "include/modbus_slave.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "MODBUS_PARAMS";

// Register arrays
// Note: Modbus registers are 16-bit values. They can be interpreted as signed or unsigned
// by the Modbus client. We use int16_t for values that can be negative (temperatures, etc.)
int16_t mb_input_registers[MB_REG_INPUT_COUNT] = {0};
int16_t mb_holding_registers[MB_REG_HOLDING_COUNT] = {0};

#define HOLDING_INDEX(reg)  ((reg) - MB_REG_HOLDING_START)

enum {
    IDX_MODBUS_BAUD = HOLDING_INDEX(MB_HOLDING_SET_MODBUS_BAUD),
    IDX_MODBUS_PARITY = HOLDING_INDEX(MB_HOLDING_SET_MODBUS_PARITY),
    IDX_MODBUS_STOP_BITS = HOLDING_INDEX(MB_HOLDING_SET_MODBUS_STOP_BITS),
    IDX_MODBUS_DATA_BITS = HOLDING_INDEX(MB_HOLDING_SET_MODBUS_DATA_BITS),
    IDX_MODBUS_SLAVE_ID = HOLDING_INDEX(MB_HOLDING_SET_MODBUS_SLAVE_ID)
};

static bool modbus_is_supported_baud(uint32_t baud);
static bool modbus_decode_parity(uint16_t code, uart_parity_t *parity);
static int16_t modbus_encode_parity(uart_parity_t parity);
static bool modbus_decode_stop_bits(uint16_t code, uart_stop_bits_t *stop_bits);
static int16_t modbus_encode_stop_bits(uart_stop_bits_t stop_bits);
static bool modbus_decode_data_bits(uint16_t code, uart_word_length_t *data_bits);
static int16_t modbus_encode_data_bits(uart_word_length_t data_bits);
static bool modbus_is_valid_slave_id(uint32_t value);
static void modbus_restore_serial_field(uint16_t reg_addr);
static void modbus_sync_serial_registers(void);
static esp_err_t modbus_build_serial_config_from_registers(modbus_serial_config_t *cfg);

static bool modbus_is_supported_baud(uint32_t baud) {
    return (baud >= 1200U && baud <= 57600U);
}

static bool modbus_decode_parity(uint16_t code, uart_parity_t *parity) {
    if (parity == NULL) {
        return false;
    }
    switch (code) {
        case 0:
            *parity = UART_PARITY_DISABLE;
            return true;
        case 1:
            *parity = UART_PARITY_EVEN;
            return true;
        case 2:
            *parity = UART_PARITY_ODD;
            return true;
        default:
            return false;
    }
}

static int16_t modbus_encode_parity(uart_parity_t parity) {
    switch (parity) {
        case UART_PARITY_EVEN:
            return 1;
        case UART_PARITY_ODD:
            return 2;
        case UART_PARITY_DISABLE:
        default:
            return 0;
    }
}

static bool modbus_decode_stop_bits(uint16_t code, uart_stop_bits_t *stop_bits) {
    if (stop_bits == NULL) {
        return false;
    }
    switch (code) {
        case 1:
            *stop_bits = UART_STOP_BITS_1;
            return true;
        case 2:
            *stop_bits = UART_STOP_BITS_2;
            return true;
        default:
            return false;
    }
}

static int16_t modbus_encode_stop_bits(uart_stop_bits_t stop_bits) {
    return (stop_bits == UART_STOP_BITS_2) ? 2 : 1;
}

static bool modbus_decode_data_bits(uint16_t code, uart_word_length_t *data_bits) {
    if (data_bits == NULL) {
        return false;
    }
    switch (code) {
        case 7:
            *data_bits = UART_DATA_7_BITS;
            return true;
        case 8:
            *data_bits = UART_DATA_8_BITS;
            return true;
        default:
            return false;
    }
}

static int16_t modbus_encode_data_bits(uart_word_length_t data_bits) {
    return (data_bits == UART_DATA_7_BITS) ? 7 : 8;
}

static bool modbus_is_valid_slave_id(uint32_t value) {
    return (value >= 1U && value <= 247U);
}

static void modbus_restore_serial_field(uint16_t reg_addr) {
    modbus_serial_config_t cfg;
    if (modbus_slave_get_serial_config(&cfg) != ESP_OK) {
        return;
    }

    switch (reg_addr) {
        case MB_HOLDING_SET_MODBUS_BAUD:
            mb_holding_registers[IDX_MODBUS_BAUD] = (int16_t)(cfg.baudrate & 0xFFFF);
            break;
        case MB_HOLDING_SET_MODBUS_PARITY:
            mb_holding_registers[IDX_MODBUS_PARITY] = modbus_encode_parity(cfg.parity);
            break;
        case MB_HOLDING_SET_MODBUS_STOP_BITS:
            mb_holding_registers[IDX_MODBUS_STOP_BITS] = modbus_encode_stop_bits(cfg.stop_bits);
            break;
        case MB_HOLDING_SET_MODBUS_DATA_BITS:
            mb_holding_registers[IDX_MODBUS_DATA_BITS] = modbus_encode_data_bits(cfg.data_bits);
            break;
        case MB_HOLDING_SET_MODBUS_SLAVE_ID:
            mb_holding_registers[IDX_MODBUS_SLAVE_ID] = (int16_t)cfg.slave_addr;
            break;
        default:
            break;
    }
}

static void modbus_sync_serial_registers(void) {
    modbus_serial_config_t cfg;
    if (modbus_slave_get_serial_config(&cfg) != ESP_OK) {
        return;
    }

    mb_holding_registers[IDX_MODBUS_BAUD] = (int16_t)(cfg.baudrate & 0xFFFF);
    mb_holding_registers[IDX_MODBUS_PARITY] = modbus_encode_parity(cfg.parity);
    mb_holding_registers[IDX_MODBUS_STOP_BITS] = modbus_encode_stop_bits(cfg.stop_bits);
    mb_holding_registers[IDX_MODBUS_DATA_BITS] = modbus_encode_data_bits(cfg.data_bits);
    mb_holding_registers[IDX_MODBUS_SLAVE_ID] = (int16_t)cfg.slave_addr;
}

static esp_err_t modbus_build_serial_config_from_registers(modbus_serial_config_t *cfg) {
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t baud = (uint16_t)mb_holding_registers[IDX_MODBUS_BAUD];
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_word_length_t data_bits;
    uint32_t slave_id = (uint16_t)mb_holding_registers[IDX_MODBUS_SLAVE_ID];

    if (!modbus_is_supported_baud(baud)) {
        ESP_LOGW(TAG, "Requested baud rate %lu not supported (1200-57600)",
                 (unsigned long)baud);
        return ESP_ERR_INVALID_ARG;
    }
    if (!modbus_decode_parity((uint16_t)mb_holding_registers[IDX_MODBUS_PARITY], &parity)) {
        ESP_LOGW(TAG, "Requested parity code %d is invalid",
                 mb_holding_registers[IDX_MODBUS_PARITY]);
        return ESP_ERR_INVALID_ARG;
    }
    if (!modbus_decode_stop_bits((uint16_t)mb_holding_registers[IDX_MODBUS_STOP_BITS], &stop_bits)) {
        ESP_LOGW(TAG, "Requested stop bits code %d is invalid",
                 mb_holding_registers[IDX_MODBUS_STOP_BITS]);
        return ESP_ERR_INVALID_ARG;
    }
    if (!modbus_decode_data_bits((uint16_t)mb_holding_registers[IDX_MODBUS_DATA_BITS], &data_bits)) {
        ESP_LOGW(TAG, "Requested data bits code %d is invalid",
                 mb_holding_registers[IDX_MODBUS_DATA_BITS]);
        return ESP_ERR_INVALID_ARG;
    }
    if (!modbus_is_valid_slave_id(slave_id)) {
        ESP_LOGW(TAG, "Requested Modbus slave id %lu invalid (1-247)",
                 (unsigned long)slave_id);
        return ESP_ERR_INVALID_ARG;
    }

    cfg->baudrate = baud;
    cfg->parity = parity;
    cfg->stop_bits = stop_bits;
    cfg->data_bits = data_bits;
    cfg->slave_addr = (uint8_t)slave_id;
    return ESP_OK;
}

/**
 * @brief Initialize Modbus parameter structures
 * @return ESP_OK on success
 */
esp_err_t modbus_params_init(void) {
    // Clear all registers
    memset(mb_input_registers, 0, sizeof(mb_input_registers));
    memset(mb_holding_registers, 0, sizeof(mb_holding_registers));

    modbus_sync_serial_registers();
    
    ESP_LOGI(TAG, "Modbus parameters initialized: %d input, %d holding registers",
             MB_REG_INPUT_COUNT, MB_REG_HOLDING_COUNT);
    
    return ESP_OK;
}

/**
 * @brief Process holding register writes (execute commands)
 * @param reg_addr Register address that was written
 * @return ESP_OK on success
 */
esp_err_t modbus_params_process_holding_write(uint16_t reg_addr) {
    esp_err_t ret = ESP_OK;
    int16_t value = mb_holding_registers[reg_addr - MB_REG_HOLDING_START];
    
    ESP_LOGI(TAG, "Processing write to register 0x%04X, value: %d", reg_addr, value);

    switch (reg_addr) {
        // Control commands
        case MB_HOLDING_SET_HEATPUMP:
            ret = set_heatpump_state(value != 0);
            break;
            
        case MB_HOLDING_SET_PUMP:
            ret = set_pump(value != 0);
            break;
            
        case MB_HOLDING_SET_MAX_PUMP_DUTY:
            if (value <= 100) {
                ret = set_max_pump_duty((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid max pump duty: %u (must be 0-100)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case MB_HOLDING_SET_QUIET_MODE:
            if (value <= 3) {
                ret = set_quiet_mode((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid quiet mode: %u (must be 0-3)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case MB_HOLDING_SET_POWERFUL_MODE:
            if (value <= 90) {
                ret = set_powerful_mode((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid powerful mode: %u (must be 0-90 minutes)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case MB_HOLDING_SET_OPERATION_MODE:
            if (value <= 6) {
                ret = set_operation_mode((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid operation mode: %u (must be 0-6)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
            
        case MB_HOLDING_SET_HOLIDAY_MODE:
            ret = set_holiday_mode(value != 0);
            break;
            
        case MB_HOLDING_SET_FORCE_DHW:
            ret = set_force_DHW(value != 0);
            break;
            
        case MB_HOLDING_SET_FORCE_DEFROST:
            ret = set_force_defrost(value != 0);
            break;
            
        case MB_HOLDING_SET_FORCE_STERILIZATION:
            ret = set_force_sterilization(value != 0);
            break;
            
        case MB_HOLDING_SET_MAIN_SCHEDULE:
            ret = set_main_schedule(value != 0);
            break;
            
        case MB_HOLDING_SET_RESET:
            ret = set_reset(value != 0);
            break;
            
        case MB_HOLDING_SET_ZONES:
            if (value <= 2) {
                ret = set_zones((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid zones: %u (must be 0-2)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;

        // External control block (0x100D-0x100F)
        case MB_HOLDING_SET_EXTERNAL_CONTROL:
            ret = set_external_control(value != 0);
            break;
        case MB_HOLDING_SET_EXTERNAL_ERROR:
            ret = set_external_error(value != 0);
            break;
        case MB_HOLDING_SET_EXTERNAL_COMPRESSOR_CONTROL:
            ret = set_external_compressor_control(value != 0);
            break;

        // Additional controls (0x1010-0x1015)
        case MB_HOLDING_SET_EXTERNAL_HEAT_COOL_CONTROL:
            ret = set_external_heat_cool_control(value != 0);
            break;
        case MB_HOLDING_SET_BIVALENT_CONTROL:
            ret = set_bivalent_control(value != 0);
            break;
        case MB_HOLDING_SET_BIVALENT_MODE:
            if (value <= 2) {
                ret = set_bivalent_mode((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid bivalent mode: %d (0-2)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        case MB_HOLDING_SET_ALT_EXTERNAL_SENSOR:
            ret = set_alt_external_sensor(value != 0);
            break;
        case MB_HOLDING_SET_EXTERNAL_PAD_HEATER:
            if (value <= 2) {
                ret = set_external_pad_heater((uint8_t)value);
            } else {
                ESP_LOGW(TAG, "Invalid pad heater: %d (0-2)", value);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        case MB_HOLDING_SET_BUFFER:
            ret = set_buffer(value != 0);
            break;

        // Temperature setpoints (value is raw decoded data, direct pass)
        case MB_HOLDING_SET_Z1_HEAT_TEMP:
            ret = set_z1_heat_request_temperature((int8_t)value);
            break;
            
        case MB_HOLDING_SET_Z1_COOL_TEMP:
            ret = set_z1_cool_request_temperature((int8_t)value);
            break;
            
        case MB_HOLDING_SET_Z2_HEAT_TEMP:
            ret = set_z2_heat_request_temperature((int8_t)value);
            break;
            
        case MB_HOLDING_SET_Z2_COOL_TEMP:
            ret = set_z2_cool_request_temperature((int8_t)value);
            break;
            
        case MB_HOLDING_SET_DHW_TEMP:
            ret = set_DHW_temp((int8_t)value);
            break;

        // Optional temperatures (we interpret holding int16 as whole °C and pass as float)
        case MB_HOLDING_SET_POOL_TEMP:
            ret = set_pool_temp((float)value);
            break;
        case MB_HOLDING_SET_BUFFER_TEMP:
            ret = set_buffer_temp((float)value);
            break;
        case MB_HOLDING_SET_Z1_ROOM_TEMP:
            ret = set_z1_room_temp((float)value);
            break;
        case MB_HOLDING_SET_Z1_WATER_TEMP:
            ret = set_z1_water_temp((float)value);
            break;
        case MB_HOLDING_SET_Z2_ROOM_TEMP:
            ret = set_z2_room_temp((float)value);
            break;
        case MB_HOLDING_SET_Z2_WATER_TEMP:
            ret = set_z2_water_temp((float)value);
            break;
        case MB_HOLDING_SET_SOLAR_TEMP:
            ret = set_solar_temp((float)value);
            break;

        // Optional controls
        case MB_HOLDING_SET_HEAT_COOL_MODE:
            ret = set_heat_cool_mode(value != 0);
            break;
        case MB_HOLDING_SET_COMPRESSOR_STATE:
            ret = set_compressor_state(value != 0);
            break;
        case MB_HOLDING_SET_SMART_GRID_MODE:
            ret = set_smart_grid_mode((uint8_t)value);
            break;
        case MB_HOLDING_SET_EXT_THERMOSTAT_1:
            ret = set_external_thermostat_1_state((uint8_t)value);
            break;
        case MB_HOLDING_SET_EXT_THERMOSTAT_2:
            ret = set_external_thermostat_2_state((uint8_t)value);
            break;
        case MB_HOLDING_SET_DEMAND_CONTROL:
            ret = set_demand_control((uint8_t)value);
            break;

        // Curves apply: read block 0x1060.. and invoke set_curves
        case MB_HOLDING_CURVES_APPLY: {
            uint8_t curves_bytes[MB_HOLDING_CURVES_REGS * 2];
            for (int i = 0; i < MB_HOLDING_CURVES_REGS; ++i) {
                int16_t reg = mb_holding_registers[(MB_HOLDING_CURVES_START - MB_REG_HOLDING_START) + i];
                curves_bytes[i * 2] = (uint8_t)((reg >> 8) & 0xFF);
                curves_bytes[i * 2 + 1] = (uint8_t)(reg & 0xFF);
            }
            ret = set_curves(curves_bytes);
            break;
        }

        // Deltas and timing (0x1030-0x1036) - int8 degrees or minutes
        case MB_HOLDING_SET_BUFFER_DELTA:
            ret = set_buffer_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_FLOOR_HEAT_DELTA:
            ret = set_floor_heat_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_FLOOR_COOL_DELTA:
            ret = set_floor_cool_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_DHW_HEAT_DELTA:
            ret = set_dhw_heat_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_HEATER_START_DELTA:
            ret = set_heater_start_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_HEATER_STOP_DELTA:
            ret = set_heater_stop_delta((int8_t)value);
            break;
        case MB_HOLDING_SET_HEATER_DELAY_TIME:
            ret = set_heater_delay_time((uint8_t)value);
            break;

        // Bivalent temperatures (commands accept int8 setpoints)
        case MB_HOLDING_SET_BIVALENT_START_TEMP:
            ret = set_bivalent_start_temp((int8_t)value);
            break;
        case MB_HOLDING_SET_BIVALENT_AP_START_TEMP:
            ret = set_bivalent_ap_start_temp((int8_t)value);
            break;
        case MB_HOLDING_SET_BIVALENT_AP_STOP_TEMP:
            ret = set_bivalent_ap_stop_temp((int8_t)value);
            break;

        // Modbus serial configuration commands
        case MB_HOLDING_SET_MODBUS_BAUD: {
            uint32_t baud = (uint16_t)mb_holding_registers[IDX_MODBUS_BAUD];
            if (!modbus_is_supported_baud(baud)) {
                ESP_LOGW(TAG, "Invalid Modbus baud rate request: %lu (1200-57600)",
                         (unsigned long)baud);
                modbus_restore_serial_field(MB_HOLDING_SET_MODBUS_BAUD);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case MB_HOLDING_SET_MODBUS_PARITY: {
            uart_parity_t tmp;
            if (!modbus_decode_parity((uint16_t)mb_holding_registers[IDX_MODBUS_PARITY], &tmp)) {
                ESP_LOGW(TAG, "Invalid Modbus parity code: %d",
                         mb_holding_registers[IDX_MODBUS_PARITY]);
                modbus_restore_serial_field(MB_HOLDING_SET_MODBUS_PARITY);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case MB_HOLDING_SET_MODBUS_STOP_BITS: {
            uart_stop_bits_t tmp;
            if (!modbus_decode_stop_bits((uint16_t)mb_holding_registers[IDX_MODBUS_STOP_BITS], &tmp)) {
                ESP_LOGW(TAG, "Invalid Modbus stop bits code: %d",
                         mb_holding_registers[IDX_MODBUS_STOP_BITS]);
                modbus_restore_serial_field(MB_HOLDING_SET_MODBUS_STOP_BITS);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case MB_HOLDING_SET_MODBUS_DATA_BITS: {
            uart_word_length_t tmp;
            if (!modbus_decode_data_bits((uint16_t)mb_holding_registers[IDX_MODBUS_DATA_BITS], &tmp)) {
                ESP_LOGW(TAG, "Invalid Modbus data bits code: %d",
                         mb_holding_registers[IDX_MODBUS_DATA_BITS]);
                modbus_restore_serial_field(MB_HOLDING_SET_MODBUS_DATA_BITS);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case MB_HOLDING_SET_MODBUS_SLAVE_ID: {
            uint32_t slave_id = (uint16_t)mb_holding_registers[IDX_MODBUS_SLAVE_ID];
            if (!modbus_is_valid_slave_id(slave_id)) {
                ESP_LOGW(TAG, "Invalid Modbus slave id: %lu (1-247)", (unsigned long)slave_id);
                modbus_restore_serial_field(MB_HOLDING_SET_MODBUS_SLAVE_ID);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }

        case MB_HOLDING_APPLY_MODBUS_SETTINGS: {
            if (value == 0) {
                ESP_LOGI(TAG, "Modbus settings apply requested with 0, ignoring");
                break;
            }
            modbus_serial_config_t new_cfg;
            esp_err_t build_ret = modbus_build_serial_config_from_registers(&new_cfg);
            if (build_ret != ESP_OK) {
                ret = build_ret;
                break;
            }
            ret = modbus_slave_apply_serial_config(&new_cfg);
            if (ret == ESP_OK) {
                modbus_sync_serial_registers();
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "Write to unhandled register: 0x%04X", reg_addr);
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Command executed successfully for register 0x%04X", reg_addr);
    } else {
        ESP_LOGE(TAG, "Command failed for register 0x%04X: %s", reg_addr, esp_err_to_name(ret));
    }

    return ret;
}
