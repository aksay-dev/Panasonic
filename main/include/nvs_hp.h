#ifndef NVS_HP_H
#define NVS_HP_H

#include "esp_err.h"
#include "modbus_slave.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t modbus_nvs_init(void);
esp_err_t modbus_nvs_load_config(modbus_serial_config_t *cfg);
esp_err_t modbus_nvs_save_config(const modbus_serial_config_t *cfg);

esp_err_t modbus_nvs_load_opt_pcb(uint8_t *value);
esp_err_t modbus_nvs_save_opt_pcb(uint8_t value);

esp_err_t modbus_nvs_load_mqtt_publish(uint8_t *value);
esp_err_t modbus_nvs_save_mqtt_publish(uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // NVS_HP_H

