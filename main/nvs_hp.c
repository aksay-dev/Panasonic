#include "include/nvs_hp.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "MODBUS_NVS";

static bool nvs_ready = false;

#define MODBUS_NVS_NAMESPACE       "modbus"
#define MODBUS_NVS_KEY_BAUD        "baud"
#define MODBUS_NVS_KEY_PARITY      "parity"
#define MODBUS_NVS_KEY_STOP_BITS   "stop"
#define MODBUS_NVS_KEY_DATA_BITS   "data"
#define MODBUS_NVS_KEY_SLAVE_ID    "slave"
#define MODBUS_NVS_KEY_OPT_PCB     "opt_pcb"

esp_err_t modbus_nvs_init(void) {
    if (nvs_ready) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS init failed (%s), erasing partition", esp_err_to_name(err));
        esp_err_t erase_ret = nvs_flash_erase();
        if (erase_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS partition: %s", esp_err_to_name(erase_ret));
            return erase_ret;
        }
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise NVS: %s", esp_err_to_name(err));
        return err;
    }

    nvs_ready = true;
    return ESP_OK;
}

esp_err_t modbus_nvs_load_config(modbus_serial_config_t *cfg) {
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = modbus_nvs_init();
    if (err != ESP_OK) {
        return err;
    }

    nvs_handle_t handle;
    err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : err;
    }

    uint32_t baud = 0;
    uint32_t stop_bits = 0;
    uint32_t data_bits = 0;
    uint8_t parity = 0;
    uint8_t slave_id = 0;

    err = nvs_get_u32(handle, MODBUS_NVS_KEY_BAUD, &baud);
    if (err != ESP_OK) goto load_exit;

    err = nvs_get_u8(handle, MODBUS_NVS_KEY_PARITY, &parity);
    if (err != ESP_OK) goto load_exit;

    err = nvs_get_u32(handle, MODBUS_NVS_KEY_STOP_BITS, &stop_bits);
    if (err != ESP_OK) goto load_exit;

    err = nvs_get_u32(handle, MODBUS_NVS_KEY_DATA_BITS, &data_bits);
    if (err != ESP_OK) goto load_exit;

    err = nvs_get_u8(handle, MODBUS_NVS_KEY_SLAVE_ID, &slave_id);
    if (err != ESP_OK) goto load_exit;

    cfg->baudrate = baud;
    cfg->parity = (uart_parity_t)parity;
    cfg->stop_bits = (uart_stop_bits_t)stop_bits;
    cfg->data_bits = (uart_word_length_t)data_bits;
    cfg->slave_addr = slave_id;

load_exit:
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_ERR_NOT_FOUND;
    }
    return err;
}

esp_err_t modbus_nvs_save_config(const modbus_serial_config_t *cfg) {
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = modbus_nvs_init();
    if (err != ESP_OK) {
        return err;
    }

    nvs_handle_t handle;
    err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", MODBUS_NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u32(handle, MODBUS_NVS_KEY_BAUD, cfg->baudrate);
    if (err != ESP_OK) goto save_exit;

    err = nvs_set_u8(handle, MODBUS_NVS_KEY_PARITY, (uint8_t)cfg->parity);
    if (err != ESP_OK) goto save_exit;

    err = nvs_set_u32(handle, MODBUS_NVS_KEY_STOP_BITS, (uint32_t)cfg->stop_bits);
    if (err != ESP_OK) goto save_exit;

    err = nvs_set_u32(handle, MODBUS_NVS_KEY_DATA_BITS, (uint32_t)cfg->data_bits);
    if (err != ESP_OK) goto save_exit;

    err = nvs_set_u8(handle, MODBUS_NVS_KEY_SLAVE_ID, cfg->slave_addr);
    if (err != ESP_OK) goto save_exit;

    err = nvs_commit(handle);

save_exit:
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist Modbus config to NVS: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t modbus_nvs_load_opt_pcb(uint8_t *value) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = modbus_nvs_init();
    if (err != ESP_OK) {
        return err;
    }

    nvs_handle_t handle;
    err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : err;
    }

    err = nvs_get_u8(handle, MODBUS_NVS_KEY_OPT_PCB, value);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_ERR_NOT_FOUND;
    }
    return err;
}

esp_err_t modbus_nvs_save_opt_pcb(uint8_t value) {
    value = value ? 1 : 0;

    esp_err_t err = modbus_nvs_init();
    if (err != ESP_OK) {
        return err;
    }

    nvs_handle_t handle;
    err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", MODBUS_NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(handle, MODBUS_NVS_KEY_OPT_PCB, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist OPT_PCB flag: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

