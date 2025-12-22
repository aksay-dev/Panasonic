/**
 * @file wifi_connect.c
 * @brief WiFi connection implementation
 * @version 1.0.0
 * @date 2025
 */

#include "include/wifi_connect.h"
#include "include/project_config.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <stdio.h>
#include "project_config.h"

static const char *TAG = "WIFI_CONNECT";

// WiFi event group bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// WiFi credentials
static wifi_credentials_t wifi_credentials = {0};
static bool wifi_initialized = false;
static bool wifi_connected = false;
static EventGroupHandle_t s_wifi_event_group = NULL;
static esp_netif_t *sta_netif = NULL;

// NVS keys
#define WIFI_NVS_NAMESPACE "wifi"
#define WIFI_NVS_KEY_SSID "ssid"
#define WIFI_NVS_KEY_PASSWORD "password"

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi station started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        wifi_connected = false;
        ESP_LOGI(TAG, "WiFi disconnected, retrying...");
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize WiFi connection module
 */
esp_err_t wifi_connect_init(void) {
    if (wifi_initialized) {
        ESP_LOGW(TAG, "WiFi already initialized");
        return ESP_OK;
    }

    // Initialize NVS (if not already done)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default WiFi station
    sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi station netif");
        return ESP_FAIL;
    }

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Create event group
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_initialized = true;
    ESP_LOGI(TAG, "WiFi initialized");
    return ESP_OK;
}

/**
 * @brief Start WiFi connection
 */
esp_err_t wifi_connect_start(void) {
    if (!wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Load configuration from NVS
    wifi_credentials_t credentials;
    esp_err_t ret = wifi_connect_load_config(&credentials);
    if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "WiFi credentials not found in NVS, using defaults from project_config.h");
        // Use default credentials from project_config.h (can be changed via Modbus later)
        strncpy(credentials.ssid, CONFIG_WIFI_SSID_DEFAULT, sizeof(credentials.ssid) - 1);
        strncpy(credentials.password, CONFIG_WIFI_PASSWORD_DEFAULT, sizeof(credentials.password) - 1);
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Copy to static credentials
    memcpy(&wifi_credentials, &credentials, sizeof(wifi_credentials_t));

    // Configure WiFi using ESP-IDF wifi_config_t
    wifi_config_t wifi_cfg = {0};
    strncpy((char*)wifi_cfg.sta.ssid, wifi_credentials.ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    wifi_cfg.sta.ssid[sizeof(wifi_cfg.sta.ssid) - 1] = '\0';
    strncpy((char*)wifi_cfg.sta.password, wifi_credentials.password, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.password[sizeof(wifi_cfg.sta.password) - 1] = '\0';
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", wifi_credentials.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Wait for connection (timeout 30 seconds)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            pdMS_TO_TICKS(CONFIG_WIFI_RECONNECT));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully");
        wifi_connected = true;
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "WiFi connection failed (timeout)");
        wifi_connected = false;
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief Stop WiFi connection
 */
esp_err_t wifi_connect_stop(void) {
    if (!wifi_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
    }

    wifi_connected = false;
    ESP_LOGI(TAG, "WiFi stopped");
    return ret;
}

/**
 * @brief Check if WiFi is connected
 */
bool wifi_connect_is_connected(void) {
    return wifi_connected;
}

/**
 * @brief Get current IP address
 */
esp_err_t wifi_connect_get_ip(char *ip_str, size_t len) {
    if (ip_str == NULL || len < 16) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!wifi_connected || sta_netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(sta_netif, &ip_info);
    if (ret != ESP_OK) {
        return ret;
    }

    snprintf(ip_str, len, IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
}

/**
 * @brief Set WiFi credentials
 */
esp_err_t wifi_connect_set_credentials(const char *ssid, const char *password) {
    if (ssid == NULL || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(ssid) >= WIFI_SSID_MAX_LEN || strlen(password) >= WIFI_PASSWORD_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_credentials_t credentials;
    strncpy(credentials.ssid, ssid, sizeof(credentials.ssid) - 1);
    credentials.ssid[sizeof(credentials.ssid) - 1] = '\0';
    strncpy(credentials.password, password, sizeof(credentials.password) - 1);
    credentials.password[sizeof(credentials.password) - 1] = '\0';

    // Save to NVS
    esp_err_t ret = wifi_connect_save_config(&credentials);
    if (ret != ESP_OK) {
        return ret;
    }

    // Update current credentials
    memcpy(&wifi_credentials, &credentials, sizeof(wifi_credentials_t));

    ESP_LOGI(TAG, "WiFi credentials updated: SSID=%s", credentials.ssid);
    return ESP_OK;
}

/**
 * @brief Load WiFi credentials from NVS
 */
esp_err_t wifi_connect_load_config(wifi_credentials_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // NVS should already be initialized by wifi_connect_init()
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : err;
    }

    size_t required_size = sizeof(config->ssid);
    err = nvs_get_str(handle, WIFI_NVS_KEY_SSID, config->ssid, &required_size);
    if (err != ESP_OK) {
        nvs_close(handle);
        return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : err;
    }

    required_size = sizeof(config->password);
    err = nvs_get_str(handle, WIFI_NVS_KEY_PASSWORD, config->password, &required_size);
    nvs_close(handle);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    return err;
}

/**
 * @brief Save WiFi credentials to NVS
 */
esp_err_t wifi_connect_save_config(const wifi_credentials_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // NVS should already be initialized by wifi_connect_init()
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", WIFI_NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(handle, WIFI_NVS_KEY_SSID, config->ssid);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    err = nvs_set_str(handle, WIFI_NVS_KEY_PASSWORD, config->password);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save WiFi config to NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "WiFi config saved to NVS");
    }

    return err;
}

