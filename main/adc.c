/**
 * @file adc.c
 * @brief ADC analog input implementation with moving average filter
 * @version 1.0.0
 * @date 2025
 */

#include "include/adc.h"
#include "include/modbus_params.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/adc.h"
#include "hal/adc_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <limits.h>

static const char *TAG = "ADC";

// Moving average filter structure
typedef struct {
    uint16_t samples[ADC_FILTER_SIZE];  // Circular buffer for samples
    uint16_t sum;                        // Sum of all samples
    uint8_t index;                       // Current index in circular buffer
    uint8_t count;                       // Number of samples collected (0 to FILTER_SIZE)
    bool initialized;                    // True when filter has enough samples
} adc_filter_t;

// ADC channel configuration
typedef struct {
    adc1_channel_t channel;  // ADC1 channel number
    uint8_t gpio;             // GPIO pin number
    adc_filter_t filter;      // Moving average filter
    uint16_t raw_value;       // Last raw reading
    uint16_t filtered_value;  // Last filtered value
} adc_channel_t;

// ADC channels configuration
static adc_channel_t adc_channels[ADC_CHANNEL_COUNT] = {
    {.channel = ADC1_CHANNEL_4, .gpio = ADC_CH0_GPIO},  // GPIO32 -> ADC1_CH4
    {.channel = ADC1_CHANNEL_6, .gpio = ADC_CH1_GPIO},  // GPIO34 -> ADC1_CH6
    {.channel = ADC1_CHANNEL_7, .gpio = ADC_CH2_GPIO}, // GPIO35 -> ADC1_CH7
};

// Task handle
static TaskHandle_t adc_task_handle = NULL;
static bool adc_initialized = false;

// Modbus register addresses for ADC channels
static const uint16_t adc_modbus_registers[ADC_CHANNEL_COUNT] = {
    MB_INPUT_ADC_AIN,
    MB_INPUT_ADC_NTC1,
    MB_INPUT_ADC_NTC2
};

/**
 * @brief Initialize moving average filter
 */
static void adc_filter_init(adc_filter_t *filter) {
    memset(filter->samples, 0, sizeof(filter->samples));
    filter->sum = 0;
    filter->index = 0;
    filter->count = 0;
    filter->initialized = false;
}

/**
 * @brief Add a new sample to the moving average filter
 * @param filter Pointer to filter structure
 * @param sample New sample value
 * @return Filtered value (average)
 */
static uint16_t adc_filter_add_sample(adc_filter_t *filter, uint16_t sample) {
    if (filter->count < ADC_FILTER_SIZE) {
        // Still filling the buffer
        filter->samples[filter->count] = sample;
        filter->sum += sample;
        filter->count++;
        
        // Return average of collected samples
        return filter->sum / filter->count;
    } else {
        // Buffer is full, use circular buffer
        // Remove oldest sample from sum
        filter->sum -= filter->samples[filter->index];
        
        // Add new sample
        filter->samples[filter->index] = sample;
        filter->sum += sample;
        
        // Move to next position in circular buffer
        filter->index = (filter->index + 1) % ADC_FILTER_SIZE;
        filter->initialized = true;
        
        // Return average
        return filter->sum / ADC_FILTER_SIZE;
    }
}

/**
 * @brief Read ADC value for a channel
 * @param channel_idx Channel index (0-2)
 * @return Raw ADC value (0-4095) or 0 on error
 */
static uint16_t adc_read_channel(uint8_t channel_idx) {
    if (channel_idx >= ADC_CHANNEL_COUNT) {
        ESP_LOGE(TAG, "Invalid channel index: %d", channel_idx);
        return 0;
    }
    
    adc_channel_t *ch = &adc_channels[channel_idx];
    
    int adc_value = adc1_get_raw(ch->channel);
    if (adc_value < 0) {
        ESP_LOGE(TAG, "Failed to read ADC channel %d (GPIO%d)", channel_idx, ch->gpio);
        return 0;
    }
    
    // ADC1 returns 0-4095 for 12-bit resolution
    return (uint16_t)adc_value;
}

/**
 * @brief Convert ADC value to NTC temperature using Beta equation
 * @param adc_value Filtered ADC value (0-4095)
 * @return Temperature in degrees Celsius * 100 (e.g., 2550 for 25.5°C)
 *         Returns INT16_MIN on error
 */
/**
 * @brief Convert ADC value to NTC temperature using Beta equation
 * @param adc_value Filtered ADC value (0-4095)
 * @return Temperature in degrees Celsius * 100 (e.g., 2550 for 25.5°C)
 *         Returns INT16_MIN on error (open circuit), INT16_MAX on very high temperature
 */
static int16_t adc_ntc_to_temperature(uint16_t adc_value) {
    // Handle edge cases
    if (adc_value == 0) {
        // Open circuit or disconnected NTC
        return INT16_MIN;
    }
    
    if (adc_value >= ADC_ADC_MAX_VALUE) {
        // Short circuit or very high temperature (NTC resistance very low)
        return INT16_MAX;
    }
    
    // Calculate voltage at ADC input
    // V_adc = (adc_value / ADC_MAX_VALUE) * V_ref
    float v_adc_mv = ((float)adc_value / (float)ADC_ADC_MAX_VALUE) * (float)ADC_ADC_REFERENCE_VOLTAGE_MV;
    
    // Calculate NTC resistance using voltage divider formula
    // V_adc = V_ref * R_ntc / (R_top + R_ntc)
    // Solving for R_ntc: R_ntc = R_top * V_adc / (V_ref - V_adc)
    float v_ref_mv = (float)ADC_ADC_REFERENCE_VOLTAGE_MV;
    float r_top_ohm = (float)ADC_NTC_VOLTAGE_DIVIDER_TOP_OHM;
    
    // Check for division by zero or very high voltage
    float v_diff = v_ref_mv - v_adc_mv;
    if (v_diff < 0.1f) {
        // Voltage too high, NTC resistance too low (short circuit or very high temperature)
        return INT16_MAX;
    }
    
    float r_ntc_ohm = r_top_ohm * v_adc_mv / v_diff;
    
    // Check for invalid resistance values
    if (r_ntc_ohm <= 0.0f || r_ntc_ohm > 1000000.0f) {
        // Invalid resistance (out of reasonable range)
        return INT16_MIN;
    }
    
    // Use Beta equation to calculate temperature
    // R = R0 * exp(Beta * (1/T - 1/T0))
    // Solving for T: T = 1 / (1/T0 + (1/Beta) * ln(R/R0))
    float r0_ohm = (float)ADC_NTC_R0_OHM;
    float beta = (float)ADC_NTC_BETA_COEFFICIENT;
    float t0_k = ADC_NTC_T0_KELVIN;
    
    // Calculate ln(R/R0)
    float r_ratio = r_ntc_ohm / r0_ohm;
    if (r_ratio <= 0.0f) {
        return INT16_MIN;
    }
    
    float ln_r_ratio = logf(r_ratio);
    
    // Calculate temperature in Kelvin
    float t_k = 1.0f / (1.0f / t0_k + (1.0f / beta) * ln_r_ratio);
    
    // Check for invalid temperature (should be between -50°C and 150°C for typical NTC)
    if (t_k < 223.15f || t_k > 423.15f) { // -50°C to 150°C
        return INT16_MIN;
    }
    
    // Convert to Celsius
    float t_celsius = t_k - 273.15f;
    
    // Return as int16_t * 100 (same format as other temperature registers)
    // Clamp to valid int16_t range
    int16_t temp_x100 = (int16_t)(t_celsius * 100.0f);
    
    return temp_x100;
}

/**
 * @brief ADC reading task
 * Reads ADC values, applies filter, and updates Modbus registers
 */
static void adc_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(ADC_UPDATE_INTERVAL_MS);
    
    while (1) {
        // Read and filter all channels
        for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
            // Read raw ADC value
            uint16_t raw = adc_read_channel(i);
            adc_channels[i].raw_value = raw;
            
            // Apply moving average filter
            uint16_t filtered = adc_filter_add_sample(&adc_channels[i].filter, raw);
            adc_channels[i].filtered_value = filtered;
            
            // Update Modbus register
            uint16_t reg_addr = adc_modbus_registers[i];
            if (reg_addr >= MB_REG_INPUT_START && reg_addr < MB_REG_INPUT_START + MB_REG_INPUT_COUNT) {
                uint16_t reg_index = reg_addr - MB_REG_INPUT_START;
                
                // For NTC channels (NTC1 and NTC2), convert to temperature
                // For AIN channel, store raw ADC value
                if (i == 1 || i == 2) { // NTC1 (GPIO34) or NTC2 (GPIO35)
                    int16_t temp_x100 = adc_ntc_to_temperature(filtered);
                    mb_input_registers[reg_index] = temp_x100;
                    
                    ESP_LOGD(TAG, "ADC CH%d (GPIO%d, NTC): raw=%d, filtered=%d, temp=%.2f°C", 
                             i, adc_channels[i].gpio, raw, filtered, temp_x100 / 100.0f);
                } else { // AIN (GPIO32)
                    mb_input_registers[reg_index] = (int16_t)filtered;
                    
                    ESP_LOGD(TAG, "ADC CH%d (GPIO%d, AIN): raw=%d, filtered=%d", 
                             i, adc_channels[i].gpio, raw, filtered);
                }
            } else {
                ESP_LOGE(TAG, "ADC register address 0x%04X out of range", reg_addr);
            }
        }
        
        // Wait for next update interval
        vTaskDelayUntil(&last_wake_time, interval);
    }
}

/**
 * @brief Initialize ADC module
 */
esp_err_t adc_init(void) {
    if (adc_initialized) {
        ESP_LOGW(TAG, "ADC already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing ADC on GPIO%d, GPIO%d, GPIO%d", 
             ADC_CH0_GPIO, ADC_CH1_GPIO, ADC_CH2_GPIO);
    
    // Configure ADC1 with 12-bit resolution
    esp_err_t ret = adc1_config_width(ADC_WIDTH_BIT_12);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC width: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure attenuation for all channels (11dB attenuation, 0-3.3V range)
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);  // GPIO32
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);  // GPIO34
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);  // GPIO35
    
    // Initialize filters
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
        adc_filter_init(&adc_channels[i].filter);
        adc_channels[i].raw_value = 0;
        adc_channels[i].filtered_value = 0;
    }
    
    adc_initialized = true;
    ESP_LOGI(TAG, "ADC initialized successfully");
    
    return ESP_OK;
}

/**
 * @brief Start ADC reading task
 */
esp_err_t adc_start(void) {
    if (!adc_initialized) {
        ESP_LOGE(TAG, "ADC not initialized. Call adc_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (adc_task_handle != NULL) {
        ESP_LOGW(TAG, "ADC task already running");
        return ESP_OK;
    }
    
    // Create ADC reading task
    BaseType_t ret = xTaskCreate(
        adc_task,
        "adc_task",
        4096,  // Stack size
        NULL,
        4,     // Priority
        &adc_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADC task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "ADC task started");
    return ESP_OK;
}

/**
 * @brief Stop ADC reading task
 */
esp_err_t adc_stop(void) {
    if (adc_task_handle == NULL) {
        return ESP_OK;
    }
    
    vTaskDelete(adc_task_handle);
    adc_task_handle = NULL;
    
    ESP_LOGI(TAG, "ADC task stopped");
    return ESP_OK;
}

/**
 * @brief Get current filtered ADC value for a channel
 */
esp_err_t adc_get_filtered_value(uint8_t channel, uint16_t *value) {
    if (channel >= ADC_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *value = adc_channels[channel].filtered_value;
    return ESP_OK;
}

/**
 * @brief Get raw (unfiltered) ADC value for a channel
 */
esp_err_t adc_get_raw_value(uint8_t channel, uint16_t *value) {
    if (channel >= ADC_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *value = adc_channels[channel].raw_value;
    return ESP_OK;
}

