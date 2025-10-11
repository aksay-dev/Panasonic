/**
 * @file hpc.h
 * @brief Main header file for HPC application
 * @version 0.1.0
 * @date 2025
 */

#ifndef HPC_H
#define HPC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Application version
#define HPC_VERSION_STRING "0.1.0"

// Data sizes
#define HPC_MAIN_DATA_SIZE 203
#define HPC_EXTRA_DATA_SIZE 203
#define HPC_OPT_DATA_SIZE 20
#define HPC_MAX_PACKET_SIZE 256

// Heat pump data structure
typedef struct {
    uint8_t main_data[HPC_MAIN_DATA_SIZE];
    uint8_t extra_data[HPC_EXTRA_DATA_SIZE];
    uint8_t opt_data[HPC_OPT_DATA_SIZE];
    bool main_data_valid;
    bool extra_data_valid;
    bool opt_data_valid;
    uint32_t last_update;
} hpc_data_t;

/**
 * @brief Initialize HPC application
 * @return ESP_OK on success
 */
esp_err_t hpc_init(void);

/**
 * @brief Start HPC application
 * @return ESP_OK on success
 */
esp_err_t hpc_start(void);


#ifdef __cplusplus
}
#endif

#endif // HPC_H
