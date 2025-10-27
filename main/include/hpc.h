/**
 * @file hpc.h
 * @brief Main header file for HPC application
 * @version 0.1.0
 * @date 2025
 */

#ifndef HPC_H
#define HPC_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Application version
#define HPC_VERSION_STRING "0.1.0"

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
