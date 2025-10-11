#ifndef HPC_MQTT_H
#define HPC_MQTT_H

#include "esp_err.h"
#include <stdbool.h>

#define MAX_TOPIC_LEN 42 // max length + 1


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *broker_uri;
    const char *client_id;
    const char *username;
    const char *password;
    const char *base_topic;
} hpc_mqtt_config_t;

esp_err_t hpc_mqtt_init(const hpc_mqtt_config_t *config);
esp_err_t hpc_mqtt_start(void);
esp_err_t hpc_mqtt_publish_str(const char *subtopic, const char *payload, int qos, bool retain);
esp_err_t hpc_mqtt_publish_int(const char *subtopic, int value, int qos, bool retain);
esp_err_t hpc_mqtt_publish_float(const char *subtopic, float value, int qos, bool retain, int precision);

#ifdef __cplusplus
}
#endif

#endif // HPC_MQTT_H

