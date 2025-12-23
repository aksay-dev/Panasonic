idf_component_register(SRCS "http_server.c" "ds18b20.c" "adc.c" "wifi_connect.c" "mqtt_client.c" "nvs_hp.c" "modbus_slave.c" "modbus_params.c" "commands.c" "decoder.c" "protocol.c" "hpc.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_timer driver esp-modbus nvs_flash mqtt esp_wifi esp_netif esp_event esp_http_client esp_http_server json onewire_bus ds18b20 esp_adc)