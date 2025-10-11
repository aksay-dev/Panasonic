idf_component_register(SRCS "decoder.c" "mqtt.c" "protocol.c" "hpc.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash esp_timer driver)