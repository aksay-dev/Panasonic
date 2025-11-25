idf_component_register(SRCS "nvs_hp.c" "modbus_slave.c" "modbus_params.c" "commands.c" "decoder.c" "protocol.c" "hpc.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_timer driver esp-modbus nvs_flash)