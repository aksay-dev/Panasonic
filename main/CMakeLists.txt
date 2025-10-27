idf_component_register(SRCS "commands.c" "decoder.c" "protocol.c" "hpc.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_timer driver)