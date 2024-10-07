idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt nvs_flash driver
                    PRIV_REQUIRES esp_adc bt)