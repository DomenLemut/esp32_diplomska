idf_component_register(SRCS "main.c" 
                    INCLUDE_DIRS "." REQUIRES LSM6DS3 esp_timer fatfs sdmmc driver)
