idf_component_register(SRCS "main.c" 
                    INCLUDE_DIRS "." REQUIRES LSM6DS3 RTC WiFi esp_timer fatfs sdmmc driver esp_adc)
