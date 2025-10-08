idf_component_register(SRCS "flex_firmware.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos lwip esp_wifi esp_event nvs_flash driver esp_timer)
