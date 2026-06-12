idf_component_register(SRCS "znp_ezb.c"
                       INCLUDE_DIRS "include"
                       REQUIRES znp_dispatch znp_uart esp_hw_support esp_system
                                espressif__esp-zigbee-lib freertos nvs_flash)
