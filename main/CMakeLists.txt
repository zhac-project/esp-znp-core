idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash espressif__esp-zigbee-lib
                                znp_uart znp_dispatch znp_ezb)
