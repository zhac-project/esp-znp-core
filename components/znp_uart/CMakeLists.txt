idf_component_register(SRCS "znp_uart.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart esp_driver_gpio mt_proto)
