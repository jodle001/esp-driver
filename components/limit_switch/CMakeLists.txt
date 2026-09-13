idf_component_register(SRCS "limit_switch.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_driver_pcnt esp_timer expander encoder motor)
