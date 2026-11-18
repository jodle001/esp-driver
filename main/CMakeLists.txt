idf_component_register(SRCS "esp_driver.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES scorbot_core scorbot_proto link_uart i2c_bus expander motor encoder_ls7366r limit_switch current nvs_store pixel_strip
                                     esp_driver_gpio esp_timer)
