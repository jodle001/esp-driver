idf_component_register(SRCS "encoder_ls7366r.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_spi esp_driver_gpio esp_driver_ledc)
