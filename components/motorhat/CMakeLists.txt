idf_component_register(SRCS "motorhat.c" "motorhat_servo.c" "pca9685.c" "servo.c"
                    INCLUDE_DIRS "include" REQUIRES esp_driver_i2c i2c_bus signal_bus esp_driver_gpio)
