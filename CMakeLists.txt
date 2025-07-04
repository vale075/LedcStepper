idf_component_register(SRCS "LedcStepper.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        esp_driver_gpio
                        esp_driver_ledc
                        esp_driver_pcnt)
