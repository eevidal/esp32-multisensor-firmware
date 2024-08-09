idf_component_register(SRCS "src/gpio_module.c"
 "src/i2c_module.c" "src/error_module.c" "src/time_module.c"
                        INCLUDE_DIRS "." "include" 
                        REQUIRES driver freertos hal)
