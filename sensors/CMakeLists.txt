idf_component_register(SRCS "src/hcrs04.c" "src/apds9960.c" "src/mpu6050.c"
                        INCLUDE_DIRS "include" "../mcu_interfaces/include"
                        REQUIRES driver hal esp_timer)