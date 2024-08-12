idf_component_register(SRCS "esp32-multisensor-firmware.c"
                    INCLUDE_DIRS "." "../sensors/include" "../mcu_interfaces/include"
                    REQUIRES soc driver)