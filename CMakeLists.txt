cmake_minimum_required(VERSION 3.16)

list(APPEND EXTRA_COMPONENT_DIRS "sensors" "mcu_interfaces")

include_directories(${PROJECT_NAME} "sensors" "mcu_interfaces")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-multisensor-firmware)
