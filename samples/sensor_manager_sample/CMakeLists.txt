# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_manager_sample)

# Include sensor manager library
target_include_directories(app PRIVATE ../../lib/src)

# Add driver include path for custom attributes
target_include_directories(app PRIVATE ../../drivers/sensor/icm20948)

# Add source files
target_sources(app PRIVATE 
    src/main.c
    src/ble_sensor_service.c
)
