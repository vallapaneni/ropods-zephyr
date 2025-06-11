# SPDX-License-Identifier: Apache-2.0

# ICM-20948 sensor driver with EMD integration

zephyr_library()

# Always include the core driver
zephyr_library_sources(icm20948.c)
zephyr_library_sources(icm20948_attr.c)
zephyr_library_sources(icm20948_platform.c)

# Include SPI transport (skip I2C implementation)
zephyr_library_sources(icm20948_spi.c)

# Include trigger support if enabled
zephyr_library_sources_ifdef(CONFIG_ICM20948_TRIGGER icm20948_trigger.c)

# Add InvenSense lib/emd paths (if available)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/emd/EMD-Core/sources)
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/emd/EMD-Core/sources)
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/emd/EMD-Core/sources/Invn/EmbUtils)
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/emd/EMD-Core/sources/Invn/Devices)
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/emd/EMD-Core/sources/Invn/Devices/Drivers/ICM20948)
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/emd/EMD-Core/sources/Invn/Devices/SensorTypes)
    
    # Add lib/emd sources (if they exist)
    file(GLOB EMD_ICM20948_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/emd/EMD-Core/sources/Invn/Devices/Drivers/ICM20948/*.c
    )
    if(EMD_ICM20948_SOURCES)
        zephyr_library_sources(${EMD_ICM20948_SOURCES})
    endif()
endif()
