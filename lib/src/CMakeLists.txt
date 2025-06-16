# CMakeLists.txt for ROPODS Libraries

# Define the sensor manager library
zephyr_library_named(sensor_manager)

# Add source files
zephyr_library_sources(sensor_manager.c)

# Add BLE service if enabled
zephyr_library_sources_ifdef(CONFIG_ROPODS_BLE_SERVICE ropods_ble_service.c)

# Add include directories
zephyr_library_include_directories(.)

# Link with required Zephyr libraries
zephyr_library_link_libraries(kernel)

# Add compile options if needed
zephyr_library_compile_options_ifdef(CONFIG_SENSOR_MANAGER_DEBUG -DDEBUG)

# Export include directory to other targets
zephyr_include_directories(.)
