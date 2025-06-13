# Sensor Manager Sample

This sample demonstrates the sensor manager library functionality with real sensor devices.

## Overview

The sensor manager sample shows how to:
- Initialize the sensor manager
- Add sensor devices (ICM20948 IMU)
- Enable specific sensor channels
- Set up data ready callbacks
- Acquire timestamped sensor data
- Process buffered sensor data
- Monitor sensor statistics

## Requirements

### Hardware
- A board with SPI interface (nRF52840 DK, nRF52 DK, STM32F4 Discovery)
- ICM20948 IMU sensor connected via SPI

### Software
- Zephyr RTOS
- ROPODS sensor manager library
- ICM20948 driver

## Building and Running

### For nRF52840 DK:
```bash
west build -b nrf52840dk_nrf52840 samples/sensor_manager_sample
west flash
```

### For nRF52 DK:
```bash
west build -b nrf52dk_nrf52832 samples/sensor_manager_sample
west flash
```

### For STM32F4 Discovery:
```bash
west build -b stm32f4_disco samples/sensor_manager_sample
west flash
```

## Wiring

The ICM20948 sensor should be connected as follows:

### nRF52840 DK / nRF52 DK:
- VCC -> 3.3V
- GND -> GND
- SCL -> P0.27 (SPI SCLK)
- SDA -> P0.26 (SPI MOSI)
- SDO -> P0.04 (SPI MISO)
- CS -> P0.05 (SPI CS)
- INT -> P0.06 (GPIO interrupt)

### STM32F4 Discovery:
- VCC -> 3.3V
- GND -> GND
- SCL -> PA5 (SPI SCLK)
- SDA -> PA7 (SPI MOSI)
- SDO -> PA6 (SPI MISO)
- CS -> PA4 (SPI CS)
- INT -> PA0 (GPIO interrupt)

## Expected Output

The sample will output sensor data and statistics every second:

```
=====================================
  Sensor Manager Sample Application  
=====================================
Build: Jan 15 2024 10:30:45

[00:00:00.123,456] <inf> sensor_manager_sample: Starting Sensor Manager Sample
[00:00:00.234,567] <inf> sensor_manager_sample: Sensor manager initialized successfully
[00:00:00.345,678] <inf> sensor_manager_sample: ICM20948 device found and ready
[00:00:00.456,789] <inf> sensor_manager_sample: Added ICM20948 to sensor manager
[00:00:00.567,890] <inf> sensor_manager_sample: Enabled accelerometer channels
[00:00:00.678,901] <inf> sensor_manager_sample: Enabled gyroscope channels
[00:00:00.789,012] <inf> sensor_manager_sample: Started ICM20948 data acquisition
[00:00:00.890,123] <inf> sensor_manager_sample: Entering main application loop...

[00:00:01.000,000] <inf> sensor_manager_sample: ICM20948: Read 5 sensor data entries
[00:00:01.000,100] <inf> sensor_manager_sample:   [1000050 μs] Ch 10: 0.125000
[00:00:01.000,200] <inf> sensor_manager_sample:   [1000100 μs] Ch 11: -0.250000
[00:00:01.000,300] <inf> sensor_manager_sample:   [1000150 μs] Ch 12: 9.750000

[00:00:05.000,000] <inf> sensor_manager_sample: === Sensor Manager Statistics ===
[00:00:05.000,100] <inf> sensor_manager_sample: ICM20948: 150 events, 0 overflows, 25% buffer usage
[00:00:05.000,200] <inf> sensor_manager_sample: ==============================
```

## Features Demonstrated

1. **Device Management**: Adding and configuring sensor devices
2. **Channel Selection**: Enabling specific accelerometer and gyroscope channels
3. **Callback Handling**: Data ready interrupt processing
4. **Data Buffering**: Timestamped sensor data storage
5. **Statistics Monitoring**: Event counts and buffer usage tracking
6. **Thread Safety**: Multi-threaded operation with proper synchronization

## Troubleshooting

### No sensor data:
- Check wiring connections
- Verify sensor power supply
- Ensure SPI interface is properly configured
- Check interrupt pin connection

### Build errors:
- Ensure `CONFIG_ROPODS_LIBRARIES=y` is set
- Verify `CONFIG_SENSOR_MANAGER=y` is enabled
- Check that all required drivers are enabled

### Buffer overflows:
- Increase buffer size in configuration
- Reduce sensor data rate
- Process data more frequently

## Configuration Options

Key configuration options in `prj.conf`:
- `CONFIG_SENSOR_MANAGER=y` - Enable sensor manager
- `CONFIG_SENSOR_MANAGER_MAX_DEVICES=8` - Maximum devices
- `CONFIG_SENSOR_MANAGER_DEFAULT_BUFFER_SIZE=64` - Buffer size
- `CONFIG_ICM20948=y` - Enable ICM20948 driver
- `CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD=y` - Enable interrupts
