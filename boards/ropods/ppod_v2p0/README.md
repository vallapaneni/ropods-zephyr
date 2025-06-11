# PPOD v2.0 Board Support

This directory contains the board support package (BSP) for the PPOD v2.0 hardware platform.

## Hardware Overview

The PPOD v2.0 is based on the Nordic nRF52840 SoC and includes:

- **MCU**: Nordic nRF52840 (ARM Cortex-M4 @ 64MHz, 1MB Flash, 256KB RAM)
- **Connectivity**: Bluetooth 5.0, IEEE 802.15.4, USB 2.0
- **Sensors**: 
  - ICM20948 9-axis motion sensor (SPI)
  - BQ274xx fuel gauge (I2C)
- **Actuators**: 
  - DRV2605 haptic driver (I2C)
  - RGB LED (PWM capable)
- **Storage**: MX25R64 8MB QSPI flash
- **Power**: Power management with hold control
- **User Interface**: 1 button, RGB LED, charge status indicator

## Building Applications

To build applications for the PPOD v2.0 board:

```bash
west build -b ppod_v2p0 path/to/your/application
```

## Board Configuration

The board provides the following configuration options:

- **Power management**: Configurable power hold GPIO and LED signaling
- **Sensor support**: ICM20948 motion sensor with interrupt capability
- **Storage**: QSPI flash with LittleFS support
- **Connectivity**: All nRF52840 wireless features available

## Device Tree

Key device tree nodes:
- `icm20948`: 9-axis motion sensor on SPI2
- `ti_bq274xx`: Fuel gauge on I2C0
- `drv2605`: Haptic driver on I2C1
- `mx25r64`: QSPI flash storage

## Sample Applications

The ropods-zephyr repository includes sample applications specifically designed for this board:

- **ICM20948 sensor sample**: Demonstrates motion sensor reading with interrupt control

## Notes

- The board includes custom power management initialization
- RGB LEDs provide visual feedback during power on/off sequences
- All GPIO pins are properly configured with pull-ups/downs as needed
- The board supports MCUboot for secure firmware updates
