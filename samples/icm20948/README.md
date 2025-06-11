# ICM20948 Sample Application

This sample demonstrates how to use the ICM20948 SPI driver to read accelerometer, gyroscope, and temperature data.

## Overview

The sample application shows two operating modes:

1. **Polling Mode** (default): Continuously reads sensor data every second
2. **Interrupt Mode**: Uses data ready interrupts to trigger sensor readings

## Building and Running

### Prerequisites

- Zephyr development environment set up
- Board with SPI support
- ICM20948 sensor connected via SPI

### Build

```bash
# For generic board (modify overlay as needed)
west build -b <your_board>

# For Nordic nRF52840 DK
west build -b nrf52840dk_nrf52840

# For STM32F4 Discovery
west build -b stm32f4_disco
```

### Flash and Run

```bash
west flash
```

### Monitor Output

```bash
# For serial console
west build -t menuconfig  # Configure console settings if needed
# Connect to serial port at 115200 baud
```

## Hardware Setup

### SPI Connections

Connect the ICM20948 to your board's SPI interface:

| ICM20948 Pin | Board Pin | Description |
|-------------|-----------|-------------|
| VDD | 3.3V | Power supply |
| GND | GND | Ground |
| SCLK | SPI SCLK | SPI Clock |
| MOSI | SPI MOSI | Master Out Slave In |
| MISO | SPI MISO | Master In Slave Out |
| CS | GPIO (e.g., P0.03) | Chip Select |
| INT | GPIO (e.g., P0.04) | Interrupt (optional) |

### Board-Specific Examples

#### Nordic nRF52840 DK
```
ICM20948    nRF52840 DK
VDD    -->  VDD (3.3V)
GND    -->  GND
SCLK   -->  P0.31 (SPI1 SCLK)
MOSI   -->  P0.30 (SPI1 MOSI)  
MISO   -->  P0.29 (SPI1 MISO)
CS     -->  P0.03 (GPIO)
INT    -->  P0.04 (GPIO, optional)
```

#### STM32F4 Discovery
```
ICM20948    STM32F4 Discovery
VDD    -->  3V
GND    -->  GND
SCLK   -->  PA5 (SPI1 SCLK)
MOSI   -->  PA7 (SPI1 MOSI)
MISO   -->  PA6 (SPI1 MISO)
CS     -->  PA4 (GPIO)
INT    -->  PA8 (GPIO, optional)
```

## Configuration

### Enable Interrupt Mode

To use interrupt-driven data acquisition, modify `prj.conf`:

```
CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD=y
```

And ensure your device tree overlay includes the interrupt GPIO:

```dts
icm20948: icm20948@0 {
    // ... other properties ...
    int-gpios = <&gpio0 4 GPIO_ACTIVE_HIGH>;
};
```

### Adjust Sensor Ranges

Modify the device tree overlay to change sensor ranges:

```dts
icm20948: icm20948@0 {
    // ... other properties ...
    accel-range = <4>;     /* ±4g instead of ±2g */
    gyro-range = <500>;    /* ±500 dps instead of ±250 dps */
};
```

## Expected Output

### Polling Mode
```
[00:00:00.123,456] <inf> icm20948_sample: ICM20948 device ready
[00:00:00.123,456] <inf> icm20948_sample: Polling mode - reading sensor every second
[00:00:01.123,456] <inf> icm20948_sample: Accel: X=0.123456 Y=-0.456789 Z=9.812345 m/s²
[00:00:01.123,456] <inf> icm20948_sample: Gyro: X=0.001234 Y=-0.002345 Z=0.000123 rad/s
[00:00:01.123,456] <inf> icm20948_sample: Temperature: 23.450000 °C
```

### Interrupt Mode
```
[00:00:00.123,456] <inf> icm20948_sample: ICM20948 device ready
[00:00:00.123,456] <inf> icm20948_sample: Trigger mode enabled - data will be printed on interrupt
[00:00:00.234,567] <inf> icm20948_sample: Accel: X=0.123456 Y=-0.456789 Z=9.812345 m/s²
[00:00:00.234,567] <inf> icm20948_sample: Gyro: X=0.001234 Y=-0.002345 Z=0.000123 rad/s
[00:00:00.234,567] <inf> icm20948_sample: Temperature: 23.450000 °C
```

## Troubleshooting

### Common Issues

1. **Device not ready**: Check SPI connections and power supply
2. **Invalid chip ID**: Verify SPI communication and CS pin configuration
3. **No interrupt**: Check interrupt GPIO connection and configuration
4. **Noisy readings**: Ensure proper power supply filtering and stable connections

### Debug Tips

1. Enable debug logging:
   ```
   CONFIG_LOG=y
   CONFIG_SENSOR_LOG_LEVEL_DBG=y
   ```

2. Check SPI configuration in device tree
3. Verify GPIO pin assignments match your hardware
4. Use an oscilloscope to verify SPI signals if available

## Extending the Sample

You can extend this sample to:

- Add magnetometer support (when implemented)
- Implement sensor fusion algorithms
- Add FIFO buffering
- Create custom trigger conditions
- Add low-power modes
