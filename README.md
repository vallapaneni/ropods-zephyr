# ICM20948 SPI Driver for Zephyr RTOS

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Zephyr](https://img.shields.io/badge/Zephyr-4.1.0+-brightgreen.svg)](https://www.zephyrproject.org/)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()

An out-of-tree SPI driver for the InvenSense ICM20948 9-axis motion tracking device, designed for the Zephyr RTOS. This driver provides complete SPI-based communication with the ICM20948 sensor, supporting accelerometer, gyroscope, and temperature measurements.

## 🚀 Features

- **SPI-Only Communication**: Optimized SPI interface with I2C disabled for better performance
- **9-Axis Sensor Support**: 
  - 3-axis accelerometer
  - 3-axis gyroscope  
  - Internal temperature sensor
- **Configurable Sensor Ranges**: 
  - Accelerometer: ±2g, ±4g, ±8g, ±16g
  - Gyroscope: ±250, ±500, ±1000, ±2000 dps
- **Interrupt Support**: Hardware data-ready interrupt with multiple threading modes
- **Full Zephyr Integration**: 
  - Device tree bindings
  - Kconfig configuration
  - Standard Zephyr sensor API
- **Board Support**: Pre-configured overlays for popular development boards
- **Production Ready**: Comprehensive error handling, logging, and documentation

## File Structure

```
ropods-zephyr/
├── CMakeLists.txt                          # Top-level build configuration
├── Kconfig                                 # Top-level Kconfig
├── zephyr/module.yml                       # Zephyr module definition
├── drivers/                                # Driver implementation
│   ├── CMakeLists.txt
│   ├── Kconfig
│   └── sensor/
│       ├── CMakeLists.txt
│       ├── Kconfig
│       └── icm20948/
│           ├── CMakeLists.txt
│           ├── icm20948.h                  # Driver header
│           ├── icm20948.c                  # Main driver implementation
│           └── icm20948_trigger.c          # Interrupt support
├── dts/bindings/sensor/
│   └── invensense,icm20948.yaml            # Device tree binding
└── samples/icm20948/                       # Sample application
    ├── CMakeLists.txt
    ├── prj.conf
    ├── boards/generic.overlay
    └── src/
        ├── CMakeLists.txt
        └── main.c
```

## Usage

### 1. Add to Your Project

To use this driver in your Zephyr project, add this module to your `west.yml`:

```yaml
manifest:
  projects:
    - name: ropods-zephyr
      url: <your-repository-url>
      path: modules/ropods-zephyr
```

### 2. Enable the Driver

Add to your `prj.conf`:

```
CONFIG_ROPODS_DRIVERS=y
CONFIG_ICM20948=y
CONFIG_SPI=y
CONFIG_SENSOR=y
```

For interrupt support, also add:
```
CONFIG_GPIO=y
CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD=y
```

### 3. Device Tree Configuration

Add the ICM20948 to your device tree overlay:

```dts
&spi1 {
    status = "okay";
    cs-gpios = <&gpio0 3 GPIO_ACTIVE_LOW>;

    icm20948: icm20948@0 {
        compatible = "invensense,icm20948";
        reg = <0>;
        spi-max-frequency = <8000000>;
        
        accel-range = <2>;    /* ±2g */
        gyro-range = <250>;   /* ±250 dps */
        
        /* Optional interrupt pin */
        int-gpios = <&gpio0 4 GPIO_ACTIVE_HIGH>;
    };
};
```

### 4. Application Code

```c
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define ICM20948_NODE DT_NODELABEL(icm20948)
static const struct device *icm20948 = DEVICE_DT_GET(ICM20948_NODE);

int main(void)
{
    struct sensor_value accel[3], gyro[3], temp;
    
    if (!device_is_ready(icm20948)) {
        return -ENODEV;
    }
    
    while (1) {
        sensor_sample_fetch(icm20948);
        
        sensor_channel_get(icm20948, SENSOR_CHAN_ACCEL_XYZ, accel);
        sensor_channel_get(icm20948, SENSOR_CHAN_GYRO_XYZ, gyro);
        sensor_channel_get(icm20948, SENSOR_CHAN_DIE_TEMP, &temp);
        
        // Process sensor data...
        
        k_sleep(K_MSEC(100));
    }
}
```

## Configuration Options

| Option | Description | Default |
|--------|-------------|---------|
| `CONFIG_ICM20948` | Enable ICM20948 driver | n |
| `CONFIG_ICM20948_TRIGGER_NONE` | No interrupt support | y |
| `CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD` | Use global thread for interrupts | n |
| `CONFIG_ICM20948_TRIGGER_OWN_THREAD` | Use dedicated thread for interrupts | n |
| `CONFIG_ICM20948_THREAD_PRIORITY` | Interrupt thread priority | 10 |
| `CONFIG_ICM20948_THREAD_STACK_SIZE` | Interrupt thread stack size | 1024 |

## Device Tree Properties

| Property | Type | Description | Default |
|----------|------|-------------|---------|
| `compatible` | string | Must be "invensense,icm20948" | - |
| `reg` | int | SPI chip select | - |
| `spi-max-frequency` | int | Maximum SPI frequency | - |
| `accel-range` | int | Accelerometer range (2, 4, 8, 16) | 2 |
| `gyro-range` | int | Gyroscope range (250, 500, 1000, 2000) | 250 |
| `int-gpios` | phandle-array | Interrupt GPIO (optional) | - |

## Hardware Connections

### SPI Connections
- **MOSI**: Connect to SPI MOSI pin
- **MISO**: Connect to SPI MISO pin  
- **SCLK**: Connect to SPI SCLK pin
- **CS**: Connect to GPIO pin (configured in device tree)
- **VDD**: 1.8V - 3.6V power supply
- **GND**: Ground

### Optional Interrupt
- **INT**: Connect to GPIO pin for data ready interrupts

## Sample Application

The included sample application demonstrates:
- Basic sensor reading in polling mode
- Interrupt-driven data acquisition
- Proper error handling
- Data formatting and logging

To build and run the sample:

```bash
cd samples/icm20948
west build -b <your_board>
west flash
```

## Limitations

- Currently only supports SPI interface (no I2C)
- Magnetometer (AK09916) support not yet implemented
- DMP (Digital Motion Processor) features not implemented
- FIFO functionality not implemented

## License

SPDX-License-Identifier: Apache-2.0
