# ICM20948 Self-Test Usage Guide

## Overview

The ICM20948 driver now supports hardware self-test functionality via a custom sensor attribute. Self-test is a hardware diagnostic feature that verifies the proper functioning of the accelerometer, gyroscope, and magnetometer sensors.

## When to Use Self-Test

Self-test is useful for:
- **Production testing**: Verify sensor functionality during manufacturing
- **Field diagnostics**: Troubleshoot sensor issues in deployed systems  
- **Safety-critical applications**: Periodic verification of sensor health
- **Initial system validation**: Confirm proper sensor operation after power-on

## Usage Example

```c
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "icm20948.h"

int run_icm20948_self_test(const struct device *icm_dev)
{
    struct sensor_value trigger_val = {.val1 = 1, .val2 = 0}; // Any non-zero value
    int ret;
    
    // Trigger self-test on all sensors
    ret = sensor_attr_set(icm_dev, 
                         SENSOR_CHAN_ALL,  // Channel ignored for self-test
                         SENSOR_ATTR_ICM20948_SELF_TEST, 
                         &trigger_val);
    
    if (ret == 0) {
        printk("ICM20948 self-test passed successfully\\n");
        return 0;
    } else {
        printk("ICM20948 self-test failed with error: %d\\n", ret);
        return ret;
    }
}
```

## Implementation Details

### Attribute Definition
- **Attribute**: `SENSOR_ATTR_ICM20948_SELF_TEST`
- **Type**: Write-only (trigger-based)
- **Value**: Any non-zero value triggers self-test, zero value is ignored
- **Channel**: Ignored (self-test runs on all sensors automatically)

### Return Values
- **0**: Self-test completed successfully
- **-EINVAL**: Invalid arguments passed to eMD library
- **-ETIMEDOUT**: Self-test operation timed out
- **-EIO**: Communication error or generic hardware failure
- **-ENODEV**: Hardware not responding or defective
- **-EACCES**: Returned if attempting to read (get) the attribute

### Error Handling
The driver automatically converts eMD library error codes to standard Zephyr error codes:

| eMD Error | Zephyr Error | Description |
|-----------|--------------|-------------|
| `INV_ERROR_BAD_ARG` | `-EINVAL` | Invalid arguments |
| `INV_ERROR_TIMEOUT` | `-ETIMEDOUT` | Operation timeout |
| `INV_ERROR_TRANSPORT` | `-EIO` | Communication error |
| `INV_ERROR_HW` | `-ENODEV` | Hardware failure |
| Other errors | `-EIO` | Generic I/O error |

### Logging
The driver provides comprehensive logging:
- **INFO**: Self-test trigger and success messages
- **ERROR**: Self-test failure with error codes
- **WARNING**: Invalid usage (e.g., trying to read write-only attribute)

## Self-Test vs Calibration

| Feature | Self-Test | Calibration |
|---------|-----------|-------------|
| **Purpose** | Hardware diagnostics | Data accuracy correction |
| **Frequency** | On-demand/periodic | Automatic/continuous |
| **Duration** | ~100ms | Ongoing background process |
| **Output** | Pass/Fail status | Bias correction values |
| **When to use** | Fault detection | Normal operation |

## Integration Notes

1. **Device Initialization**: Self-test can be run after `sensor_attr_set()` calls
2. **Timing**: Self-test takes approximately 100ms to complete
3. **Thread Safety**: The attribute handler is thread-safe via Zephyr's sensor framework
4. **Power Consumption**: Self-test temporarily increases power consumption during execution
5. **Sensor State**: Self-test may temporarily affect normal sensor readings during execution

## Troubleshooting

### Common Issues
1. **Self-test fails with -EIO**: Check SPI/I2C communication, verify wiring
2. **Self-test fails with -ENODEV**: Sensor may be damaged or improperly powered
3. **Self-test fails with -ETIMEDOUT**: Bus communication too slow or sensor stuck

### Debugging Tips
- Enable ICM20948 driver logging to see detailed error messages
- Verify sensor is properly initialized before running self-test
- Check that sensor is not in sleep/low-power mode during self-test
- Ensure stable power supply during self-test execution
