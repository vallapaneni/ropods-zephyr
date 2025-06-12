# ICM20948 Self-Test Implementation Summary

## Overview
Successfully implemented hardware self-test functionality for the ICM20948 sensor driver via a custom sensor attribute interface.

## Changes Made

### 1. Header File Updates (`icm20948.h`)
- **Added new attribute**: `SENSOR_ATTR_ICM20948_SELF_TEST` to the `icm20948_sensor_attribute` enum
- **Type**: Write-only (trigger-based) attribute
- **Location**: Line 18 in the enum definition

### 2. Implementation (`icm20948_attr.c`)
- **Added include**: `#include "Invn/Devices/Drivers/ICM20948/Icm20948SelfTest.h"`
- **New function**: `icm20948_attr_set_self_test()` for handling self-test execution
- **Enhanced switch statements**: Added cases for both set and get operations
- **Error handling**: Comprehensive error code conversion from eMD to Zephyr errors

### 3. Key Features Implemented

#### Self-Test Function (`icm20948_attr_set_self_test`)
- **Trigger logic**: Any non-zero value triggers self-test, zero value ignored
- **eMD integration**: Calls `inv_icm20948_run_selftest()` from eMD library
- **Error conversion**: Maps eMD error codes to standard Zephyr error codes:
  - `INV_ERROR_BAD_ARG` → `-EINVAL`
  - `INV_ERROR_TIMEOUT` → `-ETIMEDOUT`
  - `INV_ERROR_TRANSPORT` → `-EIO`
  - `INV_ERROR_HW` → `-ENODEV`
  - Other errors → `-EIO`
- **Logging**: Comprehensive info/error logging for diagnostics

#### Attribute Handling Integration
- **Set operation**: Added to main switch in `icm20948_attr_set()`
- **Get operation**: Returns `-EACCES` (write-only attribute)
- **Parameter validation**: Standard null pointer checks

## Usage Example

```c
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "icm20948.h"

int run_self_test(const struct device *icm_dev) {
    struct sensor_value trigger = {.val1 = 1, .val2 = 0};
    
    int ret = sensor_attr_set(icm_dev, 
                             SENSOR_CHAN_ALL,
                             SENSOR_ATTR_ICM20948_SELF_TEST, 
                             &trigger);
    
    return ret; // 0 = success, negative = error
}
```

## Documentation Created
1. **Usage Guide**: `ICM20948_SELF_TEST_USAGE.md` - Comprehensive usage documentation
2. **Demo Applications**: 
   - `self_test_demo.c` - Full demonstration with error handling
   - `self_test_simple.c` - Minimal usage example

## Integration Points

### eMD Library Integration
- **Function used**: `inv_icm20948_run_selftest()`
- **Library path**: `lib/emd/EMD-Core/sources/Invn/Devices/Drivers/ICM20948/`
- **Header included**: `Icm20948SelfTest.h`

### Zephyr Sensor Framework
- **Attribute type**: Custom sensor attribute (`SENSOR_ATTR_PRIV_START + 1`)
- **Interface**: Standard `sensor_attr_set()` API
- **Error codes**: Standard Zephyr negative error codes

## Benefits

### For Applications
- **Standard interface**: Uses existing Zephyr sensor attribute framework
- **Hardware validation**: Built-in sensor diagnostics capability
- **Error reporting**: Clear error codes for different failure modes
- **Production testing**: Ready for manufacturing test integration

### For Diagnostics
- **Field testing**: Can verify sensor health in deployed systems
- **Troubleshooting**: Distinguishes between hardware and software issues
- **Safety-critical**: Periodic sensor validation capability

## Testing Validation
- **Zero trigger test**: Ensures zero values don't trigger self-test
- **Error handling test**: Validates proper error code conversion
- **Read attempt test**: Confirms write-only behavior
- **Integration test**: Verifies normal operation after self-test

## Files Modified
```
drivers/sensor/icm20948/
├── icm20948.h                      # Attribute definition
├── icm20948_attr.c                 # Implementation
├── ICM20948_SELF_TEST_USAGE.md     # Documentation
├── self_test_demo.c                # Demo application
└── self_test_simple.c              # Simple example
```

## Compatibility
- **Zephyr version**: Compatible with current Zephyr sensor framework
- **eMD library**: Uses existing eMD self-test functionality
- **Hardware**: Works with all ICM20948 sensor variants
- **Thread safety**: Inherits thread safety from Zephyr sensor framework

## Future Enhancements
- Could add individual sensor self-test control (accel/gyro/mag separately)
- Could expose detailed self-test results beyond pass/fail
- Could add self-test scheduling/automation features
- Could integrate with system health monitoring frameworks

## Conclusion
The ICM20948 self-test attribute implementation provides a robust, standard-compliant interface for hardware diagnostics while maintaining full compatibility with the existing Zephyr sensor ecosystem and eMD library integration.
