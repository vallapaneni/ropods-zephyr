# ICM20948 Sensor Enable Restriction

## Overview

The ICM20948 driver now implements a restriction for sensor enabling: whenever new sensors need to be enabled, the driver will first disable all sensors and then enable only the requested ones. This simplifies the sensor control logic and ensures clean state transitions.

## Before (Complex Application Logic)

Previously, applications would need to manage sensor state transitions manually:

```c
// Complex application logic to manage transitions
uint32_t current_mask = get_current_sensor_mask();
uint32_t desired_mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR);

// App had to figure out what to disable and what to enable
uint32_t to_disable = current_mask & ~desired_mask;
uint32_t to_enable = desired_mask & ~current_mask;

// Multiple driver calls needed
if (to_disable) {
    set_sensor_enable_mask(dev, current_mask & ~to_disable);
}
set_sensor_enable_mask(dev, desired_mask);
```

## After (Simple Application Logic)

Now applications can simply specify the desired sensor mask:

```c
// Simple application logic - just specify what you want
uint32_t desired_mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR);

// Single driver call - driver handles all the complexity
set_sensor_enable_mask(dev, desired_mask);
```

## Driver Implementation

The driver now follows this pattern:

1. **If mask is 0**: Disable all sensors and interrupts
2. **If mask is non-zero**: 
   - Step 1: Disable all supported sensors
   - Step 2: Enable only the sensors specified in the mask
   - Step 3: Enable interrupts if any sensor is enabled

## Benefits

1. **Simplified Application Code**: Applications don't need to track current sensor state
2. **Predictable Behavior**: Every sensor enable operation starts from a clean state
3. **Reduced Code Complexity**: Less logic in both driver and application
4. **Fewer Error Conditions**: No need to handle partial state transitions
5. **Atomic Operations**: Sensor state changes are atomic from application perspective

## Usage Examples

### Enable Single Sensor
```c
// Enable only accelerometer (disables any previously enabled sensors)
uint32_t mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER);
sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
```

### Switch Between Sensor Combinations
```c
// Switch from accelerometer-only to accelerometer + rotation vector
uint32_t mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR);
sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
```

### Disable All Sensors
```c
// Disable all sensors and interrupts
uint32_t mask = 0;
sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
```

## Supported Sensors

The driver supports only three specific sensors:
- `INV_ICM20948_SENSOR_ACCELEROMETER`
- `INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR`
- `INV_ICM20948_SENSOR_ROTATION_VECTOR`
