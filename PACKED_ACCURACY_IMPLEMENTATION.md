# ICM-20948 Packed Accuracy Implementation

## Summary

Successfully implemented packed accuracy format for ICM-20948 quaternion sensors, where accuracy information is included directly with quaternion data instead of requiring separate sensor channels.

## Implementation Details

### Sensor Channels with Packed Accuracy

1. **`SENSOR_CHAN_GAME_ROTATION_VECTOR`** (Standard Zephyr channel)
   - **Data Format**: 5 values: quaternion w,x,y,z + accuracy
   - **Quaternion**: w,x,y,z (dimensionless, range -1 to 1)
   - **Accuracy**: Minimum of accelerometer and gyroscope accuracy (0-3 scale)
   - **Source**: 6-axis (accelerometer + gyroscope)

2. **`SENSOR_CHAN_ICM20948_ROTATION_VECTOR`** (Custom channel)
   - **Data Format**: 5 values: quaternion w,x,y,z + accuracy
   - **Quaternion**: w,x,y,z (dimensionless, range -1 to 1) 
   - **Accuracy**: Rotation vector accuracy (normalized 0.0-1.0)
   - **Source**: 9-axis (accelerometer + gyroscope + magnetometer)

3. **`SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION`** (Custom channel)
   - **Data Format**: 4 values: x,y,z linear acceleration + accuracy
   - **Linear Acceleration**: x,y,z components (m/s²)
   - **Accuracy**: Accelerometer accuracy (0-3 scale)
   - **Source**: Derived from accelerometer data with gravity removed

## Usage Example

```c
struct sensor_value quat_with_accuracy[5];
int ret;

/* Get Game Rotation Vector (6-axis) with accuracy */
ret = sensor_channel_get(dev, SENSOR_CHAN_GAME_ROTATION_VECTOR, quat_with_accuracy);
if (ret == 0) {
    printk("Game Rotation Vector:\n");
    printk("  Quaternion: w=%d.%06d x=%d.%06d y=%d.%06d z=%d.%06d\n",
           quat_with_accuracy[0].val1, abs(quat_with_accuracy[0].val2),
           quat_with_accuracy[1].val1, abs(quat_with_accuracy[1].val2),
           quat_with_accuracy[2].val1, abs(quat_with_accuracy[2].val2),
           quat_with_accuracy[3].val1, abs(quat_with_accuracy[3].val2));
    printk("  Accuracy: %d (0=unreliable, 3=high)\n", quat_with_accuracy[4].val1);
}

/* Get 9-axis Rotation Vector with accuracy */
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_ROTATION_VECTOR, quat_with_accuracy);
if (ret == 0) {
    printk("9-axis Rotation Vector:\n");
    printk("  Quaternion: w=%d.%06d x=%d.%06d y=%d.%06d z=%d.%06d\n",
           quat_with_accuracy[0].val1, abs(quat_with_accuracy[0].val2),
           quat_with_accuracy[1].val1, abs(quat_with_accuracy[1].val2),
           quat_with_accuracy[2].val1, abs(quat_with_accuracy[2].val2),
           quat_with_accuracy[3].val1, abs(quat_with_accuracy[3].val2));
    printk("  Accuracy: %d.%06d (0.0=unreliable, 1.0=high)\n",
           quat_with_accuracy[4].val1, abs(quat_with_accuracy[4].val2));
}

/* Get Linear Acceleration with accuracy */
struct sensor_value linear_accel_with_accuracy[4];
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION, linear_accel_with_accuracy);
if (ret == 0) {
    printk("Linear Acceleration:\n");
    printk("  X=%d.%06d Y=%d.%06d Z=%d.%06d m/s²\n",
           linear_accel_with_accuracy[0].val1, abs(linear_accel_with_accuracy[0].val2),
           linear_accel_with_accuracy[1].val1, abs(linear_accel_with_accuracy[1].val2),
           linear_accel_with_accuracy[2].val1, abs(linear_accel_with_accuracy[2].val2));
    printk("  Accuracy: %d (0=unreliable, 3=high)\n", linear_accel_with_accuracy[3].val1);
}
```

## Benefits

1. **Simplified API**: Accuracy is automatically included with quaternion data
2. **Atomic Data**: Quaternion and its accuracy are retrieved together, ensuring consistency
3. **Efficient**: No need for separate sensor_channel_get() calls for accuracy
4. **Future-proof**: Can easily add more metadata (e.g., timestamp) using the same pattern

## Data Layout

For quaternion channels, the `sensor_value` array contains:
- `val[0]`: Quaternion w component
- `val[1]`: Quaternion x component  
- `val[2]`: Quaternion y component
- `val[3]`: Quaternion z component
- `val[4]`: Accuracy value

## Build Verification

✅ **Build Status**: SUCCESS
- **Flash**: 88,324 bytes (8.42% of 1MB)
- **RAM**: 16,160 bytes (6.16% of 256KB)
- **Target**: ppod_v2p0 board
- **Warnings**: Only expected warnings for custom enum values

## Files Modified

1. `drivers/sensor/icm20948/icm20948.h` - Updated channel definitions
2. `drivers/sensor/icm20948/icm20948.c` - Modified channel_get to pack accuracy
3. `samples/icm20948/src/sensor_enable_demo.c` - Updated to demonstrate packed accuracy

## Legacy Support

Individual accuracy channels are still available if needed:
- `SENSOR_CHAN_ICM20948_ACCEL_ACCURACY`
- `SENSOR_CHAN_ICM20948_GYRO_ACCURACY` 
- `SENSOR_CHAN_ICM20948_MAGN_ACCURACY`
