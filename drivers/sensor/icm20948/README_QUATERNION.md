# ICM20948 Quaternion and Linear Acceleration Support

This document describes the quaternion and linear acceleration support added to the ICM-20948 driver.

# ICM20948 Quaternion and Linear Acceleration Support

This document describes the quaternion and linear acceleration support added to the ICM-20948 driver.

## Overview

The ICM-20948 driver now supports four quaternion sensor types through the InvenSense eMD library:

1. **Game Rotation Vector (6-axis quaternion)** - Uses accelerometer and gyroscope
2. **Rotation Vector (9-axis quaternion)** - Uses accelerometer, gyroscope, and magnetometer  
3. **Geomagnetic Rotation Vector (6-axis quaternion)** - Uses accelerometer and magnetometer
4. **Linear Acceleration** - Acceleration with gravity removed

## Sensor Channels

### Standard Zephyr Channels

- `SENSOR_CHAN_GAME_ROTATION_VECTOR` - 6-axis quaternion (accelerometer + gyroscope) with packed accuracy

### Driver-Specific Channels

- `SENSOR_CHAN_ICM20948_ROTATION_VECTOR` - 9-axis quaternion (accelerometer + gyroscope + magnetometer) with packed accuracy
- `SENSOR_CHAN_ICM20948_GEOMAGNETIC_ROTATION_VECTOR` - Geomagnetic quaternion (accelerometer + magnetometer) with packed accuracy
- `SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION` - Linear acceleration with gravity removed

### Individual Accuracy Channels (if needed separately)
- `SENSOR_CHAN_ICM20948_ACCEL_ACCURACY` - Accelerometer accuracy (0-3)
- `SENSOR_CHAN_ICM20948_GYRO_ACCURACY` - Gyroscope accuracy (0-3)  
- `SENSOR_CHAN_ICM20948_MAGN_ACCURACY` - Magnetometer accuracy (0-3)

## Data Formats

### Quaternions with Packed Accuracy
**NEW**: All quaternion channels now return 5 values: w,x,y,z,accuracy

```c
struct sensor_value quat_with_accuracy[5];
ret = sensor_channel_get(dev, SENSOR_CHAN_GAME_ROTATION_VECTOR, quat_with_accuracy);

// quat_with_accuracy[0] = w component (real part)
// quat_with_accuracy[1] = x component  
// quat_with_accuracy[2] = y component
// quat_with_accuracy[3] = z component
// quat_with_accuracy[4] = accuracy value
```

### Accuracy Formats
- **Game Rotation Vector**: Accuracy is minimum of accelerometer and gyroscope accuracy (0-3 scale)
- **9-axis Rotation Vector**: Accuracy is normalized 0.0-1.0 (converted from eMD Q29 format)
- **Geomagnetic Rotation Vector**: Accuracy is normalized 0.0-1.0 (converted from eMD Q29 format)

Accuracy meanings:
- **0-3 scale**: 0=unreliable, 1=low accuracy, 2=medium accuracy, 3=high accuracy
- **0.0-1.0 scale**: 0.0=unreliable, 1.0=highest accuracy

The quaternions are unit quaternions (normalized) with values in the range [-1, 1].

### Linear Acceleration
Linear acceleration returns 3 values in x,y,z format:
```c
struct sensor_value linear_accel[3];
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION, linear_accel);

// linear_accel[0] = x component in m/s²
// linear_accel[1] = y component in m/s²  
// linear_accel[2] = z component in m/s²
```

## Enabling Sensors

Use the `SENSOR_ATTR_ICM20948_SENSOR_ENABLE` attribute with appropriate sensor masks:

```c
struct sensor_value val;

// Enable accelerometer + 6-axis quaternion + 9-axis quaternion + linear acceleration
val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) |
           BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) |
           BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR) |
           BIT(INV_ICM20948_SENSOR_LINEAR_ACCELERATION);
val.val2 = 0;

ret = sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
```

## eMD Library Integration

The quaternion and linear acceleration data is computed by the Digital Motion Processor (DMP) in the ICM-20948 and provided through the InvenSense eMD library callbacks:

- **INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR** → quat6_raw[4]
- **INV_ICM20948_SENSOR_ROTATION_VECTOR** → quat9_raw[4]  
- **INV_ICM20948_SENSOR_LINEAR_ACCELERATION** → linear_accel_raw[3]

## Data Conversion

The eMD library provides quaternion data in Android format (x,y,z,w) which is converted to standard w,x,y,z format:

```c
/* Android quaternion format: w,x,y,z but data comes as x,y,z,w */
drv_data->quat6_raw[0] = quat6_data[3]; /* w */
drv_data->quat6_raw[1] = quat6_data[0]; /* x */
drv_data->quat6_raw[2] = quat6_data[1]; /* y */
drv_data->quat6_raw[3] = quat6_data[2]; /* z */
```

Linear acceleration is provided directly in m/s² units.

## Usage Examples

### Reading Quaternions with Packed Accuracy

```c
struct sensor_value quat_data[5]; /* w,x,y,z,accuracy */

// Read 6-axis quaternion (Game Rotation Vector) with accuracy
ret = sensor_channel_get(dev, SENSOR_CHAN_GAME_ROTATION_VECTOR, quat_data);
if (ret == 0) {
    LOG_INF("Game Rotation Vector:");
    LOG_INF("  Quaternion: w=%.3f x=%.3f y=%.3f z=%.3f",
        sensor_value_to_double(&quat_data[0]),
        sensor_value_to_double(&quat_data[1]),
        sensor_value_to_double(&quat_data[2]),
        sensor_value_to_double(&quat_data[3]));
    LOG_INF("  Accuracy: %d (0=unreliable, 3=high)", quat_data[4].val1);
}

// Read 9-axis quaternion (Rotation Vector) with accuracy  
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_ROTATION_VECTOR, quat_data);
if (ret == 0) {
    LOG_INF("9-axis Rotation Vector:");
    LOG_INF("  Quaternion: w=%.3f x=%.3f y=%.3f z=%.3f",
        sensor_value_to_double(&quat_data[0]),
        sensor_value_to_double(&quat_data[1]),
        sensor_value_to_double(&quat_data[2]),
        sensor_value_to_double(&quat_data[3]));
    LOG_INF("  Accuracy: %.3f (0.0=unreliable, 1.0=high)",
        sensor_value_to_double(&quat_data[4]));
}

// Read Geomagnetic Rotation Vector with accuracy
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_GEOMAGNETIC_ROTATION_VECTOR, quat_data);
if (ret == 0) {
    LOG_INF("Geomagnetic Rotation Vector:");
    LOG_INF("  Quaternion: w=%.3f x=%.3f y=%.3f z=%.3f",
        sensor_value_to_double(&quat_data[0]),
        sensor_value_to_double(&quat_data[1]),
        sensor_value_to_double(&quat_data[2]),
        sensor_value_to_double(&quat_data[3]));
    LOG_INF("  Accuracy: %.3f (0.0=unreliable, 1.0=high)",
        sensor_value_to_double(&quat_data[4]));
}
```

### Reading Individual Sensor Accuracy (if needed separately)

```c
struct sensor_value accuracy;

// Get accelerometer accuracy
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_ACCEL_ACCURACY, &accuracy);
if (ret == 0) {
    LOG_INF("Accelerometer accuracy: %d/3", accuracy.val1);
}

// Get gyroscope accuracy
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_GYRO_ACCURACY, &accuracy);
if (ret == 0) {
    LOG_INF("Gyroscope accuracy: %d/3", accuracy.val1);
}

// Get magnetometer accuracy
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_MAGN_ACCURACY, &accuracy);
if (ret == 0) {
    LOG_INF("Magnetometer accuracy: %d/3", accuracy.val1);
}
```
}
```

### Reading Accuracy Data
```c
/* Read individual sensor accuracies (0-3 scale) */
struct sensor_value accuracy;

ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_ACCEL_ACCURACY, &accuracy);
if (ret == 0) {
    printk("Accelerometer accuracy: %d (0=unreliable, 3=high)\n", accuracy.val1);
}

ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_GYRO_ACCURACY, &accuracy);
if (ret == 0) {
    printk("Gyroscope accuracy: %d\n", accuracy.val1);
}

/* Read quaternion accuracies */
ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_QUAT6_ACCURACY, &accuracy);
if (ret == 0) {
    printk("Game rotation vector accuracy: %d\n", accuracy.val1);
}

ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_QUAT9_ACCURACY, &accuracy);
if (ret == 0) {
    float acc_float = sensor_value_to_double(&accuracy);
    printk("Rotation vector accuracy: %.3f (0.0-1.0)\n", acc_float);
}
```

## Sample Usage

See `quaternion_demo.c` for quaternion and linear acceleration examples.
See `accuracy_demo.c` for a comprehensive accuracy monitoring demonstration.

Both examples show:
- Enabling quaternion and linear acceleration sensors
- Reading quaternion, linear acceleration, and accuracy data
- Proper sensor lifecycle management

## Notes

1. **DMP Initialization**: The DMP needs time to stabilize, so initial readings may be zero
2. **Accuracy Improvement**: Accuracy typically improves over time as sensors calibrate and stabilize
3. **Magnetometer Calibration**: Movement helps magnetometer calibration for improved 9-axis accuracy
4. **Sensor Dependencies**: Linear acceleration and 9-axis quaternion require magnetometer calibration
5. **Performance**: Quaternion computation is done in hardware by the DMP, providing efficient orientation tracking
6. **Mounting Matrix**: The eMD library automatically applies mounting matrix transformations

### Accuracy Behavior

Accuracy values are provided by the eMD library's sensor fusion algorithms and represent the calibration state of the sensors:

**Individual Sensor Accuracy (0-3):**
- Retrieved from eMD functions: `inv_icm20948_get_accel_accuracy()`, `inv_icm20948_get_gyro_accuracy()`, `inv_icm20948_get_mag_accuracy()`
- Values improve as sensors calibrate during motion
- Accelerometer and gyroscope typically reach high accuracy quickly
- Magnetometer requires figure-8 motion patterns for calibration

**Quaternion Accuracy:**
- **Game Rotation Vector (6-axis)**: Minimum of accelerometer and gyroscope accuracy
- **Rotation Vector (9-axis)**: Normalized accuracy from eMD Q29 format, includes magnetometer influence
- **Geomagnetic Rotation Vector**: Similar to 9-axis but optimized for magnetic heading

**Improving Accuracy:**
- Move the device in various orientations 
- For magnetometer calibration, perform slow figure-8 motions
- Accuracy typically improves within 10-30 seconds of motion
- Higher accuracy values indicate more reliable quaternion data
