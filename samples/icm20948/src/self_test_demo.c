/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * ICM20948 Self-Test Demonstration
 * This example shows how to use the new self-test attribute
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"

LOG_MODULE_REGISTER(icm20948_self_test_demo, LOG_LEVEL_INF);

#define ICM20948_NODE DT_NODELABEL(icm20948)

#if !DT_NODE_EXISTS(ICM20948_NODE)
#error "ICM20948 devicetree node not found. Check your board configuration."
#endif

static const struct device *icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);

/**
 * @brief Run ICM20948 self-test and report results
 * 
 * @param dev ICM20948 device pointer
 * @return 0 on success, negative error code on failure
 */
static int run_icm20948_self_test(const struct device *dev)
{
    struct sensor_value trigger_val = {.val1 = 1, .val2 = 0}; // Any non-zero value triggers test
    int ret;
    
    LOG_INF("Starting ICM20948 hardware self-test...");
    LOG_INF("This will test accelerometer, gyroscope, and magnetometer functionality");
    
    /* Trigger self-test on all sensors */
    ret = sensor_attr_set(dev, 
                         SENSOR_CHAN_ALL,  // Channel ignored for self-test
                         SENSOR_ATTR_ICM20948_SELF_TEST, 
                         &trigger_val);
    
    if (ret == 0) {
        LOG_INF("✓ ICM20948 self-test PASSED - all sensors functioning correctly");
        return 0;
    } else {
        LOG_ERR("✗ ICM20948 self-test FAILED with error: %d", ret);
        
        /* Provide detailed error interpretation */
        switch (ret) {
        case -EINVAL:
            LOG_ERR("  → Invalid arguments (sensor configuration issue)");
            break;
        case -ETIMEDOUT:
            LOG_ERR("  → Self-test timed out (sensor may be unresponsive)");
            break;
        case -EIO:
            LOG_ERR("  → Communication error (check SPI/I2C connection)");
            break;
        case -ENODEV:
            LOG_ERR("  → Hardware failure (sensor may be defective)");
            break;
        default:
            LOG_ERR("  → Unknown error (check system configuration)");
            break;
        }
        
        return ret;
    }
}

/**
 * @brief Test invalid self-test usage (zero trigger value)
 * 
 * @param dev ICM20948 device pointer
 * @return 0 on success
 */
static int test_self_test_zero_trigger(const struct device *dev)
{
    struct sensor_value zero_val = {.val1 = 0, .val2 = 0}; // Zero value should be ignored
    int ret;
    
    LOG_INF("Testing self-test with zero trigger value (should be ignored)...");
    
    ret = sensor_attr_set(dev, 
                         SENSOR_CHAN_ALL,
                         SENSOR_ATTR_ICM20948_SELF_TEST, 
                         &zero_val);
    
    if (ret == 0) {
        LOG_INF("✓ Zero trigger value correctly ignored (no self-test executed)");
        return 0;
    } else {
        LOG_ERR("✗ Unexpected error with zero trigger: %d", ret);
        return ret;
    }
}

/**
 * @brief Test trying to read (get) the write-only self-test attribute
 * 
 * @param dev ICM20948 device pointer
 * @return 0 on expected error, negative on unexpected behavior
 */
static int test_self_test_read_attempt(const struct device *dev)
{
    struct sensor_value read_val;
    int ret;
    
    LOG_INF("Testing read attempt on write-only self-test attribute...");
    
    ret = sensor_attr_get(dev, 
                         SENSOR_CHAN_ALL,
                         SENSOR_ATTR_ICM20948_SELF_TEST, 
                         &read_val);
    
    if (ret == -EACCES) {
        LOG_INF("✓ Read attempt correctly rejected with -EACCES (write-only attribute)");
        return 0;
    } else {
        LOG_ERR("✗ Unexpected return code: %d (expected -EACCES)", ret);
        return -1;
    }
}

/**
 * @brief Perform basic sensor reading test after self-test
 * 
 * @param dev ICM20948 device pointer
 * @return 0 on success, negative error code on failure
 */
static int test_sensor_readings_after_self_test(const struct device *dev)
{
    struct sensor_value accel[3], gyro[3], mag[3];
    int ret;
    
    LOG_INF("Testing normal sensor operation after self-test...");
    
    /* Fetch fresh sensor data */
    ret = sensor_sample_fetch(dev);
    if (ret) {
        LOG_ERR("Failed to fetch sensor data: %d", ret);
        return ret;
    }
    
    /* Read accelerometer data */
    ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (ret) {
        LOG_ERR("Failed to get accelerometer data: %d", ret);
        return ret;
    }
    
    /* Read gyroscope data */
    ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (ret) {
        LOG_ERR("Failed to get gyroscope data: %d", ret);
        return ret;
    }
    
    /* Read magnetometer data */
    ret = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);
    if (ret) {
        LOG_ERR("Failed to get magnetometer data: %d", ret);
        return ret;
    }
    
    LOG_INF("✓ Normal sensor readings after self-test:");
    LOG_INF("  Accel: X=%d.%06d, Y=%d.%06d, Z=%d.%06d m/s²", 
            accel[0].val1, accel[0].val2, 
            accel[1].val1, accel[1].val2, 
            accel[2].val1, accel[2].val2);
    LOG_INF("  Gyro:  X=%d.%06d, Y=%d.%06d, Z=%d.%06d rad/s", 
            gyro[0].val1, gyro[0].val2, 
            gyro[1].val1, gyro[1].val2, 
            gyro[2].val1, gyro[2].val2);
    LOG_INF("  Mag:   X=%d.%06d, Y=%d.%06d, Z=%d.%06d µT", 
            mag[0].val1, mag[0].val2, 
            mag[1].val1, mag[1].val2, 
            mag[2].val1, mag[2].val2);
    
    return 0;
}

int main(void)
{
    int ret;
    
    LOG_INF("=================================================");
    LOG_INF("ICM20948 Self-Test Demonstration");
    LOG_INF("=================================================");

    /* Check if device is ready */
    if (!device_is_ready(icm20948_dev)) {
        LOG_ERR("ICM20948 device not ready");
        return -1;
    }

    LOG_INF("ICM20948 device is ready");

    /* Test 1: Run actual self-test */
    LOG_INF("\n--- Test 1: Hardware Self-Test ---");
    ret = run_icm20948_self_test(icm20948_dev);
    if (ret) {
        LOG_ERR("Self-test failed, aborting further tests");
        return ret;
    }

    /* Test 2: Test zero trigger value */
    LOG_INF("\n--- Test 2: Zero Trigger Value ---");
    ret = test_self_test_zero_trigger(icm20948_dev);
    if (ret) {
        LOG_ERR("Zero trigger test failed");
        return ret;
    }

    /* Test 3: Test read attempt on write-only attribute */
    LOG_INF("\n--- Test 3: Read Attempt Test ---");
    ret = test_self_test_read_attempt(icm20948_dev);
    if (ret) {
        LOG_ERR("Read attempt test failed");
        return ret;
    }

    /* Test 4: Verify normal operation after self-test */
    LOG_INF("\n--- Test 4: Normal Operation Verification ---");
    ret = test_sensor_readings_after_self_test(icm20948_dev);
    if (ret) {
        LOG_ERR("Post-self-test sensor reading failed");
        return ret;
    }

    LOG_INF("\n=================================================");
    LOG_INF("✓ All self-test demonstration tests PASSED!");
    LOG_INF("✓ ICM20948 self-test attribute is working correctly");
    LOG_INF("=================================================");

    /* Continuous loop for ongoing monitoring if desired */
    LOG_INF("\nDemo completed. You can now use the self-test attribute in your applications.");
    LOG_INF("To trigger self-test: sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SELF_TEST, &non_zero_val);");

    return 0;
}
