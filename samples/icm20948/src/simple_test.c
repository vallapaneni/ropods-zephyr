/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Simple test for ICM20948 sensor enable functionality
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"

LOG_MODULE_REGISTER(icm20948_simple_test, LOG_LEVEL_INF);

#define ICM20948_NODE DT_NODELABEL(icm20948)

#if !DT_NODE_EXISTS(ICM20948_NODE)
#error "ICM20948 devicetree node not found. Check your board configuration."
#endif

static const struct device *icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);

static void dummy_trigger_handler(const struct device *dev,
				  const struct sensor_trigger *trigger)
{
	LOG_INF("Dummy trigger handler called");
}

static int test_sensor_enable_control(void)
{
	struct sensor_trigger trigger = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	struct sensor_value val;
	int ret;

	LOG_INF("=== Testing ICM20948 Sensor Enable Control ===");

	/* Test 1: Set trigger handler */
	LOG_INF("Test 1: Setting trigger handler...");
	ret = sensor_trigger_set(icm20948_dev, &trigger, dummy_trigger_handler);
	if (ret) {
		LOG_ERR("Failed to set trigger handler: %d", ret);
		return ret;
	}
	LOG_INF("✓ Trigger handler set successfully");

	/* Test 2: Check initial sensor enable state (should be disabled) */
	LOG_INF("Test 2: Checking initial sensor enable state...");
	ret = sensor_attr_get(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get sensor enable state: %d", ret);
		return ret;
	}
	LOG_INF("✓ Initial sensor enable state: 0x%08x", (uint32_t)val.val1);

	/* Test 3: Enable accelerometer only */
	LOG_INF("Test 3: Enabling accelerometer only...");
	val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER);
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable accelerometer: %d", ret);
		return ret;
	}
	LOG_INF("✓ Accelerometer enabled successfully");

	/* Test 4: Verify sensor enable state */
	LOG_INF("Test 4: Verifying sensor enable state...");
	ret = sensor_attr_get(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get sensor enable state: %d", ret);
		return ret;
	}
	if ((uint32_t)val.val1 != BIT(INV_ICM20948_SENSOR_ACCELEROMETER)) {
		LOG_ERR("Sensor enable state verification failed: expected 0x%08x, got 0x%08x", 
			BIT(INV_ICM20948_SENSOR_ACCELEROMETER), (uint32_t)val.val1);
		return -1;
	}
	LOG_INF("✓ Sensor enable state verified: 0x%08x", (uint32_t)val.val1);

	/* Test 5: Enable accelerometer and game rotation vector */
	LOG_INF("Test 5: Enabling accelerometer and game rotation vector...");
	val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable accelerometer and game rotation vector: %d", ret);
		return ret;
	}
	LOG_INF("✓ Accelerometer and game rotation vector enabled successfully");

	/* Test 6: Verify sensor enable state */
	LOG_INF("Test 6: Verifying sensor enable state...");
	ret = sensor_attr_get(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get sensor enable state: %d", ret);
		return ret;
	}
	uint32_t expected_mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
	if ((uint32_t)val.val1 != expected_mask) {
		LOG_ERR("Sensor enable state verification failed: expected 0x%08x, got 0x%08x", 
			expected_mask, (uint32_t)val.val1);
		return -1;
	}
	LOG_INF("✓ Sensor enable state verified: 0x%08x", (uint32_t)val.val1);

	/* Test 7: Disable all sensors */
	LOG_INF("Test 7: Disabling all sensors...");
	val.val1 = 0;
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to disable all sensors: %d", ret);
		return ret;
	}
	LOG_INF("✓ All sensors disabled successfully");

	/* Test 8: Verify sensor enable state is disabled */
	LOG_INF("Test 8: Verifying all sensors disabled...");
	ret = sensor_attr_get(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get sensor enable state: %d", ret);
		return ret;
	}
	if ((uint32_t)val.val1 != 0) {
		LOG_ERR("Sensor enable state verification failed: expected 0, got 0x%08x", (uint32_t)val.val1);
		return -1;
	}
	LOG_INF("✓ Sensor enable state verified: all disabled");

	/* Test 9: Clear trigger handler */
	LOG_INF("Test 9: Clearing trigger handler...");
	ret = sensor_trigger_set(icm20948_dev, &trigger, NULL);
	if (ret) {
		LOG_ERR("Failed to clear trigger handler: %d", ret);
		return ret;
	}
	LOG_INF("✓ Trigger handler cleared successfully");

	LOG_INF("=== All tests passed! ===");
	return 0;
}

int main(void)
{
	LOG_INF("ICM20948 Simple Test Starting...");

	/* Check if device is ready */
	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -1;
	}

	LOG_INF("ICM20948 device is ready");

	/* Run the sensor enable control tests */
	int ret = test_sensor_enable_control();
	if (ret) {
		LOG_ERR("Test failed with error: %d", ret);
		return -1;
	}

	LOG_INF("Test completed successfully!");

	/* Basic sensor reading test */
	LOG_INF("Testing basic sensor reading...");
	struct sensor_value accel[3];
	
	/* Enable accelerometer for testing */
	struct sensor_value val;
	val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER);
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable accelerometer for test: %d", ret);
	} else {
		/* Wait a moment for sensor to stabilize */
		k_sleep(K_MSEC(100));
		
		ret = sensor_sample_fetch(icm20948_dev);
		if (ret) {
			LOG_ERR("Failed to fetch sensor data: %d", ret);
		} else {
			ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
			if (ret) {
				LOG_ERR("Failed to get accelerometer data: %d", ret);
			} else {
				LOG_INF("Accel: X=%d.%06d Y=%d.%06d Z=%d.%06d m/s²",
					accel[0].val1, abs(accel[0].val2),
					accel[1].val1, abs(accel[1].val2),
					accel[2].val1, abs(accel[2].val2));
			}
		}
		
		/* Disable sensors after test */
		val.val1 = 0;
		val.val2 = 0;
		sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
				SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	}

	return 0;
}
