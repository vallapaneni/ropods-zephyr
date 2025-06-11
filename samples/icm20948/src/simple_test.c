/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Simple test for ICM20948 interrupt control functionality
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

static int test_interrupt_control(void)
{
	struct sensor_trigger trigger = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	struct sensor_value val;
	int ret;

	LOG_INF("=== Testing ICM20948 Interrupt Control ===");

	/* Test 1: Set trigger handler */
	LOG_INF("Test 1: Setting trigger handler...");
	ret = sensor_trigger_set(icm20948_dev, &trigger, dummy_trigger_handler);
	if (ret) {
		LOG_ERR("Failed to set trigger handler: %d", ret);
		return ret;
	}
	LOG_INF("✓ Trigger handler set successfully");

	/* Test 2: Check initial interrupt state (should be disabled) */
	LOG_INF("Test 2: Checking initial interrupt state...");
	ret = sensor_attr_get(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get interrupt state: %d", ret);
		return ret;
	}
	LOG_INF("✓ Initial interrupt state: %s", val.val1 ? "enabled" : "disabled");

	/* Test 3: Enable interrupts */
	LOG_INF("Test 3: Enabling interrupts...");
	val.val1 = 1;
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable interrupts: %d", ret);
		return ret;
	}
	LOG_INF("✓ Interrupts enabled successfully");

	/* Test 4: Verify interrupt state is enabled */
	LOG_INF("Test 4: Verifying interrupt state...");
	ret = sensor_attr_get(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get interrupt state: %d", ret);
		return ret;
	}
	if (val.val1 != 1) {
		LOG_ERR("Interrupt state verification failed: expected 1, got %d", val.val1);
		return -1;
	}
	LOG_INF("✓ Interrupt state verified: enabled");

	/* Test 5: Disable interrupts */
	LOG_INF("Test 5: Disabling interrupts...");
	val.val1 = 0;
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to disable interrupts: %d", ret);
		return ret;
	}
	LOG_INF("✓ Interrupts disabled successfully");

	/* Test 6: Verify interrupt state is disabled */
	LOG_INF("Test 6: Verifying interrupt state...");
	ret = sensor_attr_get(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get interrupt state: %d", ret);
		return ret;
	}
	if (val.val1 != 0) {
		LOG_ERR("Interrupt state verification failed: expected 0, got %d", val.val1);
		return -1;
	}
	LOG_INF("✓ Interrupt state verified: disabled");

	/* Test 7: Clear trigger handler */
	LOG_INF("Test 7: Clearing trigger handler...");
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

	/* Run the interrupt control tests */
	int ret = test_interrupt_control();
	if (ret) {
		LOG_ERR("Test failed with error: %d", ret);
		return -1;
	}

	LOG_INF("Test completed successfully!");

	/* Basic sensor reading test */
	LOG_INF("Testing basic sensor reading...");
	struct sensor_value accel[3];
	
	ret = sensor_sample_fetch(icm20948_dev);
	if (ret) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
	} else {
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (ret) {
			LOG_ERR("Failed to get accelerometer data: %d", ret);
		} else {
			LOG_INF("Accel: X=%.3f Y=%.3f Z=%.3f m/s²",
				sensor_value_to_double(&accel[0]),
				sensor_value_to_double(&accel[1]),
				sensor_value_to_double(&accel[2]));
		}
	}

	return 0;
}
