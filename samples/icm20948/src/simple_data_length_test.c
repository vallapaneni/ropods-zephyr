/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Simple Data Length Attribute Test
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"

LOG_MODULE_REGISTER(data_length_test, LOG_LEVEL_INF);

#define ICM20948_NODE DT_NODELABEL(icm20948)

#if !DT_NODE_EXISTS(ICM20948_NODE)
#error "ICM20948 devicetree node not found. Check your board configuration."
#endif

static const struct device *icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);

int main(void)
{
	struct sensor_value data_length;
	int ret;

	LOG_INF("ICM20948 Data Length Attribute Test");

	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -1;
	}

	/* Test various channels */
	const struct {
		enum sensor_channel chan;
		const char *name;
		int expected_length;
	} test_channels[] = {
		{SENSOR_CHAN_ACCEL_XYZ, "ACCEL_XYZ", 3},
		{SENSOR_CHAN_ACCEL_X, "ACCEL_X", 1},
		{SENSOR_CHAN_GAME_ROTATION_VECTOR, "GAME_ROTATION_VECTOR", 5},
		{SENSOR_CHAN_ICM20948_ROTATION_VECTOR, "ICM20948_ROTATION_VECTOR", 5},
		{SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION, "ICM20948_LINEAR_ACCELERATION", 4},
	};

	LOG_INF("Testing data length attribute for various channels:");
	LOG_INF("---------------------------------------------");

	for (int i = 0; i < ARRAY_SIZE(test_channels); i++) {
		ret = sensor_attr_get(icm20948_dev, test_channels[i].chan, 
				      SENSOR_ATTR_DATA_LENGTH, &data_length);
		if (ret) {
			LOG_ERR("Failed to get data length for %s: %d", 
				test_channels[i].name, ret);
		} else {
			bool correct = (data_length.val1 == test_channels[i].expected_length);
			LOG_INF("%-30s: %d values %s", 
				test_channels[i].name, 
				data_length.val1,
				correct ? "✓" : "✗");
			
			if (!correct) {
				LOG_ERR("Expected %d, got %d", 
					test_channels[i].expected_length, 
					data_length.val1);
			}
		}
	}

	/* Test read-only behavior */
	LOG_INF("");
	LOG_INF("Testing read-only behavior:");
	data_length.val1 = 99;
	data_length.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ACCEL_XYZ, 
			      SENSOR_ATTR_DATA_LENGTH, &data_length);
	if (ret == -EACCES) {
		LOG_INF("✓ Correctly rejected attempt to set data length (read-only)");
	} else {
		LOG_ERR("✗ Should have rejected attempt to set data length: %d", ret);
	}

	LOG_INF("");
	LOG_INF("Data length attribute test completed!");
	return 0;
}
