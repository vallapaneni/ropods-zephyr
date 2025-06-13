/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Demo: ICM20948 Data Length Attribute
 * 
 * This demo shows how to use the SENSOR_ATTR_ICM20948_DATA_LENGTH attribute
 * to query how many sensor_value elements are needed for different channels.
 * This is especially useful with packed accuracy channels that return
 * variable numbers of values.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"

LOG_MODULE_REGISTER(icm20948_data_length_demo, LOG_LEVEL_INF);

#define ICM20948_NODE DT_NODELABEL(icm20948)

#if !DT_NODE_EXISTS(ICM20948_NODE)
#error "ICM20948 devicetree node not found. Check your board configuration."
#endif

static const struct device *icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);

/* Helper function to get and display data length for a channel */
static int show_channel_data_length(enum sensor_channel chan, const char *channel_name)
{
	struct sensor_value data_length;
	int ret;

	ret = sensor_attr_get(icm20948_dev, chan, SENSOR_ATTR_DATA_LENGTH, &data_length);
	if (ret) {
		LOG_ERR("Failed to get data length for %s: %d", channel_name, ret);
		return ret;
	}

	LOG_INF("%-35s: %d values", channel_name, data_length.val1);
	return 0;
}

/* Dynamic allocation example using data length */
static int dynamic_read_example(enum sensor_channel chan, const char *channel_name)
{
	struct sensor_value data_length;
	struct sensor_value *data;
	int ret;

	/* Get the required data length */
	ret = sensor_attr_get(icm20948_dev, chan, SENSOR_ATTR_DATA_LENGTH, &data_length);
	if (ret) {
		LOG_ERR("Failed to get data length for %s: %d", channel_name, ret);
		return ret;
	}

	/* Allocate array of the correct size */
	data = k_malloc(data_length.val1 * sizeof(struct sensor_value));
	if (!data) {
		LOG_ERR("Failed to allocate memory for %s data", channel_name);
		return -ENOMEM;
	}

	LOG_INF("Allocated %d values for %s", data_length.val1, channel_name);

	/* Enable required sensors for testing */
	struct sensor_value enable_val;
	enable_val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | 
			  BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) |
			  BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR) |
			  BIT(INV_ICM20948_SENSOR_LINEAR_ACCELERATION);
	enable_val.val2 = 0;
	sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &enable_val);

	/* Fetch data */
	ret = sensor_sample_fetch(icm20948_dev);
	if (ret) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
		k_free(data);
		return ret;
	}

	/* Get channel data */
	ret = sensor_channel_get(icm20948_dev, chan, data);
	if (ret) {
		LOG_ERR("Failed to get %s data: %d", channel_name, ret);
		k_free(data);
		return ret;
	}

	/* Display the data */
	LOG_INF("%s data:", channel_name);
	for (int i = 0; i < data_length.val1; i++) {
		LOG_INF("  [%d]: %d.%06d", i, data[i].val1, abs(data[i].val2));
	}

	k_free(data);
	return 0;
}

static int test_data_length_attribute(void)
{
	LOG_INF("=== ICM20948 Data Length Attribute Demo ===");
	LOG_INF("");

	/* Show data lengths for all supported channels */
	LOG_INF("Channel data lengths:");
	LOG_INF("-------------------------------------");
	
	/* Standard 3-axis channels */
	show_channel_data_length(SENSOR_CHAN_ACCEL_XYZ, "SENSOR_CHAN_ACCEL_XYZ");
	show_channel_data_length(SENSOR_CHAN_GYRO_XYZ, "SENSOR_CHAN_GYRO_XYZ");
	show_channel_data_length(SENSOR_CHAN_MAGN_XYZ, "SENSOR_CHAN_MAGN_XYZ");
	
	/* Single-axis channels */
	show_channel_data_length(SENSOR_CHAN_ACCEL_X, "SENSOR_CHAN_ACCEL_X");
	show_channel_data_length(SENSOR_CHAN_GYRO_Y, "SENSOR_CHAN_GYRO_Y");
	show_channel_data_length(SENSOR_CHAN_MAGN_Z, "SENSOR_CHAN_MAGN_Z");
	show_channel_data_length(SENSOR_CHAN_DIE_TEMP, "SENSOR_CHAN_DIE_TEMP");
	
	/* Packed accuracy channels */
	show_channel_data_length(SENSOR_CHAN_GAME_ROTATION_VECTOR, "SENSOR_CHAN_GAME_ROTATION_VECTOR");
	show_channel_data_length(SENSOR_CHAN_ICM20948_ROTATION_VECTOR, "SENSOR_CHAN_ICM20948_ROTATION_VECTOR");
	show_channel_data_length(SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION, "SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION");

	LOG_INF("");
	LOG_INF("=== Dynamic Allocation Examples ===");
	LOG_INF("");

	/* Dynamic allocation examples */
	dynamic_read_example(SENSOR_CHAN_ACCEL_XYZ, "Accelerometer XYZ");
	k_sleep(K_MSEC(100));
	
	dynamic_read_example(SENSOR_CHAN_GAME_ROTATION_VECTOR, "Game Rotation Vector");
	k_sleep(K_MSEC(100));
	
	dynamic_read_example(SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION, "Linear Acceleration");

	/* Disable sensors after demo */
	struct sensor_value disable_val = {0, 0};
	sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &disable_val);

	return 0;
}

/* Generic helper function for any channel */
static struct sensor_value* allocate_channel_data(enum sensor_channel chan)
{
	struct sensor_value data_length;
	struct sensor_value *data;
	int ret;

	ret = sensor_attr_get(icm20948_dev, chan, SENSOR_ATTR_DATA_LENGTH, &data_length);
	if (ret) {
		LOG_ERR("Failed to get data length for channel %d: %d", chan, ret);
		return NULL;
	}

	data = k_malloc(data_length.val1 * sizeof(struct sensor_value));
	if (!data) {
		LOG_ERR("Failed to allocate memory for channel %d", chan);
		return NULL;
	}

	LOG_DBG("Allocated %d values for channel %d", data_length.val1, chan);
	return data;
}

static void show_usage_patterns(void)
{
	LOG_INF("=== Usage Patterns ===");
	LOG_INF("");
	LOG_INF("1. Static allocation with known size:");
	LOG_INF("   struct sensor_value quat_data[5];  // For quaternion + accuracy");
	LOG_INF("   sensor_channel_get(dev, SENSOR_CHAN_GAME_ROTATION_VECTOR, quat_data);");
	LOG_INF("");
	LOG_INF("2. Dynamic allocation based on data length:");
	LOG_INF("   struct sensor_value length;");
	LOG_INF("   sensor_attr_get(dev, chan, SENSOR_ATTR_DATA_LENGTH, &length);");
	LOG_INF("   struct sensor_value *data = k_malloc(length.val1 * sizeof(struct sensor_value));");
	LOG_INF("   sensor_channel_get(dev, chan, data);");
	LOG_INF("");
	LOG_INF("3. Generic channel reader function:");
	LOG_INF("   struct sensor_value *data = allocate_channel_data(chan);");
	LOG_INF("   sensor_channel_get(dev, chan, data);");
	LOG_INF("   k_free(data);");
}

int main(void)
{
	LOG_INF("ICM20948 Data Length Attribute Demo Starting...");

	/* Check if device is ready */
	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -1;
	}

	LOG_INF("ICM20948 device is ready");
	LOG_INF("");

	/* Run the demo */
	int ret = test_data_length_attribute();
	if (ret) {
		LOG_ERR("Demo failed with error: %d", ret);
		return ret;
	}

	LOG_INF("");
	show_usage_patterns();

	LOG_INF("");
	LOG_INF("Demo completed successfully!");
	return 0;
}
