/*
 * Copyright (c) 2024 ROPODS
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

/* Include eMD library headers for sensor enums */
#include "Invn/Devices/Drivers/ICM20948/Icm20948Setup.h"

/* Include ICM20948 driver header for attributes */
#include "icm20948.h"

LOG_MODULE_REGISTER(sensor_enable_demo, LOG_LEVEL_INF);

#define ICM20948_NODE DT_NODELABEL(icm20948)

static const struct device *const icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);

static void print_sensor_data(const struct device *dev)
{
	struct sensor_value accel[3], gyro[3], mag[3], temp;
	int ret;

	/* Fetch sensor data */
	ret = sensor_sample_fetch(dev);
	if (ret) {
		LOG_ERR("Failed to fetch sample: %d", ret);
		return;
	}

	/* Get accelerometer data (always available when enabled) */
	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret == 0) {
		LOG_INF("Accel: X=%d.%06d Y=%d.%06d Z=%d.%06d m/s²",
			accel[0].val1, abs(accel[0].val2),
			accel[1].val1, abs(accel[1].val2),
			accel[2].val1, abs(accel[2].val2));
	} else {
		LOG_WRN("Failed to get accelerometer data: %d", ret);
	}

	/* Get gyroscope data (underlying gyro used by rotation vectors) */
	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (ret == 0) {
		LOG_INF("Gyro:  X=%d.%06d Y=%d.%06d Z=%d.%06d rad/s",
			gyro[0].val1, abs(gyro[0].val2),
			gyro[1].val1, abs(gyro[1].val2),
			gyro[2].val1, abs(gyro[2].val2));
	} else {
		LOG_WRN("Failed to get gyroscope data: %d", ret);
	}

	/* Note: Rotation vector data (INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, INV_ICM20948_SENSOR_ROTATION_VECTOR)
	 * is provided through eMD library callbacks, not through standard Zephyr sensor channels.
	 * These are quaternion-based orientation representations computed by the DMP. */

	/* Get magnetometer data (if available) */
	ret = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);
	if (ret == 0) {
		LOG_INF("Mag:   X=%d.%06d Y=%d.%06d Z=%d.%06d µT",
			mag[0].val1, abs(mag[0].val2),
			mag[1].val1, abs(mag[1].val2),
			mag[2].val1, abs(mag[2].val2));
	} else {
		LOG_DBG("Magnetometer data not available: %d", ret);
	}

	/* Get temperature data */
	ret = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);
	if (ret == 0) {
		LOG_INF("Temp:  %d.%06d °C", temp.val1, abs(temp.val2));
	} else {
		LOG_WRN("Failed to get temperature data: %d", ret);
	}
}

static int set_sensor_enable_mask(const struct device *dev, uint32_t mask)
{
	struct sensor_value val;
	int ret;

	val.val1 = (int32_t)mask;
	val.val2 = 0;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to set sensor enable mask 0x%08X: %d", mask, ret);
		return ret;
	}

	LOG_INF("Sensor enable mask set to 0x%08X", mask);
	return 0;
}

static int get_sensor_enable_mask(const struct device *dev)
{
	struct sensor_value val;
	int ret;

	ret = sensor_attr_get(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get sensor enable mask: %d", ret);
		return ret;
	}

	LOG_INF("Current sensor enable mask: 0x%08X", (uint32_t)val.val1);
	return val.val1;
}

int main(void)
{
	int ret;
	uint32_t sensor_mask;

	LOG_INF("=================================================");
	LOG_INF("ICM20948 Sensor Enable Demonstration");
	LOG_INF("=================================================");

	/* Check if device is ready */
	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -ENODEV;
	}

	LOG_INF("✓ ICM20948 device ready");

	/* Test 1: Check initial state (should be all disabled) */
	LOG_INF("Test 1: Checking initial sensor enable state...");
	ret = get_sensor_enable_mask(icm20948_dev);
	if (ret < 0) {
		return ret;
	}
	LOG_INF("✓ Initial state checked");

	/* Test 2: Try to read sensors when disabled (should fail or return zero data) */
	LOG_INF("Test 2: Attempting to read sensors when disabled...");
	print_sensor_data(icm20948_dev);

	/* Test 3: Enable only accelerometer */
	LOG_INF("Test 3: Enabling only accelerometer...");
	sensor_mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER);
	ret = set_sensor_enable_mask(icm20948_dev, sensor_mask);
	if (ret) {
		return ret;
	}
	k_sleep(K_MSEC(100)); /* Give sensor time to stabilize */
	
	LOG_INF("Reading sensor data with only accelerometer enabled:");
	print_sensor_data(icm20948_dev);

	/* Test 4: Enable accelerometer and game rotation vector */
	LOG_INF("Test 4: Enabling accelerometer and game rotation vector...");
	/* Note: The driver efficiently manages state transitions by only changing sensors that need to change */
	sensor_mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
	ret = set_sensor_enable_mask(icm20948_dev, sensor_mask);
	if (ret) {
		return ret;
	}
	k_sleep(K_MSEC(100)); /* Give sensor time to stabilize */
	
	LOG_INF("Reading sensor data with accelerometer and game rotation vector enabled:");
	print_sensor_data(icm20948_dev);

	/* Test 5: Enable all supported sensors */
	LOG_INF("Test 5: Enabling all supported sensors...");
	/* The driver will enable the rotation vector sensor while keeping accelerometer and game rotation vector enabled */
	sensor_mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | 
	              BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) |
	              BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR);
	ret = set_sensor_enable_mask(icm20948_dev, sensor_mask);
	if (ret) {
		return ret;
	}
	k_sleep(K_MSEC(100)); /* Give sensor time to stabilize */
	
	LOG_INF("Reading sensor data with all supported sensors enabled:");
	print_sensor_data(icm20948_dev);

	/* Test 6: Switch to just rotation vector */
	LOG_INF("Test 6: Switching to only rotation vector...");
	/* The driver will disable accelerometer and game rotation vector while keeping rotation vector enabled */
	sensor_mask = BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR);
	ret = set_sensor_enable_mask(icm20948_dev, sensor_mask);
	if (ret) {
		return ret;
	}
	k_sleep(K_MSEC(100)); /* Give sensor time to stabilize */
	
	LOG_INF("Reading sensor data with only rotation vector enabled:");
	print_sensor_data(icm20948_dev);

	/* Test 7: Disable all sensors */
	LOG_INF("Test 7: Disabling all sensors...");
	ret = set_sensor_enable_mask(icm20948_dev, 0);
	if (ret) {
		return ret;
	}
	k_sleep(K_MSEC(100));
	
	LOG_INF("Reading sensor data with all sensors disabled:");
	print_sensor_data(icm20948_dev);

	/* Test 8: Continuous reading with all supported sensors enabled */
	LOG_INF("Test 8: Continuous reading with all supported sensors enabled...");
	sensor_mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | 
	              BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) |
	              BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR);
	ret = set_sensor_enable_mask(icm20948_dev, sensor_mask);
	if (ret) {
		return ret;
	}

	LOG_INF("Starting continuous sensor reading (Ctrl+C to stop)...");
	for (int i = 0; i < 10; i++) {
		LOG_INF("--- Reading %d ---", i + 1);
		print_sensor_data(icm20948_dev);
		k_sleep(K_SECONDS(2));
	}

	LOG_INF("=================================================");
	LOG_INF("ICM20948 Sensor Enable Demo completed successfully!");
	LOG_INF("=================================================");

	return 0;
}
