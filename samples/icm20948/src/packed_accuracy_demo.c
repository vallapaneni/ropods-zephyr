/*
 * Copyright (c) 2024 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(packed_accuracy_demo, LOG_LEVEL_INF);

/* ICM20948 driver specific includes */
#include "../../../drivers/sensor/icm20948/icm20948.h"

/* Device tree node for ICM20948 */
#define ICM20948_NODE DT_ALIAS(icm20948)

/**
 * Demonstrate the new packed accuracy format for quaternion data
 * Each quaternion channel now returns 5 values: w,x,y,z,accuracy
 */
static void demo_packed_quaternion_accuracy(const struct device *dev)
{
	struct sensor_value quat_data[5]; /* w,x,y,z,accuracy */
	int ret;

	LOG_INF("=== Packed Quaternion + Accuracy Demo ===");

	/* Fetch fresh sensor data */
	ret = sensor_sample_fetch(dev);
	if (ret) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
		return;
	}

	/* Demo 1: Game Rotation Vector (6-axis: accel + gyro) */
	ret = sensor_channel_get(dev, SENSOR_CHAN_GAME_ROTATION_VECTOR, quat_data);
	if (ret == 0) {
		LOG_INF("Game Rotation Vector (6-axis):");
		LOG_INF("  Quaternion: w=%d.%06d x=%d.%06d y=%d.%06d z=%d.%06d",
			quat_data[0].val1, abs(quat_data[0].val2),
			quat_data[1].val1, abs(quat_data[1].val2),
			quat_data[2].val1, abs(quat_data[2].val2),
			quat_data[3].val1, abs(quat_data[3].val2));
		LOG_INF("  Accuracy: %d (0-3 scale, min of accel+gyro accuracy)",
			quat_data[4].val1);
	} else {
		LOG_WRN("Failed to get Game Rotation Vector: %d", ret);
	}

	/* Demo 2: 9-axis Rotation Vector (accel + gyro + mag) */
	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_ROTATION_VECTOR, quat_data);
	if (ret == 0) {
		LOG_INF("9-axis Rotation Vector:");
		LOG_INF("  Quaternion: w=%d.%06d x=%d.%06d y=%d.%06d z=%d.%06d",
			quat_data[0].val1, abs(quat_data[0].val2),
			quat_data[1].val1, abs(quat_data[1].val2),
			quat_data[2].val1, abs(quat_data[2].val2),
			quat_data[3].val1, abs(quat_data[3].val2));
		LOG_INF("  Accuracy: %d.%06d (0-1 normalized scale)",
			quat_data[4].val1, abs(quat_data[4].val2));
	} else {
		LOG_WRN("Failed to get 9-axis Rotation Vector: %d", ret);
	}

	/* Demo 3: Geomagnetic Rotation Vector (accel + mag) */
	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_GEOMAGNETIC_ROTATION_VECTOR, quat_data);
	if (ret == 0) {
		LOG_INF("Geomagnetic Rotation Vector (accel + mag):");
		LOG_INF("  Quaternion: w=%d.%06d x=%d.%06d y=%d.%06d z=%d.%06d",
			quat_data[0].val1, abs(quat_data[0].val2),
			quat_data[1].val1, abs(quat_data[1].val2),
			quat_data[2].val1, abs(quat_data[2].val2),
			quat_data[3].val1, abs(quat_data[3].val2));
		LOG_INF("  Accuracy: %d.%06d (0-1 normalized scale)",
			quat_data[4].val1, abs(quat_data[4].val2));
	} else {
		LOG_WRN("Failed to get Geomagnetic Rotation Vector: %d", ret);
	}
}

/**
 * Demonstrate accessing individual accuracy values (if needed separately)
 */
static void demo_individual_accuracy(const struct device *dev)
{
	struct sensor_value accuracy;
	int ret;

	LOG_INF("=== Individual Accuracy Values ===");

	/* Individual accuracy channels are still available if needed */
	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_ACCEL_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Accelerometer Accuracy: %d (0-3 scale)", accuracy.val1);
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_GYRO_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Gyroscope Accuracy: %d (0-3 scale)", accuracy.val1);
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_MAGN_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Magnetometer Accuracy: %d (0-3 scale)", accuracy.val1);
	}
}

/**
 * Convert quaternion to Euler angles for demonstration
 */
static void quaternion_to_euler(const struct sensor_value quat[4], float *roll, float *pitch, float *yaw)
{
	float w = sensor_value_to_double(&quat[0]);
	float x = sensor_value_to_double(&quat[1]);
	float y = sensor_value_to_double(&quat[2]);
	float z = sensor_value_to_double(&quat[3]);

	/* Roll (x-axis rotation) */
	float sinr_cosp = 2 * (w * x + y * z);
	float cosr_cosp = 1 - 2 * (x * x + y * y);
	*roll = atan2f(sinr_cosp, cosr_cosp);

	/* Pitch (y-axis rotation) */
	float sinp = 2 * (w * y - z * x);
	if (fabsf(sinp) >= 1) {
		*pitch = copysignf(M_PI / 2, sinp); /* Use 90 degrees if out of range */
	} else {
		*pitch = asinf(sinp);
	}

	/* Yaw (z-axis rotation) */
	float siny_cosp = 2 * (w * z + x * y);
	float cosy_cosp = 1 - 2 * (y * y + z * z);
	*yaw = atan2f(siny_cosp, cosy_cosp);

	/* Convert to degrees */
	*roll *= 180.0f / M_PI;
	*pitch *= 180.0f / M_PI;
	*yaw *= 180.0f / M_PI;
}

/**
 * Enable quaternion sensors for demonstration
 */
static int enable_quaternion_sensors(const struct device *dev)
{
	struct sensor_value enable_mask;
	int ret;

	/* Enable all quaternion sensors */
	enable_mask.val1 = INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR |
			    INV_ICM20948_SENSOR_ROTATION_VECTOR |
			    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
	enable_mask.val2 = 0;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &enable_mask);
	if (ret) {
		LOG_ERR("Failed to enable quaternion sensors: %d", ret);
		return ret;
	}

	LOG_INF("Quaternion sensors enabled successfully");
	return 0;
}

int main(void)
{
	const struct device *icm20948_dev;

	LOG_INF("ICM20948 Packed Accuracy Demo Starting");

	/* Get ICM20948 device */
	if (!device_is_ready(DEVICE_DT_GET(ICM20948_NODE))) {
		LOG_ERR("ICM20948 device not ready");
		return -1;
	}
	icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);
	LOG_INF("ICM20948 device found: %s", icm20948_dev->name);

	/* Enable quaternion sensors */
	if (enable_quaternion_sensors(icm20948_dev) != 0) {
		return -1;
	}

	/* Wait for sensors to stabilize */
	k_sleep(K_SECONDS(2));

	/* Main demo loop */
	for (int i = 0; i < 10; i++) {
		LOG_INF("\n--- Sample %d ---", i + 1);
		
		demo_packed_quaternion_accuracy(icm20948_dev);
		demo_individual_accuracy(icm20948_dev);

		/* Demonstrate Euler angle conversion */
		struct sensor_value quat_data[5];
		int ret = sensor_sample_fetch(icm20948_dev);
		if (ret == 0) {
			ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_GAME_ROTATION_VECTOR, quat_data);
			if (ret == 0) {
				float roll, pitch, yaw;
				quaternion_to_euler(quat_data, &roll, &pitch, &yaw);
				LOG_INF("Euler Angles: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°", 
					(double)roll, (double)pitch, (double)yaw);
			}
		}

		k_sleep(K_SECONDS(1));
	}

	LOG_INF("Demo completed");
	return 0;
}
