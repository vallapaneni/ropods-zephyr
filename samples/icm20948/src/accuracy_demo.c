/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"

LOG_MODULE_REGISTER(icm20948_accuracy_demo, CONFIG_SENSOR_LOG_LEVEL);

/* ICM20948 device */
static const struct device *icm20948_dev = DEVICE_DT_GET_ONE(invensense_icm20948);

static void print_accuracy_legend(void)
{
	LOG_INF("\n--- Accuracy Scale Legend ---");
	LOG_INF("Individual sensors (Accel/Gyro/Mag): 0-3 scale");
	LOG_INF("  0 = Unreliable");
	LOG_INF("  1 = Low");  
	LOG_INF("  2 = Medium");
	LOG_INF("  3 = High");
	LOG_INF("");
	LOG_INF("Quaternion sensors (RV/GMRV): 0.0-1.0 normalized scale");
	LOG_INF("  0.0 = Unreliable");
	LOG_INF("  1.0 = High accuracy");
	LOG_INF("");
	LOG_INF("Game rotation vector: Uses minimum of Accel/Gyro accuracy");
	LOG_INF("=====================================");
}

static void print_all_accuracy(void)
{
	struct sensor_value accuracy;
	int ret;

	LOG_INF("\n--- Current Sensor Accuracy ---");

	/* Individual sensor accuracies */
	ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_ACCEL_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Accelerometer:     %d/3", accuracy.val1);
	} else {
		LOG_WRN("Accelerometer:     Error %d", ret);
	}

	ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_GYRO_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Gyroscope:         %d/3", accuracy.val1);
	} else {
		LOG_WRN("Gyroscope:         Error %d", ret);
	}

	ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_MAGN_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Magnetometer:      %d/3", accuracy.val1);
	} else {
		LOG_WRN("Magnetometer:      Error %d", ret);
	}

	/* Quaternion sensor accuracies */
	ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_QUAT6_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Game RV (6-axis):  %d/3", accuracy.val1);
	} else {
		LOG_WRN("Game RV (6-axis):  Error %d", ret);
	}

	ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_QUAT9_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Rotation Vector:   %d.%06d", accuracy.val1, abs(accuracy.val2));
	} else {
		LOG_WRN("Rotation Vector:   Error %d", ret);
	}

	ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_GMRV_ACCURACY, &accuracy);
	if (ret == 0) {
		LOG_INF("Geomag RV:         %d.%06d", accuracy.val1, abs(accuracy.val2));
	} else {
		LOG_WRN("Geomag RV:         Error %d", ret);
	}

	LOG_INF("---");
}

int main(void)
{
	struct sensor_value val;
	int ret;
	int sample_count = 0;

	LOG_INF("=======================================================");
	LOG_INF("ICM20948 Sensor Accuracy Monitoring Demo");
	LOG_INF("=======================================================");

	/* Check if device is ready */
	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -ENODEV;
	}

	LOG_INF("✓ ICM20948 device ready");
	print_accuracy_legend();

	/* Enable all quaternion-related sensors for comprehensive accuracy monitoring */
	LOG_INF("\n--- Enabling Sensors ---");
	
	val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | 
		   BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) |
		   BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR) |
		   BIT(INV_ICM20948_SENSOR_LINEAR_ACCELERATION);
	val.val2 = 0;
	
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable sensors: %d", ret);
		return ret;
	}
	LOG_INF("✓ Sensors enabled (Accel, Game RV, 9-axis RV, Linear Accel)");

	LOG_INF("\n--- Accuracy Monitoring (30 samples) ---");
	LOG_INF("Move the device to see accuracy changes...");

	/* Monitor accuracy for 30 samples */
	while (sample_count < 30) {
		/* Fetch new sensor data */
		ret = sensor_sample_fetch(icm20948_dev);
		if (ret) {
			LOG_ERR("Failed to fetch sample: %d", ret);
			k_sleep(K_MSEC(500));
			continue;
		}

		LOG_INF("\nSample %d:", ++sample_count);
		print_all_accuracy();

		/* Wait 1 second between samples */
		k_sleep(K_SECONDS(1));
	}

	LOG_INF("\n--- Accuracy Monitoring Complete ---");
	LOG_INF("Note: Accuracy typically improves over time as the sensors");
	LOG_INF("calibrate and stabilize. Movement helps with magnetometer");
	LOG_INF("calibration for 9-axis quaternion accuracy.");

	/* Disable all sensors */
	val.val1 = 0;
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to disable sensors: %d", ret);
	} else {
		LOG_INF("✓ All sensors disabled");
	}

	LOG_INF("Accuracy monitoring demo completed!");
	return 0;
}
