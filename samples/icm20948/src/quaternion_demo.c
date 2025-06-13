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

LOG_MODULE_REGISTER(icm20948_quaternion_demo, CONFIG_SENSOR_LOG_LEVEL);

/* ICM20948 device */
static const struct device *icm20948_dev = DEVICE_DT_GET_ONE(invensense_icm20948);

int main(void)
{
	struct sensor_value val;
	struct sensor_value quat6[4], quat9[4], linear_accel[3];
	int ret;

	LOG_INF("=======================================================");
	LOG_INF("ICM20948 Quaternion and Linear Acceleration Demo");
	LOG_INF("=======================================================");

	/* Check if device is ready */
	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -ENODEV;
	}

	LOG_INF("✓ ICM20948 device ready");

	/* Enable quaternion and linear acceleration sensors */
	LOG_INF("\n--- Enabling Sensors ---");
	
	/* Enable accelerometer as a base requirement */
	val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER);
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable accelerometer: %d", ret);
		return ret;
	}
	LOG_INF("✓ Accelerometer enabled");

	/* Enable game rotation vector (6-axis quaternion) */
	val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable game rotation vector: %d", ret);
		return ret;
	}
	LOG_INF("✓ Game rotation vector (6-axis quaternion) enabled");

	/* Enable full rotation vector (9-axis quaternion) */
	val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | 
		   BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) |
		   BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR);
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable rotation vector: %d", ret);
		return ret;
	}
	LOG_INF("✓ Full rotation vector (9-axis quaternion) enabled");

	/* Enable linear acceleration */
	val.val1 = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | 
		   BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) |
		   BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR) |
		   BIT(INV_ICM20948_SENSOR_LINEAR_ACCELERATION);
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to enable linear acceleration: %d", ret);
		return ret;
	}
	LOG_INF("✓ Linear acceleration enabled");

	LOG_INF("\n--- Starting Data Collection ---");
	LOG_INF("Note: DMP needs time to stabilize, initial readings may be zero");
	LOG_INF("Quaternion format: [w, x, y, z] (unit quaternion, normalized)");
	LOG_INF("Linear acceleration: [x, y, z] in m/s² (gravity removed)");

	/* Data collection loop */
	for (int i = 0; i < 100; i++) {
		/* Wait for data */
		k_sleep(K_MSEC(100));

		/* Fetch sensor data */
		ret = sensor_sample_fetch(icm20948_dev);
		if (ret) {
			LOG_WRN("Failed to fetch sample: %d", ret);
			continue;
		}

		/* Get 6-axis quaternion (game rotation vector) */
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_GAME_ROTATION_VECTOR, quat6);
		if (ret == 0) {
			LOG_INF("Quat6: [%d.%06d, %d.%06d, %d.%06d, %d.%06d]",
				quat6[0].val1, abs(quat6[0].val2),
				quat6[1].val1, abs(quat6[1].val2),
				quat6[2].val1, abs(quat6[2].val2),
				quat6[3].val1, abs(quat6[3].val2));
		} else {
			LOG_DBG("No quat6 data available: %d", ret);
		}

		/* Get 9-axis quaternion (full rotation vector) */
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_ROTATION_VECTOR, quat9);
		if (ret == 0) {
			LOG_INF("Quat9: [%d.%06d, %d.%06d, %d.%06d, %d.%06d]",
				quat9[0].val1, abs(quat9[0].val2),
				quat9[1].val1, abs(quat9[1].val2),
				quat9[2].val1, abs(quat9[2].val2),
				quat9[3].val1, abs(quat9[3].val2));
		} else {
			LOG_DBG("No quat9 data available: %d", ret);
		}

		/* Get linear acceleration */
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION, linear_accel);
		if (ret == 0) {
			LOG_INF("LinAcc: [%d.%06d, %d.%06d, %d.%06d] m/s²",
				linear_accel[0].val1, abs(linear_accel[0].val2),
				linear_accel[1].val1, abs(linear_accel[1].val2),
				linear_accel[2].val1, abs(linear_accel[2].val2));
		} else {
			LOG_DBG("No linear acceleration data available: %d", ret);
		}

		/* Get sensor accuracy information */
		struct sensor_value accuracy;
		
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_ACCEL_ACCURACY, &accuracy);
		if (ret == 0) {
			LOG_INF("Accuracy - Accel: %d", accuracy.val1);
		}
		
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_GYRO_ACCURACY, &accuracy);
		if (ret == 0) {
			LOG_INF("Accuracy - Gyro: %d", accuracy.val1);
		}
		
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_MAGN_ACCURACY, &accuracy);
		if (ret == 0) {
			LOG_INF("Accuracy - Mag: %d", accuracy.val1);
		}
		
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_QUAT6_ACCURACY, &accuracy);
		if (ret == 0) {
			LOG_INF("Accuracy - Quat6 (Game): %d", accuracy.val1);
		}
		
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_QUAT9_ACCURACY, &accuracy);
		if (ret == 0) {
			LOG_INF("Accuracy - Quat9 (9-axis): %d.%06d", accuracy.val1, abs(accuracy.val2));
		}
		
		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ICM20948_GMRV_ACCURACY, &accuracy);
		if (ret == 0) {
			LOG_INF("Accuracy - GMRV: %d.%06d", accuracy.val1, abs(accuracy.val2));
		}

		LOG_INF("---");
	}

	LOG_INF("\n--- Cleaning up ---");
	
	/* Disable all sensors */
	val.val1 = 0;
	val.val2 = 0;
	ret = sensor_attr_set(icm20948_dev, SENSOR_CHAN_ALL, 
			      SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to disable all sensors: %d", ret);
	} else {
		LOG_INF("✓ All sensors disabled");
	}

	LOG_INF("Demo completed successfully!");
	return 0;
}
