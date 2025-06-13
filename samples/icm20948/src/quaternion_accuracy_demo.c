/*
 * ICM-20948 Accuracy Demo
 * 
 * This demo demonstrates how to read accuracy information packed with quaternion data
 * and individual sensor components (accelerometer, gyroscope, magnetometer).
 * 
 * Quaternion channels now return 5 values: w,x,y,z,accuracy
 * - Game rotation vector (quat6): accuracy is minimum of accel and gyro accuracy (0-3)
 * - 9-axis rotation vector (quat9): accuracy is normalized 0.0-1.0 (1.0=highest accuracy)
 * - Geomagnetic rotation vector (gmrv): accuracy is normalized 0.0-1.0 (1.0=highest accuracy)
 * 
 * Individual sensor accuracies: 0-3 (0=unreliable, 3=high accuracy)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "../../drivers/sensor/icm20948/icm20948.h"

LOG_MODULE_REGISTER(accuracy_demo, LOG_LEVEL_INF);

/* Get device from devicetree */
static const struct device *icm20948_dev = DEVICE_DT_GET_ONE(invensense_icm20948);

/* Function to print quaternion data with packed accuracy */
static void print_quaternion_with_accuracy(const struct device *dev)
{
	struct sensor_value quat_data[5]; /* w,x,y,z,accuracy */
	int ret;

	LOG_INF("=== Quaternion Data with Packed Accuracy ===");

	/* Read 6-axis quaternion (Game Rotation Vector) with packed accuracy */
	ret = sensor_channel_get(dev, SENSOR_CHAN_GAME_ROTATION_VECTOR, quat_data);
	if (ret == 0) {
		LOG_INF("Game Rotation Vector (6-axis: accel+gyro):");
		LOG_INF("  Quaternion: w=%d.%06d x=%d.%06d y=%d.%06d z=%d.%06d",
			quat_data[0].val1, abs(quat_data[0].val2),
			quat_data[1].val1, abs(quat_data[1].val2),
			quat_data[2].val1, abs(quat_data[2].val2),
			quat_data[3].val1, abs(quat_data[3].val2));
		LOG_INF("  Accuracy: %d (0=unreliable, 3=high)", quat_data[4].val1);
	} else {
		LOG_WRN("Failed to read game rotation vector: %d", ret);
	}

	/* Read 9-axis quaternion (Full Rotation Vector) with packed accuracy */
	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_ROTATION_VECTOR, quat_data);
	if (ret == 0) {
		LOG_INF("9-axis Rotation Vector (accel+gyro+mag):");
		LOG_INF("  Quaternion: w=%d.%06d x=%d.%06d y=%d.%06d z=%d.%06d",
			quat_data[0].val1, abs(quat_data[0].val2),
			quat_data[1].val1, abs(quat_data[1].val2),
			quat_data[2].val1, abs(quat_data[2].val2),
			quat_data[3].val1, abs(quat_data[3].val2));
		LOG_INF("  Accuracy: %d.%06d (0.0=unreliable, 1.0=highest)",
			quat_data[4].val1, abs(quat_data[4].val2));
	} else {
		LOG_WRN("Failed to read 9-axis rotation vector: %d", ret);
	}

	/* Read Geomagnetic Rotation Vector with packed accuracy */
	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_GEOMAGNETIC_ROTATION_VECTOR, quat_data);
	if (ret == 0) {
		LOG_INF("Geomagnetic Rotation Vector (accel+mag):");
		LOG_INF("  Quaternion: w=%d.%06d x=%d.%06d y=%d.%06d z=%d.%06d",
			quat_data[0].val1, abs(quat_data[0].val2),
			quat_data[1].val1, abs(quat_data[1].val2),
			quat_data[2].val1, abs(quat_data[2].val2),
			quat_data[3].val1, abs(quat_data[3].val2));
		LOG_INF("  Accuracy: %d.%06d (0.0=unreliable, 1.0=highest)",
			quat_data[4].val1, abs(quat_data[4].val2));
	} else {
		LOG_WRN("Failed to read geomagnetic rotation vector: %d", ret);
	}
}

/* Function to print individual sensor accuracy (if needed separately) */
static void print_individual_sensor_accuracy(const struct device *dev)
{
	struct sensor_value accel_acc, gyro_acc, mag_acc;
	int ret;

	LOG_INF("=== Individual Sensor Accuracy ===");

	/* Read individual sensor accuracies (0-3 scale) */
	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_ACCEL_ACCURACY, &accel_acc);
	if (ret == 0) {
		LOG_INF("Accelerometer Accuracy: %d (0=unreliable, 3=high)", accel_acc.val1);
	} else {
		LOG_WRN("Failed to read accelerometer accuracy: %d", ret);
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_GYRO_ACCURACY, &gyro_acc);
	if (ret == 0) {
		LOG_INF("Gyroscope Accuracy:     %d (0=unreliable, 3=high)", gyro_acc.val1);
	} else {
		LOG_WRN("Failed to read gyroscope accuracy: %d", ret);
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ICM20948_MAGN_ACCURACY, &mag_acc);
	if (ret == 0) {
		LOG_INF("Magnetometer Accuracy:  %d (0=unreliable, 3=high)", mag_acc.val1);
	} else {
		LOG_WRN("Failed to read magnetometer accuracy: %d", ret);
	}
}

/* Function to enable quaternion sensors */
static int enable_quaternion_sensors(const struct device *dev)
{
	struct sensor_value sensor_mask;
	int ret;

	/* Enable accelerometer, gyroscope, magnetometer, and quaternion sensors
	 * Bit mask: 
	 * - INV_ICM20948_SENSOR_ACCELEROMETER = bit 0
	 * - INV_ICM20948_SENSOR_GYROSCOPE = bit 1  
	 * - INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD = bit 2
	 * - INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR = bit 11  
	 * - INV_ICM20948_SENSOR_ROTATION_VECTOR = bit 12
	 * - INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR = bit 20
	 */
	uint32_t enable_mask = BIT(0) |   /* Accelerometer */
			       BIT(1) |   /* Gyroscope */
			       BIT(2) |   /* Magnetometer */
			       BIT(11) |  /* Game rotation vector */
			       BIT(12) |  /* Rotation vector */
			       BIT(20);   /* Geomagnetic rotation vector */

	sensor_mask.val1 = enable_mask;
	sensor_mask.val2 = 0;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &sensor_mask);
	if (ret != 0) {
		LOG_ERR("Failed to enable quaternion sensors: %d", ret);
		return ret;
	}

	LOG_INF("Enabled accelerometer, gyroscope, magnetometer, and all quaternion sensors");
	
	/* Wait for sensors to stabilize */
	k_sleep(K_MSEC(500));
	
	return 0;
}

int main(void)
{
	int ret;

	LOG_INF("ICM-20948 Packed Accuracy Demo Starting");

	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM-20948 device not ready");
		return -ENODEV;
	}

	LOG_INF("ICM-20948 device found: %s", icm20948_dev->name);

	/* Enable quaternion sensors */
	ret = enable_quaternion_sensors(icm20948_dev);
	if (ret != 0) {
		return ret;
	}

	/* Main loop - show different accuracy information */
	int loop_count = 0;
	while (1) {
		LOG_INF("\n=== Loop %d ===", ++loop_count);

		/* Fetch fresh sensor data */
		ret = sensor_sample_fetch(icm20948_dev);
		if (ret != 0) {
			LOG_ERR("Failed to fetch sensor data: %d", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Demonstrate packed accuracy format */
		print_quaternion_with_accuracy(icm20948_dev);
		
		/* Show individual sensor accuracies if needed */
		if (loop_count % 3 == 0) {
			print_individual_sensor_accuracy(icm20948_dev);
		}

		/* Wait before next reading */
		k_sleep(K_SECONDS(2));
	}

	return 0;
}
