/*
 * Copyright (c) 2024 ROPODS
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(icm20948_sample, LOG_LEVEL_INF);

#define ICM20948_NODE DT_NODELABEL(icm20948)

static const struct device *const icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);

#ifdef CONFIG_ICM20948_TRIGGER
static void icm20948_data_ready_handler(const struct device *dev,
					const struct sensor_trigger *trigger)
{
	struct sensor_value accel[3], gyro[3], temp;
	int ret;

	ret = sensor_sample_fetch(dev);
	if (ret) {
		LOG_ERR("Failed to fetch sample: %d", ret);
		return;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret) {
		LOG_ERR("Failed to get accelerometer data: %d", ret);
		return;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (ret) {
		LOG_ERR("Failed to get gyroscope data: %d", ret);
		return;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);
	if (ret) {
		LOG_ERR("Failed to get temperature data: %d", ret);
		return;
	}

	LOG_INF("Accel: X=%d.%06d Y=%d.%06d Z=%d.%06d m/s²",
		accel[0].val1, abs(accel[0].val2),
		accel[1].val1, abs(accel[1].val2),
		accel[2].val1, abs(accel[2].val2));

	LOG_INF("Gyro: X=%d.%06d Y=%d.%06d Z=%d.%06d rad/s",
		gyro[0].val1, abs(gyro[0].val2),
		gyro[1].val1, abs(gyro[1].val2),
		gyro[2].val1, abs(gyro[2].val2));

	LOG_INF("Temperature: %d.%06d °C",
		temp.val1, abs(temp.val2));
}
#endif

int main(void)
{
	int ret;

	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -ENODEV;
	}

	LOG_INF("ICM20948 device ready");

#ifdef CONFIG_ICM20948_TRIGGER
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	ret = sensor_trigger_set(icm20948_dev, &trig, icm20948_data_ready_handler);
	if (ret) {
		LOG_ERR("Failed to set trigger: %d", ret);
	} else {
		LOG_INF("Trigger mode enabled - data will be printed on interrupt");
		
		/* In trigger mode, just sleep */
		while (1) {
			k_sleep(K_SECONDS(1));
		}
	}
#else
	LOG_INF("Polling mode - reading sensor every second");

	while (1) {
		struct sensor_value accel[3], gyro[3], temp;
		
		ret = sensor_sample_fetch(icm20948_dev);
		if (ret) {
			LOG_ERR("Failed to fetch sample: %d", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}

		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (ret) {
			LOG_ERR("Failed to get accelerometer data: %d", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}

		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		if (ret) {
			LOG_ERR("Failed to get gyroscope data: %d", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}

		ret = sensor_channel_get(icm20948_dev, SENSOR_CHAN_DIE_TEMP, &temp);
		if (ret) {
			LOG_ERR("Failed to get temperature data: %d", ret);
			k_sleep(K_SECONDS(1));
			continue;
		}

		LOG_INF("Accel: X=%d.%06d Y=%d.%06d Z=%d.%06d m/s²",
			accel[0].val1, abs(accel[0].val2),
			accel[1].val1, abs(accel[1].val2),
			accel[2].val1, abs(accel[2].val2));

		LOG_INF("Gyro: X=%d.%06d Y=%d.%06d Z=%d.%06d rad/s",
			gyro[0].val1, abs(gyro[0].val2),
			gyro[1].val1, abs(gyro[1].val2),
			gyro[2].val1, abs(gyro[2].val2));

		LOG_INF("Temperature: %d.%06d °C",
			temp.val1, abs(temp.val2));

		k_sleep(K_SECONDS(1));
	}
#endif

	return 0;
}
