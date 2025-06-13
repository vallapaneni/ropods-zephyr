/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "icm20948_attr.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948SelfTest.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(icm20948_attr, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Set sampling frequency for ICM20948 sensors
 *
 * @param dev Pointer to the device structure
 * @param chan Sensor channel to configure
 * @param val Pointer to the frequency value to set
 *
 * @return 0 on success, negative error code on failure
 */
static int icm20948_attr_set_sampling_frequency(const struct device *dev,
						enum sensor_channel chan,
						const struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;
	int ret = 0;

	LOG_INF("Setting sampling frequency to %d.%06d Hz for channel %d", 
		val->val1, val->val2, chan);

	/* TODO: Implement sampling frequency setting via lib/emd API */
	/* This would typically involve calling inv_icm20948_set_odr() or similar */
	
	ARG_UNUSED(data);
	ARG_UNUSED(chan);

	return ret;
}

/**
 * @brief Set full scale range for ICM20948 sensors
 *
 * @param dev Pointer to the device structure
 * @param chan Sensor channel to configure
 * @param val Pointer to the range value to set
 *
 * @return 0 on success, negative error code on failure
 */
static int icm20948_attr_set_full_scale(const struct device *dev,
					enum sensor_channel chan,
					const struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;
	int ret = 0;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
		LOG_INF("Setting accelerometer range to %d.%06d", val->val1, val->val2);
		/* Set accel FSR via lib/emd */
		ret = inv_icm20948_set_fsr(&data->icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, &val->val1);
		break;

	case SENSOR_CHAN_GYRO_XYZ:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
		LOG_INF("Setting gyroscope range to %d.%06d", val->val1, val->val2);
		/* Set gyro FSR via lib/emd */
		ret = inv_icm20948_set_fsr(&data->icm_device, INV_ICM20948_SENSOR_GYROSCOPE, &val->val1);
		break;

	default:
		LOG_WRN("Full scale range setting not supported for channel %d", chan);
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

/**
 * @brief Trigger self-test for ICM20948 sensors
 *
 * @param dev Pointer to the device structure
 * @param chan Sensor channel (ignored - self-test runs on all sensors)
 * @param val Pointer to trigger value (any non-zero value triggers self-test)
 *
 * @return 0 on success, negative error code on failure
 */
static int icm20948_attr_set_self_test(const struct device *dev,
				       enum sensor_channel chan,
				       const struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;
	int ret = 0;
	
	ARG_UNUSED(chan); /* Self-test applies to all sensors */

	/* Only trigger self-test if value is non-zero */
	if (val->val1 == 0 && val->val2 == 0) {
		LOG_INF("Self-test not triggered (zero value)");
		return 0;
	}

	LOG_INF("Triggering ICM20948 self-test");

	/* Prepare bias arrays for self-test function */
	int gyro_bias[3] = {0};
	int accel_bias[3] = {0};

	/* Run self-test via eMD library */
	ret = inv_icm20948_run_selftest(&data->icm_device, gyro_bias, accel_bias);
	
	if (ret == 7) {  /* 7 = all sensors passed (compass | accel | gyro) */
		LOG_INF("ICM20948 self-test completed successfully (result: %d)", ret);
		LOG_INF("  Gyro bias: [%d, %d, %d]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
		LOG_INF("  Accel bias: [%d, %d, %d]", accel_bias[0], accel_bias[1], accel_bias[2]);
		ret = 0;  /* Convert to standard success code */
	} else {
		LOG_ERR("ICM20948 self-test failed with result: %d", ret);
		LOG_ERR("  Expected: 7 (all sensors pass), Got: %d", ret);
		if (!(ret & 0x1)) LOG_ERR("  → Gyroscope self-test failed");
		if (!(ret & 0x2)) LOG_ERR("  → Accelerometer self-test failed");
		if (!(ret & 0x4)) LOG_ERR("  → Compass self-test failed");
		ret = -EIO;  /* Convert to standard error code */
	}

	return ret;
}

/**
 * @brief Enable or disable sensors and interrupts for ICM20948
 *
 * @param dev Pointer to the device structure
 * @param chan Sensor channel to configure (ignored for sensor control)
 * @param val Pointer to the sensor mask value (0 = disable all, non-zero = enable based on eMD sensor mask)
 *        Use INV_ICM20948_SENSOR_* values from eMD library
 *
 * @return 0 on success, negative error code on failure
 */
static int icm20948_attr_set_sensor_enable(const struct device *dev,
					    enum sensor_channel chan,
					    const struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;
	uint32_t sensor_mask = (uint32_t)val->val1;
	int ret;

	ARG_UNUSED(chan); /* Sensor control applies globally */

	LOG_INF("Setting sensor enable mask to 0x%08X", sensor_mask);

	/* 
	 * NOTE: We process the sensor mask bit by bit because the eMD library function
	 * inv_icm20948_enable_sensor() only accepts individual sensor enums, not masks.
	 * 
	 * Function signature: int inv_icm20948_enable_sensor(struct inv_icm20948 * s, 
	 *                                                     enum inv_icm20948_sensor sensor, 
	 *                                                     inv_bool_t state)
	 * 
	 * The function expects:
	 * - A single sensor enum (like INV_ICM20948_SENSOR_ACCELEROMETER)
	 * - A simple boolean state (0 or 1)
	 * 
	 * It does NOT support sensor masks or bit combinations, so we must expand
	 * the mask bit by bit and call the function for each individual sensor.
	 */

	uint32_t current_mask = data->sensor_enable_mask;
	uint32_t sensors_to_disable = current_mask & ~sensor_mask;  /* Sensors that need to be disabled */
	uint32_t sensors_to_enable = sensor_mask & ~current_mask;   /* Sensors that need to be enabled */

	/* Disable sensors that are currently enabled but not in the new mask */
	if (sensors_to_disable & BIT(INV_ICM20948_SENSOR_ACCELEROMETER)) {
		ret = inv_icm20948_enable_sensor(&data->icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, 0);
		if (ret != 0) {
			LOG_ERR("Failed to disable accelerometer: %d", ret);
			return -EIO;
		}
		LOG_INF("Accelerometer disabled");
	}
	
	if (sensors_to_disable & BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR)) {
		ret = inv_icm20948_enable_sensor(&data->icm_device, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 0);
		if (ret != 0) {
			LOG_ERR("Failed to disable game rotation vector: %d", ret);
			return -EIO;
		}
		LOG_INF("Game rotation vector disabled");
	}
	
	if (sensors_to_disable & BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR)) {
		ret = inv_icm20948_enable_sensor(&data->icm_device, INV_ICM20948_SENSOR_ROTATION_VECTOR, 0);
		if (ret != 0) {
			LOG_ERR("Failed to disable rotation vector: %d", ret);
			return -EIO;
		}
		LOG_INF("Rotation vector disabled");
	}

	if (sensors_to_disable & BIT(INV_ICM20948_SENSOR_LINEAR_ACCELERATION)) {
		ret = inv_icm20948_enable_sensor(&data->icm_device, INV_ICM20948_SENSOR_LINEAR_ACCELERATION, 0);
		if (ret != 0) {
			LOG_ERR("Failed to disable linear acceleration: %d", ret);
			return -EIO;
		}
		LOG_INF("Linear acceleration disabled");
	}

	/* Enable sensors that are in the new mask but not currently enabled */
	if (sensors_to_enable & BIT(INV_ICM20948_SENSOR_ACCELEROMETER)) {
		ret = inv_icm20948_enable_sensor(&data->icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, 1);
		if (ret != 0) {
			LOG_ERR("Failed to enable accelerometer: %d", ret);
			return -EIO;
		}
		LOG_INF("Accelerometer enabled");
	}
	
	if (sensors_to_enable & BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR)) {
		ret = inv_icm20948_enable_sensor(&data->icm_device, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1);
		if (ret != 0) {
			LOG_ERR("Failed to enable game rotation vector: %d", ret);
			return -EIO;
		}
		LOG_INF("Game rotation vector enabled");
	}
	
	if (sensors_to_enable & BIT(INV_ICM20948_SENSOR_ROTATION_VECTOR)) {
		ret = inv_icm20948_enable_sensor(&data->icm_device, INV_ICM20948_SENSOR_ROTATION_VECTOR, 1);
		if (ret != 0) {
			LOG_ERR("Failed to enable rotation vector: %d", ret);
			return -EIO;
		}
		LOG_INF("Rotation vector enabled");
	}

	if (sensors_to_enable & BIT(INV_ICM20948_SENSOR_LINEAR_ACCELERATION)) {
		ret = inv_icm20948_enable_sensor(&data->icm_device, INV_ICM20948_SENSOR_LINEAR_ACCELERATION, 1);
		if (ret != 0) {
			LOG_ERR("Failed to enable linear acceleration: %d", ret);
			return -EIO;
		}
		LOG_INF("Linear acceleration enabled");
	}

#ifdef CONFIG_ICM20948_TRIGGER
	/* Manage GPIO interrupts based on sensor state */
	const struct icm20948_config *cfg = dev->config;
	if (cfg->int_gpio.port) {
		if (sensor_mask == 0 && data->interrupt_enabled) {
			/* Disable interrupts when no sensors are enabled */
			ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);
			if (ret < 0) {
				LOG_ERR("Failed to disable GPIO interrupt: %d", ret);
				return ret;
			}
			data->interrupt_enabled = false;
			LOG_INF("GPIO interrupts disabled");
		} else if (sensor_mask != 0 && !data->interrupt_enabled) {
			/* Enable interrupts when sensors are enabled */
			ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
			if (ret < 0) {
				LOG_ERR("Failed to enable GPIO interrupt: %d", ret);
				return ret;
			}
			data->interrupt_enabled = true;
			LOG_INF("GPIO interrupts enabled");
		}
	}
#endif

	/* Update the stored sensor enable mask */
	data->sensor_enable_mask = sensor_mask;

	LOG_INF("Sensor enable operation completed successfully");
	return 0;
}

int icm20948_attr_set(const struct device *dev, enum sensor_channel chan,
		      enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = 0;

	if (val == NULL) {
		return -EINVAL;
	}

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		ret = icm20948_attr_set_sampling_frequency(dev, chan, val);
		break;

	case SENSOR_ATTR_FULL_SCALE:
		ret = icm20948_attr_set_full_scale(dev, chan, val);
		break;

	case SENSOR_ATTR_ICM20948_SENSOR_ENABLE:
		ret = icm20948_attr_set_sensor_enable(dev, chan, val);
		break;

	case SENSOR_ATTR_ICM20948_SELF_TEST:
		ret = icm20948_attr_set_self_test(dev, chan, val);
		break;

	case SENSOR_ATTR_ICM20948_DATA_LENGTH:
		/* Data length is a read-only attribute */
		LOG_WRN("Data length is a read-only attribute");
		ret = -EACCES;
		break;

	default:
		LOG_WRN("Attribute %d not supported", attr);
		ret = -ENOTSUP;
		break;
	}

	if (ret != 0) {
		LOG_ERR("Failed to set attribute %d for channel %d: %d", attr, chan, ret);
	}

	return ret;
}

int icm20948_attr_get(const struct device *dev, enum sensor_channel chan,
		      enum sensor_attribute attr, struct sensor_value *val)
{
	int ret = 0;

	if (val == NULL) {
		return -EINVAL;
	}

	/* Initialize the value to zero */
	val->val1 = 0;
	val->val2 = 0;

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		/* TODO: Implement sampling frequency getting via lib/emd API */
		LOG_INF("Getting sampling frequency for channel %d", chan);
		/* This would typically involve calling inv_icm20948_get_odr() or similar */
		ret = -ENOTSUP; /* Not implemented yet */
		break;

	case SENSOR_ATTR_FULL_SCALE:
		/* TODO: Implement full scale range getting via lib/emd API */
		LOG_INF("Getting full scale range for channel %d", chan);
		/* This would typically involve calling inv_icm20948_get_fsr() or similar */
		ret = -ENOTSUP; /* Not implemented yet */
		break;

	case SENSOR_ATTR_ICM20948_SENSOR_ENABLE:
		{
			struct icm20948_data *data = dev->data;
			val->val1 = (int32_t)data->sensor_enable_mask;
			val->val2 = 0;
			LOG_INF("Getting sensor enable mask: 0x%08X", data->sensor_enable_mask);
		}
		break;

	case SENSOR_ATTR_ICM20948_SELF_TEST:
		/* Self-test is write-only (trigger-only) attribute */
		LOG_WRN("Self-test is a write-only (trigger) attribute");
		ret = -EACCES;
		break;

	case SENSOR_ATTR_ICM20948_DATA_LENGTH:
		/* Return the number of sensor_value elements needed for the specified channel */
		switch (chan) {
		case SENSOR_CHAN_ACCEL_XYZ:
		case SENSOR_CHAN_GYRO_XYZ:
		case SENSOR_CHAN_MAGN_XYZ:
			val->val1 = 3;  /* X, Y, Z */
			break;
		case SENSOR_CHAN_ACCEL_X:
		case SENSOR_CHAN_ACCEL_Y:
		case SENSOR_CHAN_ACCEL_Z:
		case SENSOR_CHAN_GYRO_X:
		case SENSOR_CHAN_GYRO_Y:
		case SENSOR_CHAN_GYRO_Z:
		case SENSOR_CHAN_MAGN_X:
		case SENSOR_CHAN_MAGN_Y:
		case SENSOR_CHAN_MAGN_Z:
		case SENSOR_CHAN_DIE_TEMP:
			val->val1 = 1;  /* Single value */
			break;
		case SENSOR_CHAN_GAME_ROTATION_VECTOR:
		case SENSOR_CHAN_ICM20948_ROTATION_VECTOR:
			val->val1 = 5;  /* Quaternion w,x,y,z + accuracy */
			break;
		case SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION:
			val->val1 = 4;  /* X, Y, Z + accuracy */
			break;
		default:
			LOG_WRN("Data length not defined for channel %d", chan);
			ret = -ENOTSUP;
			break;
		}
		if (ret == 0) {
			val->val2 = 0;
			LOG_DBG("Channel %d data length: %d", chan, val->val1);
		}
		break;

	default:
		LOG_WRN("Attribute %d not supported for getting", attr);
		ret = -ENOTSUP;
		break;
	}

	if (ret != 0) {
		LOG_ERR("Failed to get attribute %d for channel %d: %d", attr, chan, ret);
	}

	return ret;
}
