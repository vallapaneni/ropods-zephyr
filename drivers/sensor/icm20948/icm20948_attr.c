/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "icm20948_attr.h"

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
 * @brief Enable or disable interrupt generation for ICM20948
 *
 * @param dev Pointer to the device structure
 * @param chan Sensor channel to configure (ignored for interrupt control)
 * @param val Pointer to the enable value (0 = disable, non-zero = enable)
 *
 * @return 0 on success, negative error code on failure
 */
static int icm20948_attr_set_interrupt_enable(const struct device *dev,
					      enum sensor_channel chan,
					      const struct sensor_value *val)
{
#ifdef CONFIG_ICM20948_TRIGGER
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *cfg = dev->config;
	bool enable = (val->val1 != 0);
	int ret;

	ARG_UNUSED(chan); /* Interrupt control applies globally */

	if (!cfg->int_gpio.port) {
		LOG_WRN("No interrupt GPIO configured");
		return -ENOTSUP;
	}

	LOG_INF("Setting interrupt enable to %s", enable ? "enabled" : "disabled");

	if (enable && !data->interrupt_enabled) {
		/* Enable GPIO interrupt */
		ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to enable GPIO interrupt: %d", ret);
			return ret;
		}
		data->interrupt_enabled = true;
	} else if (!enable && data->interrupt_enabled) {
		/* Disable GPIO interrupt */
		ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);
		if (ret < 0) {
			LOG_ERR("Failed to disable GPIO interrupt: %d", ret);
			return ret;
		}
		data->interrupt_enabled = false;
	}

	return 0;
#else
	LOG_WRN("Trigger support not compiled in");
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
	ARG_UNUSED(val);
	return -ENOTSUP;
#endif /* CONFIG_ICM20948_TRIGGER */
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

	case SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE:
		ret = icm20948_attr_set_interrupt_enable(dev, chan, val);
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

	case SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE:
#ifdef CONFIG_ICM20948_TRIGGER
		{
			struct icm20948_data *data = dev->data;
			val->val1 = data->interrupt_enabled ? 1 : 0;
			val->val2 = 0;
			LOG_INF("Getting interrupt enable state: %d", val->val1);
		}
#else
		LOG_WRN("Trigger support not compiled in");
		ret = -ENOTSUP;
#endif /* CONFIG_ICM20948_TRIGGER */
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
