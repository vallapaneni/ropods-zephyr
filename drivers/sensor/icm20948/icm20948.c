/*
 * Copyright (c) 2024 ROPODS
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm20948

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

#include "icm20948.h"

LOG_MODULE_REGISTER(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

static int icm20948_reset(const struct device *dev)
{
	int ret;

	ret = icm20948_select_bank(dev, ICM20948_BANK_0);
	if (ret) {
		return ret;
	}

	ret = icm20948_spi_write_reg(dev, ICM20948_REG_PWR_MGMT_1, ICM20948_PWR_MGMT_1_DEVICE_RESET);
	if (ret) {
		return ret;
	}

	k_msleep(100); /* Wait for reset to complete */

	return 0;
}

static int icm20948_setup(const struct device *dev)
{
	const struct icm20948_config *config = dev->config;
	uint8_t chip_id, val;
	int ret;

	/* Reset the device */
	ret = icm20948_reset(dev);
	if (ret) {
		LOG_ERR("Failed to reset device");
		return ret;
	}

	/* Select bank 0 and check chip ID */
	ret = icm20948_select_bank(dev, ICM20948_BANK_0);
	if (ret) {
		return ret;
	}

	ret = icm20948_spi_read_reg(dev, ICM20948_REG_WHO_AM_I, &chip_id);
	if (ret) {
		LOG_ERR("Failed to read chip ID");
		return ret;
	}

	if (chip_id != ICM20948_CHIP_ID) {
		LOG_ERR("Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, ICM20948_CHIP_ID);
		return -ENODEV;
	}

	LOG_INF("ICM20948 chip ID: 0x%02X", chip_id);

	/* Disable I2C interface and enable SPI */
	ret = icm20948_spi_write_reg(dev, ICM20948_REG_USER_CTRL, ICM20948_USER_CTRL_I2C_IF_DIS);
	if (ret) {
		return ret;
	}

	/* Wake up device and set clock source */
	ret = icm20948_spi_write_reg(dev, ICM20948_REG_PWR_MGMT_1, 0x01); /* Use best available clock source */
	if (ret) {
		return ret;
	}

	/* Enable accelerometer and gyroscope */
	ret = icm20948_spi_write_reg(dev, ICM20948_REG_PWR_MGMT_2, 0x00); /* Enable all axes */
	if (ret) {
		return ret;
	}

	/* Configure gyroscope (Bank 2) */
	ret = icm20948_select_bank(dev, ICM20948_BANK_2);
	if (ret) {
		return ret;
	}

	/* Set gyroscope range */
	switch (config->gyro_range) {
	case 250:
		val = ICM20948_GYRO_FS_SEL_250DPS;
		break;
	case 500:
		val = ICM20948_GYRO_FS_SEL_500DPS;
		break;
	case 1000:
		val = ICM20948_GYRO_FS_SEL_1000DPS;
		break;
	case 2000:
		val = ICM20948_GYRO_FS_SEL_2000DPS;
		break;
	default:
		val = ICM20948_GYRO_FS_SEL_250DPS;
		break;
	}

	ret = icm20948_spi_write_reg(dev, ICM20948_REG_GYRO_CONFIG_1, val);
	if (ret) {
		return ret;
	}

	/* Set accelerometer range */
	switch (config->accel_range) {
	case 2:
		val = ICM20948_ACCEL_FS_SEL_2G;
		break;
	case 4:
		val = ICM20948_ACCEL_FS_SEL_4G;
		break;
	case 8:
		val = ICM20948_ACCEL_FS_SEL_8G;
		break;
	case 16:
		val = ICM20948_ACCEL_FS_SEL_16G;
		break;
	default:
		val = ICM20948_ACCEL_FS_SEL_2G;
		break;
	}

	ret = icm20948_spi_write_reg(dev, ICM20948_REG_ACCEL_CONFIG, val);
	if (ret) {
		return ret;
	}

	/* Return to bank 0 */
	ret = icm20948_select_bank(dev, ICM20948_BANK_0);
	if (ret) {
		return ret;
	}

	return 0;
}

static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct icm20948_data *data = dev->data;
	uint8_t raw_data[14];
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			 chan == SENSOR_CHAN_ACCEL_XYZ ||
			 chan == SENSOR_CHAN_GYRO_XYZ ||
			 chan == SENSOR_CHAN_DIE_TEMP);

	ret = icm20948_select_bank(dev, ICM20948_BANK_0);
	if (ret) {
		return ret;
	}

	/* Read accelerometer, gyroscope, and temperature data */
	ret = icm20948_spi_read(dev, ICM20948_REG_ACCEL_XOUT_H, raw_data, 14);
	if (ret) {
		LOG_ERR("Failed to read sensor data");
		return ret;
	}

	/* Convert raw data to signed 16-bit values */
	data->accel_x = (int16_t)sys_get_be16(&raw_data[0]);
	data->accel_y = (int16_t)sys_get_be16(&raw_data[2]);
	data->accel_z = (int16_t)sys_get_be16(&raw_data[4]);
	data->gyro_x = (int16_t)sys_get_be16(&raw_data[6]);
	data->gyro_y = (int16_t)sys_get_be16(&raw_data[8]);
	data->gyro_z = (int16_t)sys_get_be16(&raw_data[10]);
	data->temp = (int16_t)sys_get_be16(&raw_data[12]);

	return 0;
}

static void icm20948_accel_convert(struct sensor_value *val, int16_t raw_val,
				   uint16_t range)
{
	int32_t scale;

	/* Convert to m/s^2 */
	switch (range) {
	case 2:
		scale = 2 * SENSOR_G / 32768;
		break;
	case 4:
		scale = 4 * SENSOR_G / 32768;
		break;
	case 8:
		scale = 8 * SENSOR_G / 32768;
		break;
	case 16:
		scale = 16 * SENSOR_G / 32768;
		break;
	default:
		scale = 2 * SENSOR_G / 32768;
		break;
	}

	int64_t tmp = (int64_t)raw_val * scale;
	val->val1 = tmp / 1000000;
	val->val2 = tmp % 1000000;
}

static void icm20948_gyro_convert(struct sensor_value *val, int16_t raw_val,
				  uint16_t range)
{
	int32_t scale;

	/* Convert to rad/s */
	switch (range) {
	case 250:
		scale = 250 * SENSOR_PI / (32768 * 180);
		break;
	case 500:
		scale = 500 * SENSOR_PI / (32768 * 180);
		break;
	case 1000:
		scale = 1000 * SENSOR_PI / (32768 * 180);
		break;
	case 2000:
		scale = 2000 * SENSOR_PI / (32768 * 180);
		break;
	default:
		scale = 250 * SENSOR_PI / (32768 * 180);
		break;
	}

	int64_t tmp = (int64_t)raw_val * scale;
	val->val1 = tmp / 1000000;
	val->val2 = tmp % 1000000;
}

static void icm20948_temp_convert(struct sensor_value *val, int16_t raw_val)
{
	/* Temperature in Celsius = (TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity + 21°C */
	int32_t temp = (raw_val / 333.87) + 21;
	
	val->val1 = temp;
	val->val2 = 0;
}

static int icm20948_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *config = dev->config;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		icm20948_accel_convert(val, data->accel_x, config->accel_range);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20948_accel_convert(val, data->accel_y, config->accel_range);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20948_accel_convert(val, data->accel_z, config->accel_range);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20948_accel_convert(&val[0], data->accel_x, config->accel_range);
		icm20948_accel_convert(&val[1], data->accel_y, config->accel_range);
		icm20948_accel_convert(&val[2], data->accel_z, config->accel_range);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm20948_gyro_convert(val, data->gyro_x, config->gyro_range);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20948_gyro_convert(val, data->gyro_y, config->gyro_range);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20948_gyro_convert(val, data->gyro_z, config->gyro_range);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20948_gyro_convert(&val[0], data->gyro_x, config->gyro_range);
		icm20948_gyro_convert(&val[1], data->gyro_y, config->gyro_range);
		icm20948_gyro_convert(&val[2], data->gyro_z, config->gyro_range);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm20948_temp_convert(val, data->temp);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int icm20948_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	return -ENOTSUP;
}

static const struct sensor_driver_api icm20948_driver_api = {
	.sample_fetch = icm20948_sample_fetch,
	.channel_get = icm20948_channel_get,
	.attr_set = icm20948_attr_set,
#ifdef CONFIG_ICM20948_TRIGGER
	.trigger_set = icm20948_trigger_set,
#endif
};

static int icm20948_init(const struct device *dev)
{
	const struct icm20948_config *config = dev->config;
	int ret;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	ret = icm20948_setup(dev);
	if (ret) {
		LOG_ERR("Failed to setup device");
		return ret;
	}

#ifdef CONFIG_ICM20948_TRIGGER
	ret = icm20948_init_interrupt(dev);
	if (ret) {
		LOG_ERR("Failed to initialize interrupt");
		return ret;
	}
#endif

	LOG_INF("ICM20948 initialized successfully");
	return 0;
}

#define ICM20948_CONFIG(inst)						\
	{								\
		.spi = SPI_DT_SPEC_INST_GET(inst,			\
			SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8), 0),\
		.accel_range = DT_INST_PROP(inst, accel_range),		\
		.gyro_range = DT_INST_PROP(inst, gyro_range),		\
		IF_ENABLED(CONFIG_ICM20948_TRIGGER,			\
			(.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,	\
							      int_gpios,\
							      { 0 }),))	\
	}

#define ICM20948_DEFINE(inst)						\
	static struct icm20948_data icm20948_data_##inst;		\
									\
	static const struct icm20948_config icm20948_config_##inst =	\
		ICM20948_CONFIG(inst);					\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL,	\
				      &icm20948_data_##inst,		\
				      &icm20948_config_##inst,		\
				      POST_KERNEL,			\
				      CONFIG_SENSOR_INIT_PRIORITY,	\
				      &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM20948_DEFINE)
