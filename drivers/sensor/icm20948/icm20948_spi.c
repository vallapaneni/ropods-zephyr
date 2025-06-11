/*
 * Copyright (c) 2024 ROPODS
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include "icm20948.h"

LOG_MODULE_DECLARE(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

int icm20948_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct icm20948_config *config = dev->config;
	const struct spi_buf tx_buf = {
		.buf = &reg,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2,
	};

	reg |= ICM20948_SPI_READ;

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

int icm20948_spi_write(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct icm20948_config *config = dev->config;
	uint8_t tx_buffer[len + 1];
	
	tx_buffer[0] = reg;
	memcpy(&tx_buffer[1], data, len);

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = len + 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	return spi_transceive_dt(&config->spi, &tx, NULL);
}

int icm20948_spi_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	return icm20948_spi_write(dev, reg, &value, 1);
}

int icm20948_spi_read_reg(const struct device *dev, uint8_t reg, uint8_t *value)
{
	return icm20948_spi_read(dev, reg, value, 1);
}

int icm20948_select_bank(const struct device *dev, uint8_t bank)
{
	return icm20948_spi_write_reg(dev, ICM20948_REG_REG_BANK_SEL, bank);
}
