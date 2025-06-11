/*
 * Copyright (c) 2024 ROPODS
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm20948

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "icm20948.h"

LOG_MODULE_DECLARE(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

static void icm20948_handle_interrupt(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	uint8_t int_status;
	int ret;

	ret = icm20948_select_bank(dev, ICM20948_BANK_0);
	if (ret) {
		LOG_ERR("Failed to select bank 0");
		return;
	}

	ret = icm20948_spi_read(dev, ICM20948_REG_INT_STATUS, &int_status, 1);
	if (ret) {
		LOG_ERR("Failed to read interrupt status");
		return;
	}

	if (int_status & 0x01) { /* Data ready interrupt */
		if (data->data_ready_handler) {
			data->data_ready_handler(dev, data->data_ready_trigger);
		}
	}
}

static void icm20948_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb,
				   uint32_t pins)
{
	struct icm20948_data *data =
		CONTAINER_OF(cb, struct icm20948_data, gpio_cb);

	ARG_UNUSED(pins);

#if defined(CONFIG_ICM20948_TRIGGER_OWN_THREAD)
	k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

#ifdef CONFIG_ICM20948_TRIGGER_OWN_THREAD
static void icm20948_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct icm20948_data *data = p1;

	while (1) {
		k_sem_take(&data->gpio_sem, K_FOREVER);
		icm20948_handle_interrupt(data->dev);
	}
}
#endif

#ifdef CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD
static void icm20948_work_cb(struct k_work *work)
{
	struct icm20948_data *data =
		CONTAINER_OF(work, struct icm20948_data, work);

	icm20948_handle_interrupt(data->dev);
}
#endif

int icm20948_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *config = dev->config;
	uint8_t val;
	int ret;

	if (!config->int_gpio.port) {
		return -ENOTSUP;
	}

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		data->data_ready_handler = handler;
		data->data_ready_trigger = trig;

		ret = icm20948_select_bank(dev, ICM20948_BANK_0);
		if (ret) {
			return ret;
		}

		/* Enable/disable data ready interrupt */
		val = handler ? 0x01 : 0x00;
		ret = icm20948_spi_write(dev, ICM20948_REG_INT_ENABLE, &val, 1);
		if (ret) {
			return ret;
		}
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}

	return 0;
}

int icm20948_init_interrupt(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *config = dev->config;
	uint8_t val;
	int ret;

	if (!config->int_gpio.port) {
		LOG_DBG("No interrupt GPIO configured");
		return 0;
	}

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	data->dev = dev;

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Failed to configure interrupt GPIO");
		return ret;
	}

	gpio_init_callback(&data->gpio_cb,
			   icm20948_gpio_callback,
			   BIT(config->int_gpio.pin));

	ret = gpio_add_callback(config->int_gpio.port, &data->gpio_cb);
	if (ret) {
		LOG_ERR("Failed to add GPIO callback");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->int_gpio,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure interrupt");
		return ret;
	}

#if defined(CONFIG_ICM20948_TRIGGER_OWN_THREAD)
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_ICM20948_THREAD_STACK_SIZE,
			icm20948_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_ICM20948_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
	k_work_init(&data->work, icm20948_work_cb);
#endif

	/* Configure interrupt pin */
	ret = icm20948_select_bank(dev, ICM20948_BANK_0);
	if (ret) {
		return ret;
	}

	/* Configure interrupt pin as push-pull, active high, latched */
	val = ICM20948_INT_LATCH_EN | ICM20948_INT_ANYRD_2CLEAR;
	ret = icm20948_spi_write(dev, ICM20948_REG_INT_PIN_CFG, &val, 1);
	if (ret) {
		return ret;
	}

	LOG_DBG("Interrupt initialized successfully");
	return 0;
}
