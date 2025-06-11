/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "icm20948.h"

#ifdef CONFIG_ICM20948_TRIGGER

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(icm20948, CONFIG_SENSOR_LOG_LEVEL);

static void icm20948_thread_cb(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	int ret;

	/* Automatically fetch sensor data when interrupt occurs */
	ret = inv_icm20948_poll_sensor(&data->icm_device, data, icm20948_sensor_event_cb);
	if (ret != 0) {
		LOG_ERR("Failed to poll sensor data in interrupt: %d", ret);
		return;
	}

	/* Mark data as fresh from interrupt */
	data->data_ready_from_int = true;

	/* 
	 * Note: data_ready_handler is called asynchronously from icm20948_sensor_event_cb
	 * No need to call it here as the callback mechanism handles notification
	 */
}

static void icm20948_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	struct icm20948_data *data = CONTAINER_OF(cb, struct icm20948_data, gpio_cb);
	const struct device *sensor = data->dev;
	(void)sensor; /* Suppress unused variable warning */

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
		icm20948_thread_cb(data->dev);
	}
}
#endif

#ifdef CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD
static void icm20948_work_cb(struct k_work *work)
{
	struct icm20948_data *data = CONTAINER_OF(work, struct icm20948_data, work);

	icm20948_thread_cb(data->dev);
}
#endif

int icm20948_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	if (!cfg->int_gpio.port) {
		return -ENOTSUP;
	}

	/* Set the handler and trigger - do NOT enable interrupt automatically */
	data->data_ready_handler = handler;
	data->data_ready_trigger = trig;

	/* If handler is NULL, disable interrupt if it was previously enabled */
	if (handler == NULL && data->interrupt_enabled) {
		int ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);
		if (ret < 0) {
			LOG_ERR("Failed to disable GPIO interrupt");
			return ret;
		}
		data->interrupt_enabled = false;
	}

	LOG_DBG("Trigger handler %s, interrupt state: %s", 
		handler ? "set" : "cleared",
		data->interrupt_enabled ? "enabled" : "disabled");

	return 0;
}

int icm20948_init_interrupt(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *cfg = dev->config;
	int ret;

	/* Store device reference for callback */
	data->dev = dev;

	if (!cfg->int_gpio.port) {
		LOG_DBG("No interrupt GPIO specified");
		return 0;
	}

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt GPIO");
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, icm20948_gpio_callback,
			   BIT(cfg->int_gpio.pin));

	ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add GPIO callback");
		return ret;
	}

#if defined(CONFIG_ICM20948_TRIGGER_OWN_THREAD)
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_ICM20948_THREAD_STACK_SIZE,
			icm20948_thread, data,
			NULL, NULL, K_PRIO_COOP(CONFIG_SENSOR_INIT_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
	k_work_init(&data->work, icm20948_work_cb);
#endif

	LOG_DBG("ICM20948 interrupt initialized");
	return 0;
}

#endif /* CONFIG_ICM20948_TRIGGER */
