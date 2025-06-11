/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * ICM20948 sensor sample with separated interrupt control
 * 
 * This sample demonstrates how to use the ICM20948 sensor driver with
 * interrupt control separated from trigger setup. The application can
 * control when interrupts are enabled/disabled independently of the
 * trigger handler configuration.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"  /* For custom sensor attributes */

LOG_MODULE_REGISTER(icm20948_interrupt_control, LOG_LEVEL_INF);

/* Get the ICM20948 device from devicetree */
#define ICM20948_NODE DT_NODELABEL(icm20948)

#if !DT_NODE_EXISTS(ICM20948_NODE)
#error "ICM20948 devicetree node not found. Check your board configuration."
#endif

static const struct device *icm20948_dev = DEVICE_DT_GET(ICM20948_NODE);

/* Interrupt handling */
static volatile bool data_ready = false;
static int interrupt_count = 0;

static void icm20948_trigger_handler(const struct device *dev,
				     const struct sensor_trigger *trigger)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trigger);
	
	data_ready = true;
	interrupt_count++;
	LOG_INF("Data ready interrupt #%d received", interrupt_count);
}

static void print_sensor_data(const struct device *dev)
{
	struct sensor_value accel[3], gyro[3], mag[3];
	int ret;

	/* Fetch latest sensor data */
	ret = sensor_sample_fetch(dev);
	if (ret) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
		return;
	}

	/* Get accelerometer data */
	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret) {
		LOG_WRN("Failed to get accelerometer data: %d", ret);
	} else {
		LOG_INF("Accel: X=%.3f Y=%.3f Z=%.3f m/s²",
			sensor_value_to_double(&accel[0]),
			sensor_value_to_double(&accel[1]),
			sensor_value_to_double(&accel[2]));
	}

	/* Get gyroscope data */
	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (ret) {
		LOG_WRN("Failed to get gyroscope data: %d", ret);
	} else {
		LOG_INF("Gyro:  X=%.3f Y=%.3f Z=%.3f rad/s",
			sensor_value_to_double(&gyro[0]),
			sensor_value_to_double(&gyro[1]),
			sensor_value_to_double(&gyro[2]));
	}

	/* Get magnetometer data */
	ret = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);
	if (ret) {
		LOG_WRN("Failed to get magnetometer data: %d", ret);
	} else {
		LOG_INF("Mag:   X=%.3f Y=%.3f Z=%.3f gauss",
			sensor_value_to_double(&mag[0]),
			sensor_value_to_double(&mag[1]),
			sensor_value_to_double(&mag[2]));
	}
}

static int set_interrupt_enable(const struct device *dev, bool enable)
{
	struct sensor_value val;
	int ret;

	val.val1 = enable ? 1 : 0;
	val.val2 = 0;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to %s interrupts: %d", enable ? "enable" : "disable", ret);
		return ret;
	}

	LOG_INF("Interrupts %s", enable ? "enabled" : "disabled");
	return 0;
}

static int get_interrupt_enable(const struct device *dev)
{
	struct sensor_value val;
	int ret;

	ret = sensor_attr_get(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_INTERRUPT_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get interrupt enable state: %d", ret);
		return ret;
	}

	LOG_INF("Interrupt enable state: %s", val.val1 ? "enabled" : "disabled");
	return val.val1;
}

int main(void)
{
	struct sensor_trigger trigger = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	int ret;

	LOG_INF("ICM20948 Sensor Sample with Interrupt Control");
	LOG_INF("==============================================");

	/* Check if device is ready */
	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -1;
	}

	LOG_INF("ICM20948 device is ready");

	/* Phase 1: Set up trigger handler (but don't enable interrupts yet) */
	LOG_INF("\n--- Phase 1: Setting up trigger handler ---");
	ret = sensor_trigger_set(icm20948_dev, &trigger, icm20948_trigger_handler);
	if (ret) {
		LOG_ERR("Failed to set trigger handler: %d", ret);
		return -1;
	}

	LOG_INF("Trigger handler set successfully");

	/* Check initial interrupt state */
	get_interrupt_enable(icm20948_dev);

	/* Phase 2: Polling mode (interrupts disabled) */
	LOG_INF("\n--- Phase 2: Polling mode (5 seconds) ---");
	for (int i = 0; i < 5; i++) {
		print_sensor_data(icm20948_dev);
		k_sleep(K_SECONDS(1));
	}

	LOG_INF("Interrupt count during polling: %d", interrupt_count);

	/* Phase 3: Enable interrupts and use interrupt-driven mode */
	LOG_INF("\n--- Phase 3: Enabling interrupts ---");
	ret = set_interrupt_enable(icm20948_dev, true);
	if (ret) {
		return -1;
	}

	/* Verify interrupt state */
	get_interrupt_enable(icm20948_dev);

	LOG_INF("\n--- Phase 4: Interrupt-driven mode (10 seconds) ---");
	uint32_t start_time = k_uptime_get_32();
	while (k_uptime_get_32() - start_time < 10000) { /* 10 seconds */
		if (data_ready) {
			data_ready = false;
			print_sensor_data(icm20948_dev);
		}
		k_sleep(K_MSEC(10)); /* Small delay to prevent busy waiting */
	}

	LOG_INF("Total interrupts received: %d", interrupt_count);

	/* Phase 5: Disable interrupts and return to polling */
	LOG_INF("\n--- Phase 5: Disabling interrupts ---");
	ret = set_interrupt_enable(icm20948_dev, false);
	if (ret) {
		return -1;
	}

	/* Verify interrupt state */
	get_interrupt_enable(icm20948_dev);

	LOG_INF("\n--- Phase 6: Back to polling mode (5 seconds) ---");
	int count_before_disable = interrupt_count;
	for (int i = 0; i < 5; i++) {
		print_sensor_data(icm20948_dev);
		k_sleep(K_SECONDS(1));
	}

	LOG_INF("Interrupt count after disable: %d (should be %d)", 
		interrupt_count, count_before_disable);

	/* Phase 7: Re-enable interrupts to demonstrate control */
	LOG_INF("\n--- Phase 7: Re-enabling interrupts (5 seconds) ---");
	ret = set_interrupt_enable(icm20948_dev, true);
	if (ret) {
		return -1;
	}

	start_time = k_uptime_get_32();
	while (k_uptime_get_32() - start_time < 5000) { /* 5 seconds */
		if (data_ready) {
			data_ready = false;
			print_sensor_data(icm20948_dev);
		}
		k_sleep(K_MSEC(10));
	}

	LOG_INF("Final interrupt count: %d", interrupt_count);

	/* Cleanup: disable interrupts */
	set_interrupt_enable(icm20948_dev, false);

	LOG_INF("\n=== Sample completed successfully ===");
	LOG_INF("This sample demonstrated:");
	LOG_INF("1. Setting trigger handler without enabling interrupts");
	LOG_INF("2. Polling mode operation");
	LOG_INF("3. Enabling interrupts via attribute");
	LOG_INF("4. Interrupt-driven operation");
	LOG_INF("5. Disabling interrupts via attribute");
	LOG_INF("6. Returning to polling mode");
	LOG_INF("7. Dynamic interrupt control");

	return 0;
}
