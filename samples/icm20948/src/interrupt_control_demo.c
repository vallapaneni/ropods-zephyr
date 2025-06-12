/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * ICM20948 sensor sample with sensor enable control
 * 
 * This sample demonstrates how to use the ICM20948 sensor driver with
 * the new sensor enable functionality that controls both sensors and
 * interrupts through a single mask-based attribute.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"  /* For custom sensor attributes */

LOG_MODULE_REGISTER(icm20948_sensor_enable_control, LOG_LEVEL_INF);

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

static int set_sensor_enable(const struct device *dev, uint32_t sensor_mask)
{
	struct sensor_value val;
	int ret;

	val.val1 = sensor_mask;
	val.val2 = 0;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to set sensor enable mask to 0x%08x: %d", sensor_mask, ret);
		return ret;
	}

	if (sensor_mask == 0) {
		LOG_INF("All sensors disabled");
	} else {
		LOG_INF("Sensors enabled with mask: 0x%08x", sensor_mask);
	}
	return 0;
}

static int get_sensor_enable(const struct device *dev)
{
	struct sensor_value val;
	int ret;

	ret = sensor_attr_get(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_ICM20948_SENSOR_ENABLE, &val);
	if (ret) {
		LOG_ERR("Failed to get sensor enable state: %d", ret);
		return ret;
	}

	LOG_INF("Sensor enable state: 0x%08x", (uint32_t)val.val1);
	return (uint32_t)val.val1;
}

int main(void)
{
	struct sensor_trigger trigger = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	int ret;

	LOG_INF("ICM20948 Sensor Sample with Sensor Enable Control");
	LOG_INF("==================================================");

	/* Check if device is ready */
	if (!device_is_ready(icm20948_dev)) {
		LOG_ERR("ICM20948 device not ready");
		return -1;
	}

	LOG_INF("ICM20948 device is ready");

	/* Phase 1: Set up trigger handler (but don't enable sensors yet) */
	LOG_INF("\n--- Phase 1: Setting up trigger handler ---");
	ret = sensor_trigger_set(icm20948_dev, &trigger, icm20948_trigger_handler);
	if (ret) {
		LOG_ERR("Failed to set trigger handler: %d", ret);
		return -1;
	}

	LOG_INF("Trigger handler set successfully");

	/* Check initial sensor enable state */
	get_sensor_enable(icm20948_dev);

	/* Phase 2: Polling mode (all sensors disabled) */
	LOG_INF("\n--- Phase 2: Polling mode (all sensors disabled) ---");
	for (int i = 0; i < 3; i++) {
		LOG_INF("Attempting to read sensor data (should fail)...");
		ret = sensor_sample_fetch(icm20948_dev);
		if (ret) {
			LOG_INF("✓ Expected: sensor_sample_fetch failed: %d (sensors disabled)", ret);
		} else {
			LOG_WRN("Unexpected: sensor_sample_fetch succeeded despite sensors being disabled");
		}
		k_sleep(K_SECONDS(1));
	}

	LOG_INF("Interrupt count during disabled state: %d", interrupt_count);

	/* Phase 3: Enable accelerometer only */
	LOG_INF("\n--- Phase 3: Enabling accelerometer only ---");
	ret = set_sensor_enable(icm20948_dev, BIT(INV_ICM20948_SENSOR_ACCELEROMETER));
	if (ret) {
		return -1;
	}

	/* Verify sensor enable state */
	get_sensor_enable(icm20948_dev);

	LOG_INF("\n--- Phase 4: Accelerometer-only mode (5 seconds) ---");
	uint32_t start_time = k_uptime_get_32();
	while (k_uptime_get_32() - start_time < 5000) { /* 5 seconds */
		if (data_ready) {
			data_ready = false;
			print_sensor_data(icm20948_dev);
		}
		k_sleep(K_MSEC(10)); /* Small delay to prevent busy waiting */
	}

	LOG_INF("Interrupts with accelerometer only: %d", interrupt_count);

	/* Phase 5: Enable accelerometer and game rotation vector */
	LOG_INF("\n--- Phase 5: Enabling accelerometer and game rotation vector ---");
	uint32_t accel_grv_mask = BIT(INV_ICM20948_SENSOR_ACCELEROMETER) | BIT(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
	ret = set_sensor_enable(icm20948_dev, accel_grv_mask);
	if (ret) {
		return -1;
	}

	/* Verify sensor enable state */
	get_sensor_enable(icm20948_dev);

	LOG_INF("\n--- Phase 6: Accelerometer + Game Rotation Vector mode (5 seconds) ---");
	start_time = k_uptime_get_32();
	while (k_uptime_get_32() - start_time < 5000) { /* 5 seconds */
		if (data_ready) {
			data_ready = false;
			print_sensor_data(icm20948_dev);
		}
		k_sleep(K_MSEC(10)); /* Small delay to prevent busy waiting */
	}

	LOG_INF("Total interrupts with accel+game rotation vector: %d", interrupt_count);

	/* Phase 7: Disable all sensors and return to polling */
	LOG_INF("\n--- Phase 7: Disabling all sensors ---");
	ret = set_sensor_enable(icm20948_dev, 0);
	if (ret) {
		return -1;
	}

	/* Verify sensor enable state */
	get_sensor_enable(icm20948_dev);

	LOG_INF("\n--- Phase 8: Back to disabled state (3 seconds) ---");
	int count_before_disable = interrupt_count;
	for (int i = 0; i < 3; i++) {
		LOG_INF("Attempting to read sensor data (should fail)...");
		ret = sensor_sample_fetch(icm20948_dev);
		if (ret) {
			LOG_INF("✓ Expected: sensor_sample_fetch failed: %d (sensors disabled)", ret);
		} else {
			LOG_WRN("Unexpected: sensor_sample_fetch succeeded despite sensors being disabled");
		}
		k_sleep(K_SECONDS(1));
	}

	LOG_INF("Interrupt count after disable: %d (should be %d)", 
		interrupt_count, count_before_disable);

	/* Phase 9: Re-enable accelerometer to demonstrate control */
	LOG_INF("\n--- Phase 9: Re-enabling accelerometer (3 seconds) ---");
	ret = set_sensor_enable(icm20948_dev, BIT(INV_ICM20948_SENSOR_ACCELEROMETER));
	if (ret) {
		return -1;
	}

	start_time = k_uptime_get_32();
	while (k_uptime_get_32() - start_time < 3000) { /* 3 seconds */
		if (data_ready) {
			data_ready = false;
			print_sensor_data(icm20948_dev);
		}
		k_sleep(K_MSEC(10));
	}

	LOG_INF("Final interrupt count: %d", interrupt_count);

	/* Cleanup: disable all sensors */
	set_sensor_enable(icm20948_dev, 0);

	LOG_INF("\n=== Sample completed successfully ===");
	LOG_INF("This sample demonstrated:");
	LOG_INF("1. Setting trigger handler without enabling sensors");
	LOG_INF("2. Sensor fetch failure when sensors are disabled");
	LOG_INF("3. Enabling individual sensors via mask");
	LOG_INF("4. Interrupt-driven operation when sensors are enabled");
	LOG_INF("5. Enabling multiple sensors simultaneously");
	LOG_INF("6. Disabling all sensors via zero mask");
	LOG_INF("7. Dynamic sensor enable/disable control");

	return 0;
}
