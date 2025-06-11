/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm20948

#include "icm20948.h"
#include "icm20948_attr.h"
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(icm20948, CONFIG_SENSOR_LOG_LEVEL);


#ifdef CONFIG_ICM20948_DMP
/* DMP firmware image */
static const uint8_t dmp3_image[] = {
#include "../../../lib/emd/EMD-App/src/ICM20948/icm20948_img.dmp3a.h"
};
#endif /* CONFIG_ICM20948_DMP */

/* Data ready callback from lib/emd */
void icm20948_sensor_event_cb(void * context, enum inv_icm20948_sensor sensor, uint64_t timestamp, const void * data, const void *arg)
{
	struct icm20948_data *drv_data = (struct icm20948_data *)context;
	
	switch(sensor) {
	case INV_ICM20948_SENSOR_ACCELEROMETER:
		if (data) {
			const short *accel_data = (const short *)data;
			drv_data->accel_raw[0] = accel_data[0];
			drv_data->accel_raw[1] = accel_data[1]; 
			drv_data->accel_raw[2] = accel_data[2];
		}
		break;
		
	case INV_ICM20948_SENSOR_GYROSCOPE:
		if (data) {
			const short *gyro_data = (const short *)data;
			drv_data->gyro_raw[0] = gyro_data[0];
			drv_data->gyro_raw[1] = gyro_data[1];
			drv_data->gyro_raw[2] = gyro_data[2];
		}
		break;
		
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		if (data) {
			const short *mag_data = (const short *)data;
			drv_data->mag_raw[0] = mag_data[0];
			drv_data->mag_raw[1] = mag_data[1];
			drv_data->mag_raw[2] = mag_data[2];
		}
		break;
		
	default:
		break;
	}
	
	/* Trigger data ready callback if configured */
#ifdef CONFIG_ICM20948_TRIGGER
	if (drv_data->data_ready_handler) {
		struct sensor_trigger trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};
		drv_data->data_ready_handler(drv_data->dev, &trig);
	}
#endif
}

/**
 * Apply mounting matrix to all relevant sensors using eMD library (following eMD example pattern)
 * 
 * @param dev Pointer to the device structure
 */
static void icm20948_apply_mounting_matrix(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *config = dev->config;
	float float_matrix[9];
	int ret;

	/* Convert integer mounting matrix to float for eMD library */
	for (int i = 0; i < 9; i++) {
		float_matrix[i] = (float)config->mount_matrix[i];
	}

	LOG_INF("Applying mounting matrix to sensors using eMD library");

	/* Apply mounting matrix to all sensor types (following eMD example pattern) */
	for (int sensor_type = 0; sensor_type < INV_ICM20948_SENSOR_MAX; sensor_type++) {
		ret = inv_icm20948_set_matrix(&data->icm_device, float_matrix, sensor_type);
		if (ret != 0) {
			LOG_WRN("Failed to set mounting matrix for sensor type %d: %d", sensor_type, ret);
		}
	}

}

#ifdef CONFIG_ICM20948_TRIGGER
#ifndef CONFIG_ICM20948_DMP
/**
 * Configure ICM20948 hardware to generate raw data ready interrupts
 * 
 * This function configures the ICM20948 hardware registers to generate
 * interrupts when raw sensor data is ready. These are separate from the
 * DMP-related interrupts (DMP interrupt and FIFO overflow) which are
 * automatically enabled by the eMD library's inv_icm20948_initialize_lower_driver()
 * function. Both interrupt types serve different purposes and complement each other:
 * - Raw data interrupts (configured here): for basic accelerometer/gyroscope data
 * - DMP interrupts (configured by eMD): for advanced features like quaternions,
 *   step counting, gesture recognition, and sensor fusion algorithms
 *
 * Note: This function only configures raw data interrupts when DMP is not enabled,
 * since DMP mode uses its own interrupt mechanisms.
 *
 * @param dev Pointer to the device structure
 * @return 0 on success, negative error code on failure
 */
static int icm20948_configure_raw_data_interrupts(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	uint8_t int_enable_1 = 0;
	int ret;

	LOG_INF("Configuring ICM20948 raw data interrupts (DMP disabled)");

	/* Read current INT_ENABLE_1 register value */
	ret = inv_icm20948_read_mems_reg(&data->icm_device, REG_INT_ENABLE_1, 1, &int_enable_1);
	if (ret != 0) {
		LOG_ERR("Failed to read INT_ENABLE_1 register: %d", ret);
		return -EIO;
	}

	/* Enable data ready interrupts for all relevant sensor channels
	 * Based on other TDK drivers and ICM20948 datasheet:
	 * - BIT_DATA_RDY_0_EN: Data ready interrupt for sensor set 0 (typically accel/gyro)
	 * - BIT_DATA_RDY_1_EN: Data ready interrupt for sensor set 1 (typically additional sensors)
	 * - BIT_DATA_RDY_2_EN: Data ready interrupt for sensor set 2 (typically compass/mag)
	 * - BIT_DATA_RDY_3_EN: Data ready interrupt for sensor set 3 (typically additional)
	 * 
	 * We enable data ready interrupts for the primary sensor sets we use:
	 * accel/gyro (0) and compass/magnetometer (2)
	 */
	int_enable_1 |= BIT_DATA_RDY_0_EN | BIT_DATA_RDY_2_EN;

	/* Write back the updated register value */
	ret = inv_icm20948_write_single_mems_reg(&data->icm_device, REG_INT_ENABLE_1, int_enable_1);
	if (ret != 0) {
		LOG_ERR("Failed to write INT_ENABLE_1 register: %d", ret);
		return -EIO;
	}

	LOG_INF("Raw data interrupts configured successfully (INT_ENABLE_1=0x%02X)", int_enable_1);
	
	return 0;
}
#endif /* CONFIG_ICM20948_DMP */
#endif /* CONFIG_ICM20948_TRIGGER */

static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct icm20948_data *data = dev->data;
	int ret;

#ifdef CONFIG_ICM20948_TRIGGER
	/* If we have fresh data from trigger (interrupt), no need to poll again - data fetch happens from trigger */
	if (data->data_ready_from_int) {
		data->data_ready_from_int = false; /* Reset flag after consuming */
		LOG_DBG("Using fresh data from interrupt");
		return 0;
	}
#endif

	/* Poll sensor data from lib/emd - mounting matrix is applied internally by eMD */
	ret = inv_icm20948_poll_sensor(&data->icm_device, data, icm20948_sensor_event_cb);
	if (ret != 0) {
		LOG_ERR("Failed to poll sensor data: %d", ret);
		return -EIO;
	}

	/* No need for software mounting matrix transformation - eMD library handles it */
	return 0;
}

static int icm20948_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *config = dev->config;
	(void)config; /* Suppress unused variable warning */

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		/* Convert accelerometer data to m/s^2 - eMD library applies mounting matrix internally */
		for (int i = 0; i < 3; i++) {
			/* Scale factor depends on configured range */
			int32_t scaled = data->accel_raw[i] * config->accel_range;
			val[i].val1 = scaled / 32768; /* 16-bit signed */
			val[i].val2 = (scaled % 32768) * 1000000 / 32768;
		}
		break;

	case SENSOR_CHAN_ACCEL_X:
		val->val1 = data->accel_raw[0] * config->accel_range / 32768;
		val->val2 = (data->accel_raw[0] * config->accel_range % 32768) * 1000000 / 32768;
		break;

	case SENSOR_CHAN_ACCEL_Y:
		val->val1 = data->accel_raw[1] * config->accel_range / 32768;
		val->val2 = (data->accel_raw[1] * config->accel_range % 32768) * 1000000 / 32768;
		break;

	case SENSOR_CHAN_ACCEL_Z:
		val->val1 = data->accel_raw[2] * config->accel_range / 32768;
		val->val2 = (data->accel_raw[2] * config->accel_range % 32768) * 1000000 / 32768;
		break;

	case SENSOR_CHAN_GYRO_XYZ:
		/* Convert gyroscope data to rad/s - eMD library applies mounting matrix internally */
		for (int i = 0; i < 3; i++) {
			int32_t scaled = data->gyro_raw[i] * config->gyro_range;
			val[i].val1 = scaled / 32768;
			val[i].val2 = (scaled % 32768) * 1000000 / 32768;
		}
		break;

	case SENSOR_CHAN_GYRO_X:
		val->val1 = data->gyro_raw[0] * config->gyro_range / 32768;
		val->val2 = (data->gyro_raw[0] * config->gyro_range % 32768) * 1000000 / 32768;
		break;

	case SENSOR_CHAN_GYRO_Y:
		val->val1 = data->gyro_raw[1] * config->gyro_range / 32768;
		val->val2 = (data->gyro_raw[1] * config->gyro_range % 32768) * 1000000 / 32768;
		break;

	case SENSOR_CHAN_GYRO_Z:
		val->val1 = data->gyro_raw[2] * config->gyro_range / 32768;
		val->val2 = (data->gyro_raw[2] * config->gyro_range % 32768) * 1000000 / 32768;
		break;

	case SENSOR_CHAN_MAGN_XYZ:
		/* Convert magnetometer data to Gauss - eMD library applies mounting matrix internally */
		for (int i = 0; i < 3; i++) {
			val[i].val1 = data->mag_raw[i] / 1000; /* Convert to Gauss */
			val[i].val2 = (data->mag_raw[i] % 1000) * 1000;
		}
		break;

	case SENSOR_CHAN_MAGN_X:
		val->val1 = data->mag_raw[0] / 1000;
		val->val2 = (data->mag_raw[0] % 1000) * 1000;
		break;

	case SENSOR_CHAN_MAGN_Y:
		val->val1 = data->mag_raw[1] / 1000;
		val->val2 = (data->mag_raw[1] % 1000) * 1000;
		break;

	case SENSOR_CHAN_MAGN_Z:
		val->val1 = data->mag_raw[2] / 1000;
		val->val2 = (data->mag_raw[2] % 1000) * 1000;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api icm20948_api = {
	.sample_fetch = icm20948_sample_fetch,
	.channel_get = icm20948_channel_get,
	.attr_set = icm20948_attr_set,
	.attr_get = icm20948_attr_get,
#ifdef CONFIG_ICM20948_TRIGGER
	.trigger_set = icm20948_trigger_set,
#endif
};

int icm20948_init(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *config = dev->config;
	int ret;

	data->dev = dev;

	/* Initialize interrupt control flag */
	data->interrupt_enabled = false;

	ret = icm20948_spi_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize transport: %d", ret);
		return ret;
	}

	/* Configure serif (serial interface) for lib/emd */
	data->icm_device.serif.context = (void *)dev;
	data->icm_device.serif.read_reg = config->transport_api->emd_read;
	data->icm_device.serif.write_reg = config->transport_api->emd_write;
	data->icm_device.serif.max_read = 256;  /* Maximum read burst size */
	data->icm_device.serif.max_write = 256; /* Maximum write burst size */
	data->icm_device.serif.is_spi = 1;

	/* 
	 * Note: inv_icm20948_reset_states() is NOT called implicitly by the eMD library.
	 * It's only used in example applications (like EMD-App/src/ICM20948/main.c) for
	 * explicit driver state initialization. The inv_icm20948_initialize() function
	 * performs its own internal initialization without calling reset_states().
	 * For multi-instance support, we manually configure the serif interface above
	 * instead of using inv_icm20948_reset_states() which has global state issues.
	 */

	/* Verify device communication by reading WHO_AM_I register */
	uint8_t whoami = 0;
	ret = inv_icm20948_get_whoami(&data->icm_device, &whoami);
	if (ret != 0) {
		LOG_ERR("Failed to read WHO_AM_I register: %d", ret);
		return -EIO;
	}
	
	if (whoami != ICM20948_CHIP_ID) {
		LOG_ERR("Invalid WHO_AM_I value: expected 0x%02X, got 0x%02X", 
			ICM20948_CHIP_ID, whoami);
		return -ENODEV;
	}
	
	LOG_INF("ICM20948 WHO_AM_I verified: 0x%02X", whoami);

	/* Log mounting matrix configuration */
	LOG_INF("Mounting matrix: [%d %d %d; %d %d %d; %d %d %d]",
		config->mount_matrix[0], config->mount_matrix[1], config->mount_matrix[2],
		config->mount_matrix[3], config->mount_matrix[4], config->mount_matrix[5],
		config->mount_matrix[6], config->mount_matrix[7], config->mount_matrix[8]);

	/* Register auxiliary compass (magnetometer) - MUST be called before inv_icm20948_initialize() */
	LOG_INF("Registering auxiliary compass (AK09916)");
	inv_icm20948_register_aux_compass(&data->icm_device, INV_ICM20948_COMPASS_ID_AK09916, 0x0C);

#ifdef CONFIG_ICM20948_DMP
	/* Initialize lib/emd with DMP firmware */
	LOG_INF("Loading DMP firmware (%zu bytes)", sizeof(dmp3_image));
	ret = inv_icm20948_initialize(&data->icm_device, dmp3_image, sizeof(dmp3_image));
	if (ret != 0) {
		LOG_ERR("Failed to initialize lib/emd with DMP firmware: %d", ret);
		return -EIO;
	}
	LOG_INF("DMP firmware loaded successfully");
#else
	/* Initialize lib/emd without DMP firmware */
	LOG_INF("Initializing without DMP firmware");
	ret = inv_icm20948_initialize(&data->icm_device, NULL, 0);
	if (ret != 0) {
		LOG_ERR("Failed to initialize lib/emd: %d", ret);
		return -EIO;
	}
#endif /* CONFIG_ICM20948_DMP */

	/* Initialize auxiliary sensors */
	LOG_INF("Initializing auxiliary compass");
	ret = inv_icm20948_initialize_auxiliary(&data->icm_device);
	if (ret != 0) {
		LOG_ERR("Failed to initialize auxiliary compass: %d", ret);
		LOG_WRN("Magnetometer may not be available");
		/* Don't fail initialization - continue without magnetometer */
	} else {
		LOG_INF("Auxiliary compass initialized successfully");
	}

	/* Apply mounting matrix to sensors using eMD library */
	icm20948_apply_mounting_matrix(dev);

	/* Enable sensors */
	uint32_t sensor_mask = INV_ICM20948_SENSOR_ACCELEROMETER | 
			       INV_ICM20948_SENSOR_GYROSCOPE |
			       INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;

	ret = inv_icm20948_enable_sensor(&data->icm_device, sensor_mask, 1);
	if (ret != 0) {
		LOG_ERR("Failed to enable sensors: %d", ret);
		return -EIO;
	}

#ifdef CONFIG_ICM20948_TRIGGER
	ret = icm20948_init_interrupt(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize triggers: %d", ret);
		return ret;
	}

#ifndef CONFIG_ICM20948_DMP

	/* Configure hardware to generate data ready interrupts */
	ret = icm20948_configure_raw_data_interrupts(dev);
	if (ret < 0) {
		LOG_ERR("Failed to configure raw data interrupts: %d", ret);
		return ret;
	}
#endif
#endif

	LOG_INF("ICM-20948 sensor initialized successfully with mounting matrix support");
	return 0;
}

/* Helper macro to get mounting matrix from devicetree with fallback to identity matrix */
#define ICM20948_GET_MOUNT_MATRIX(inst)						\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, mount_matrix),			\
		({								\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 0),		\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 1),		\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 2),		\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 3),		\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 4),		\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 5),		\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 6),		\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 7),		\
			DT_INST_PROP_BY_IDX(inst, mount_matrix, 8),		\
		}),								\
		({1, 0, 0, 0, 1, 0, 0, 0, 1}))

#define ICM20948_CONFIG(inst)						\
	{									\
									\
		.spi = SPI_DT_SPEC_INST_GET(inst,			\
			SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8), 0),\
		.transport_api = &icm20948_spi_api,				\
		IF_ENABLED(CONFIG_ICM20948_TRIGGER,				\
			(.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),))\
		.accel_range = DT_INST_PROP_OR(inst, accel_range, 2),		\
		.gyro_range = DT_INST_PROP_OR(inst, gyro_range, 250),		\
		.accel_dlpf = DT_INST_PROP_OR(inst, accel_dlpf, 6),		\
		.gyro_dlpf = DT_INST_PROP_OR(inst, gyro_dlpf, 6),		\
		.mount_matrix = ICM20948_GET_MOUNT_MATRIX(inst),		\
	}

#define ICM20948_DEFINE(inst)						\
	static struct icm20948_data icm20948_data_##inst;			\
	static const struct icm20948_config icm20948_config_##inst =		\
		ICM20948_CONFIG(inst);					\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL,		\
				     &icm20948_data_##inst,			\
				     &icm20948_config_##inst, POST_KERNEL,	\
				     CONFIG_SENSOR_INIT_PRIORITY,		\
				     &icm20948_api);

DT_INST_FOREACH_STATUS_OKAY(ICM20948_DEFINE)
