/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

/* Custom sensor attributes for ICM20948 */
enum icm20948_sensor_attribute {
	/** Enable/disable sensors with eMD sensor mask (0 = disable all, non-zero = enable based on mask) 
	 *  Supported sensors: INV_ICM20948_SENSOR_ACCELEROMETER, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 
	 *                     INV_ICM20948_SENSOR_ROTATION_VECTOR, INV_ICM20948_SENSOR_LINEAR_ACCELERATION
	 *  
	 *  The driver efficiently manages sensor state transitions by only enabling/disabling sensors that actually changed. */
	SENSOR_ATTR_ICM20948_SENSOR_ENABLE = SENSOR_ATTR_PRIV_START,
	/** Trigger self-test (write-only, any non-zero value triggers test) */
	SENSOR_ATTR_ICM20948_SELF_TEST,
};

/** 
 * Generic sensor attribute to get data length for a specific channel
 * (read-only, returns number of sensor_value elements needed)
 * 
 * This is defined as a generic attribute that can be used by any sensor driver.
 * Usage: sensor_attr_get(dev, channel, SENSOR_ATTR_DATA_LENGTH, &length);
 */
#define SENSOR_ATTR_DATA_LENGTH		(SENSOR_ATTR_PRIV_START + 100)

/* Custom sensor channels for ICM20948 */
enum icm20948_sensor_channel {
	/** 9-axis rotation vector (quaternion: accel + gyro + mag) - w,x,y,z,accuracy format */
	SENSOR_CHAN_ICM20948_ROTATION_VECTOR = SENSOR_CHAN_PRIV_START,
	/** 6-axis game rotation vector (quaternion: accel + gyro) - w,x,y,z,accuracy format */
	SENSOR_CHAN_ICM20948_GAME_ROTATION_VECTOR,
	/** Linear acceleration (gravity removed) - x,y,z,accuracy format in m/s² */
	SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION,
	
	/* Packed channels without accuracy conversion */
	/** 6-axis game rotation vector (quaternion: accel + gyro) - w,x,y,z format (no accuracy) */
	SENSOR_CHAN_GAME_ROTATION_VECTOR_PACKED,
	/** 9-axis rotation vector (quaternion: accel + gyro + mag) - w,x,y,z format (no accuracy) */
	SENSOR_CHAN_ROTATION_VECTOR_PACKED,
	/** Linear acceleration (gravity removed) - x,y,z format in m/s² (no accuracy) */
	SENSOR_CHAN_ICM20948_LINEAR_ACCELERATION_PACKED,
	
	/* Accuracy flags channel */
	/** All accuracy flags packed - accel_accuracy, gyro_accuracy, mag_accuracy, rv_accuracy */
	SENSOR_CHAN_ACCURACY_FLAGS_PACKED,
};

/* Prevent macro redefinition warnings from lib/emd */
#ifdef MAX
#undef MAX
#endif
#ifdef MIN  
#undef MIN
#endif
#ifdef BIT
#undef BIT
#endif
#ifdef MSEC_PER_SEC
#undef MSEC_PER_SEC
#endif
#ifdef NSEC_PER_MSEC
#undef NSEC_PER_MSEC
#endif
#ifdef NSEC_PER_SEC
#undef NSEC_PER_SEC
#endif

/* Include lib/emd headers */
#include "Invn/Devices/Drivers/ICM20948/Icm20948.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948Defs.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948Transport.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948Setup.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948AuxCompassAkm.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948MPUFifoControl.h"
#include "Invn/EmbUtils/InvError.h"

/* Restore Zephyr macros */
#include <zephyr/sys/util.h>

/* ICM-20948 chip ID */
#define ICM20948_CHIP_ID            0xEA

/* Maximum number of bytes that can be read from FIFO */
#define ICM20948_FIFO_MAX_COUNT     512

struct icm20948_data {
	/* eMD driver instance */
	inv_icm20948_t icm_device;
	
	/* Sensor data */
	int16_t accel_x, accel_y, accel_z;
	int16_t gyro_x, gyro_y, gyro_z;
	int16_t mag_x, mag_y, mag_z;
	int16_t temp;
	
	/* Raw sensor data arrays (from eMD library callbacks - already transformed) */
	int16_t accel_raw[3];
	int16_t gyro_raw[3];
	int16_t mag_raw[3];
	int16_t temp_raw;
	
	/* Quaternion data (from eMD DMP - Q30 format converted to float) */
	float quat6_raw[4];  /* Game rotation vector (6-axis: accel + gyro) w,x,y,z */
	float quat9_raw[4];  /* Rotation vector (9-axis: accel + gyro + mag) w,x,y,z */
	
	/* Linear acceleration data (from eMD DMP - gravity removed) */
	float linear_accel_raw[3]; /* Linear acceleration in m/s² */
	
	/* Accuracy data for quaternion sensors */
	uint8_t accel_accuracy;     /* Accelerometer accuracy (0-3) */
	uint8_t gyro_accuracy;      /* Gyroscope accuracy (0-3) */
	uint8_t mag_accuracy;       /* Magnetometer accuracy (0-3) */
	float rv_accuracy;          /* Rotation vector accuracy (converted from Q29) */
	
	/* Interrupt-driven data freshness tracking */
	volatile bool data_ready_from_int;
	
	/* Interrupt control */
	bool interrupt_enabled;
	
	/* Sensor enable mask */
	uint32_t sensor_enable_mask;
	
	/* Configuration */
	uint8_t accel_range;
	uint8_t gyro_range;
	uint8_t accel_dlpf;
	uint8_t gyro_dlpf;
	
#ifdef CONFIG_ICM20948_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t data_ready_handler;
	const struct sensor_trigger *data_ready_trigger;
	
#if defined(CONFIG_ICM20948_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ICM20948_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_ICM20948_TRIGGER */
};

/* Transport API for bus abstraction */
struct icm20948_transport_api {
	int (*read_reg)(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len);
	int (*write_reg)(const struct device *dev, uint8_t reg, const uint8_t *data, uint16_t len);
	/* Single-byte convenience functions - UNUSED, commented out to reduce code size */
	/* int (*read_reg_single)(const struct device *dev, uint8_t reg, uint8_t *data); */
	/* int (*write_reg_single)(const struct device *dev, uint8_t reg, uint8_t data); */
	int (*emd_read)(void *context, uint8_t reg, uint8_t *data, uint32_t len);
	int (*emd_write)(void *context, uint8_t reg, const uint8_t *data, uint32_t len);
};

struct icm20948_config {
	struct spi_dt_spec spi;
	
	const struct icm20948_transport_api *transport_api;
	
#ifdef CONFIG_ICM20948_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif
	uint8_t accel_range;
	uint8_t gyro_range;
	uint8_t accel_dlpf;
	uint8_t gyro_dlpf;
	
	/* 3x3 mounting matrix for sensor orientation correction */
	int8_t mount_matrix[9];
};

/* Function prototypes */
#ifdef CONFIG_ICM20948_TRIGGER
int icm20948_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);

int icm20948_init_interrupt(const struct device *dev);
#endif

/* Transport layer APIs */
extern const struct icm20948_transport_api icm20948_spi_api;

int icm20948_spi_init(const struct device *dev);

/* Common functions */
/* These functions are implemented as static in icm20948.c */
int icm20948_init(const struct device *dev);

#ifdef CONFIG_ICM20948_TRIGGER
int icm20948_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);
int icm20948_init_interrupt(const struct device *dev);
#endif

/* eMD library sensor event callback function (shared between modules) */
void icm20948_sensor_event_cb(void * context, enum inv_icm20948_sensor sensor, uint64_t timestamp, const void * data, const void *arg);

extern const struct sensor_driver_api icm20948_driver_api;

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM20948_H_ */
