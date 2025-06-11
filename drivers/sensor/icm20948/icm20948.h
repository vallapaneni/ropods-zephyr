/*
 * Copyright (c) 2024 ROPODS
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_ICM20948_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_ICM20948_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

/* ICM20948 register addresses */
#define ICM20948_REG_WHO_AM_I           0x00
#define ICM20948_REG_USER_CTRL          0x03
#define ICM20948_REG_LP_CONFIG          0x05
#define ICM20948_REG_PWR_MGMT_1         0x06
#define ICM20948_REG_PWR_MGMT_2         0x07
#define ICM20948_REG_INT_PIN_CFG        0x0F
#define ICM20948_REG_INT_ENABLE         0x10
#define ICM20948_REG_INT_ENABLE_1       0x11
#define ICM20948_REG_INT_ENABLE_2       0x12
#define ICM20948_REG_INT_ENABLE_3       0x13
#define ICM20948_REG_I2C_MST_STATUS     0x17
#define ICM20948_REG_INT_STATUS         0x19
#define ICM20948_REG_INT_STATUS_1       0x1A
#define ICM20948_REG_INT_STATUS_2       0x1B
#define ICM20948_REG_INT_STATUS_3       0x1C
#define ICM20948_REG_DELAY_TIMEH        0x28
#define ICM20948_REG_DELAY_TIMEL        0x29
#define ICM20948_REG_ACCEL_XOUT_H       0x2D
#define ICM20948_REG_ACCEL_XOUT_L       0x2E
#define ICM20948_REG_ACCEL_YOUT_H       0x2F
#define ICM20948_REG_ACCEL_YOUT_L       0x30
#define ICM20948_REG_ACCEL_ZOUT_H       0x31
#define ICM20948_REG_ACCEL_ZOUT_L       0x32
#define ICM20948_REG_GYRO_XOUT_H        0x33
#define ICM20948_REG_GYRO_XOUT_L        0x34
#define ICM20948_REG_GYRO_YOUT_H        0x35
#define ICM20948_REG_GYRO_YOUT_L        0x36
#define ICM20948_REG_GYRO_ZOUT_H        0x37
#define ICM20948_REG_GYRO_ZOUT_L        0x38
#define ICM20948_REG_TEMP_OUT_H         0x39
#define ICM20948_REG_TEMP_OUT_L         0x3A
#define ICM20948_REG_EXT_SLV_SENS_DATA_00   0x3B
#define ICM20948_REG_FIFO_EN_1          0x66
#define ICM20948_REG_FIFO_EN_2          0x67
#define ICM20948_REG_FIFO_RST           0x68
#define ICM20948_REG_FIFO_MODE          0x69
#define ICM20948_REG_FIFO_COUNTH        0x70
#define ICM20948_REG_FIFO_COUNTL        0x71
#define ICM20948_REG_FIFO_R_W           0x72
#define ICM20948_REG_DATA_RDY_STATUS    0x74
#define ICM20948_REG_FIFO_CFG           0x76
#define ICM20948_REG_REG_BANK_SEL       0x7F

/* User Bank 1 registers */
#define ICM20948_REG_SELF_TEST_X_GYRO   0x02
#define ICM20948_REG_SELF_TEST_Y_GYRO   0x03
#define ICM20948_REG_SELF_TEST_Z_GYRO   0x04
#define ICM20948_REG_SELF_TEST_X_ACCEL  0x0E
#define ICM20948_REG_SELF_TEST_Y_ACCEL  0x0F
#define ICM20948_REG_SELF_TEST_Z_ACCEL  0x10
#define ICM20948_REG_XA_OFFS_H          0x14
#define ICM20948_REG_XA_OFFS_L          0x15
#define ICM20948_REG_YA_OFFS_H          0x17
#define ICM20948_REG_YA_OFFS_L          0x18
#define ICM20948_REG_ZA_OFFS_H          0x1A
#define ICM20948_REG_ZA_OFFS_L          0x1B
#define ICM20948_REG_TIMEBASE_CORRECTION_PLL    0x28

/* User Bank 2 registers */
#define ICM20948_REG_GYRO_SMPLRT_DIV    0x00
#define ICM20948_REG_GYRO_CONFIG_1      0x01
#define ICM20948_REG_GYRO_CONFIG_2      0x02
#define ICM20948_REG_XG_OFFS_USRH       0x03
#define ICM20948_REG_XG_OFFS_USRL       0x04
#define ICM20948_REG_YG_OFFS_USRH       0x05
#define ICM20948_REG_YG_OFFS_USRL       0x06
#define ICM20948_REG_ZG_OFFS_USRH       0x07
#define ICM20948_REG_ZG_OFFS_USRL       0x08
#define ICM20948_REG_ODR_ALIGN_EN       0x09
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_REG_ACCEL_INTEL_CTRL   0x12
#define ICM20948_REG_ACCEL_WOM_THR      0x13
#define ICM20948_REG_ACCEL_CONFIG       0x14
#define ICM20948_REG_ACCEL_CONFIG_2     0x15
#define ICM20948_REG_FSYNC_CONFIG       0x52
#define ICM20948_REG_TEMP_CONFIG        0x53
#define ICM20948_REG_MOD_CTRL_USR       0x54

/* User Bank 3 registers */
#define ICM20948_REG_I2C_MST_ODR_CONFIG 0x00
#define ICM20948_REG_I2C_MST_CTRL       0x01
#define ICM20948_REG_I2C_MST_DELAY_CTRL 0x02
#define ICM20948_REG_I2C_SLV0_ADDR      0x03
#define ICM20948_REG_I2C_SLV0_REG       0x04
#define ICM20948_REG_I2C_SLV0_CTRL      0x05
#define ICM20948_REG_I2C_SLV0_DO        0x06

/* Register values */
#define ICM20948_CHIP_ID                0xEA
#define ICM20948_SPI_READ               0x80

/* Bank selection */
#define ICM20948_BANK_0                 0x00
#define ICM20948_BANK_1                 0x10
#define ICM20948_BANK_2                 0x20
#define ICM20948_BANK_3                 0x30

/* Power management */
#define ICM20948_PWR_MGMT_1_DEVICE_RESET    0x80
#define ICM20948_PWR_MGMT_1_SLEEP           0x40
#define ICM20948_PWR_MGMT_1_LP_EN           0x20
#define ICM20948_PWR_MGMT_1_TEMP_DIS        0x08
#define ICM20948_PWR_MGMT_1_CLK_SEL_MASK    0x07

/* User control */
#define ICM20948_USER_CTRL_DMP_EN           0x80
#define ICM20948_USER_CTRL_FIFO_EN          0x40
#define ICM20948_USER_CTRL_I2C_MST_EN       0x20
#define ICM20948_USER_CTRL_I2C_IF_DIS       0x10
#define ICM20948_USER_CTRL_DMP_RST          0x08
#define ICM20948_USER_CTRL_SRAM_RST         0x04
#define ICM20948_USER_CTRL_I2C_MST_RST      0x02

/* Gyroscope configuration */
#define ICM20948_GYRO_FS_SEL_250DPS     0x00
#define ICM20948_GYRO_FS_SEL_500DPS     0x02
#define ICM20948_GYRO_FS_SEL_1000DPS    0x04
#define ICM20948_GYRO_FS_SEL_2000DPS    0x06
#define ICM20948_GYRO_FS_SEL_MASK       0x06

/* Accelerometer configuration */
#define ICM20948_ACCEL_FS_SEL_2G        0x00
#define ICM20948_ACCEL_FS_SEL_4G        0x02
#define ICM20948_ACCEL_FS_SEL_8G        0x04
#define ICM20948_ACCEL_FS_SEL_16G       0x06
#define ICM20948_ACCEL_FS_SEL_MASK      0x06

/* Interrupt configuration */
#define ICM20948_INT_ACTL               0x80
#define ICM20948_INT_OPEN               0x40
#define ICM20948_INT_LATCH_EN           0x20
#define ICM20948_INT_ANYRD_2CLEAR       0x10
#define ICM20948_INT_ACTL_FSYNC         0x08
#define ICM20948_INT_FSYNC_MODE_EN      0x04
#define ICM20948_INT_BYPASS_EN          0x02

/* Magnetometer (AK09916) registers */
#define AK09916_REG_WIA1                0x00
#define AK09916_REG_WIA2                0x01
#define AK09916_REG_ST1                 0x10
#define AK09916_REG_HXL                 0x11
#define AK09916_REG_HXH                 0x12
#define AK09916_REG_HYL                 0x13
#define AK09916_REG_HYH                 0x14
#define AK09916_REG_HZL                 0x15
#define AK09916_REG_HZH                 0x16
#define AK09916_REG_ST2                 0x18
#define AK09916_REG_CNTL2               0x31
#define AK09916_REG_CNTL3               0x32

#define AK09916_CHIP_ID                 0x4809
#define AK09916_I2C_ADDR                0x0C
#define AK09916_MODE_POWER_DOWN         0x00
#define AK09916_MODE_SINGLE_MEASURE     0x01
#define AK09916_MODE_CONT_MEASURE_1     0x02
#define AK09916_MODE_CONT_MEASURE_2     0x04
#define AK09916_MODE_CONT_MEASURE_3     0x06
#define AK09916_MODE_CONT_MEASURE_4     0x08
#define AK09916_MODE_SELF_TEST          0x10

struct icm20948_data {
	int16_t accel_x, accel_y, accel_z;
	int16_t gyro_x, gyro_y, gyro_z;
	int16_t magn_x, magn_y, magn_z;
	int16_t temp;

#ifdef CONFIG_ICM20948_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t data_ready_handler;
	const struct sensor_trigger *data_ready_trigger;

#ifdef CONFIG_ICM20948_TRIGGER_OWN_THREAD
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ICM20948_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_ICM20948_TRIGGER */
};

struct icm20948_config {
	struct spi_dt_spec spi;
	uint16_t accel_range;
	uint16_t gyro_range;
#ifdef CONFIG_ICM20948_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif
};

#ifdef CONFIG_ICM20948_TRIGGER
int icm20948_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);

int icm20948_init_interrupt(const struct device *dev);
#endif

/* SPI Communication Functions */
int icm20948_select_bank(const struct device *dev, uint8_t bank);
int icm20948_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len);
int icm20948_spi_write(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len);
int icm20948_spi_read_reg(const struct device *dev, uint8_t reg, uint8_t *value);
int icm20948_spi_write_reg(const struct device *dev, uint8_t reg, uint8_t value);

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM20948_ICM20948_H_ */
