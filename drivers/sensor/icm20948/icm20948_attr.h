/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ICM20948_ATTR_H
#define ICM20948_ATTR_H

#include "icm20948.h"

/**
 * @brief Set sensor attributes for ICM20948
 *
 * @param dev Pointer to the device structure
 * @param chan Sensor channel to configure
 * @param attr Sensor attribute to set
 * @param val Pointer to the sensor value to set
 *
 * @return 0 on success, negative error code on failure
 */
int icm20948_attr_set(const struct device *dev, enum sensor_channel chan,
		      enum sensor_attribute attr, const struct sensor_value *val);

/**
 * @brief Get sensor attributes for ICM20948
 *
 * @param dev Pointer to the device structure
 * @param chan Sensor channel to query
 * @param attr Sensor attribute to get
 * @param val Pointer to store the retrieved sensor value
 *
 * @return 0 on success, negative error code on failure
 */
int icm20948_attr_get(const struct device *dev, enum sensor_channel chan,
		      enum sensor_attribute attr, struct sensor_value *val);

#endif /* ICM20948_ATTR_H */
