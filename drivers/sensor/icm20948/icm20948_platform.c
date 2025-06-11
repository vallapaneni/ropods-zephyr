/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"

LOG_MODULE_REGISTER(icm20948_platform, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Get current time in microseconds
 * 
 * This function is required by lib/emd to provide timing information.
 * 
 * @return Current time in microseconds
 */
uint64_t inv_icm20948_get_time_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

/**
 * @brief Sleep for specified number of microseconds
 * 
 * This function is required by lib/emd for delays.
 * 
 * @param us Number of microseconds to sleep
 */
void inv_icm20948_sleep_us(int us)
{
	if (us > 0) {
		/* Use k_busy_wait for short delays to maintain precision */
		if (us < 1000) {
			k_busy_wait((uint32_t)us);
		} else {
			/* Use k_sleep for longer delays to be more system-friendly */
			k_sleep(K_USEC((uint32_t)us));
		}
	}
}
