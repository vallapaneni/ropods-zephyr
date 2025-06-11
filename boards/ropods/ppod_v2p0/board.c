/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <hal/nrf_power.h>
#include <zephyr/drivers/gpio.h>
#include "ppod_v2p0_defs.h"

static int board_ppod_v2p0_init(void)
{
/* This code should not go to mcuboot as it does not need voltage change
 * Else this will affect if firmware sets different voltage from mcuboot, this will go on like infinite loop
 */
#ifndef CONFIG_MCUBOOT
	if ((nrf_power_mainregstatus_get(NRF_POWER) ==
	     NRF_POWER_MAINREGSTATUS_HIGH) &&
	    ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
	     (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))) {
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
			;
		}

		NRF_UICR->REGOUT0 =
		    (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
		    (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos);

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
			;
		}

		/* a reset is required for changes to take effect */
		NVIC_SystemReset();
	}
#endif
	return 0;
}

SYS_INIT(board_ppod_v2p0_init, PRE_KERNEL_1,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int pwr_ctrl_init(void)
{
	bool led_is_on = true;
	uint8_t cnt = 0;
	
	// Configure RGBs as outputs
	const struct device *rled_dev = DEVICE_DT_GET(DT_GPIO_CTLR(LEDR_NODE, gpios));
	if (!device_is_ready(rled_dev)) {
		return -ENODEV;
	}
	gpio_pin_configure(rled_dev, LEDR_PIN, GPIO_OUTPUT | LEDR_FLAGS);
	
	const struct device *gled_dev = DEVICE_DT_GET(DT_GPIO_CTLR(LEDG_NODE, gpios));
	if (!device_is_ready(gled_dev)) {
		return -ENODEV;
	}
	gpio_pin_configure(gled_dev, LEDG_PIN, GPIO_OUTPUT | LEDG_FLAGS);
	
	const struct device *bled_dev = DEVICE_DT_GET(DT_GPIO_CTLR(LEDB_NODE, gpios));
	if (!device_is_ready(bled_dev)) {
		return -ENODEV;
	}
	gpio_pin_configure(bled_dev, LEDB_PIN, GPIO_OUTPUT | LEDB_FLAGS);
	
	// Wait for sometime before enabling hold
	while (cnt < CONFIG_POWER_ON_FLASH_TIMES) {
		gpio_pin_set(rled_dev, LEDR_PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		cnt++;
		k_msleep(CONFIG_POWER_ON_LED_FLASH_DELAY);
	}

	const struct device *hold_dev = DEVICE_DT_GET(DT_GPIO_CTLR(PWR_HOLD_GPIO, gpios));
	if (!device_is_ready(hold_dev)) {
		return -ENODEV;
	}
	gpio_pin_configure(hold_dev, PWR_HOLD_GPIO_PIN, GPIO_OUTPUT | PWR_HOLD_GPIO_FLAGS);
	gpio_pin_set(hold_dev, PWR_HOLD_GPIO_PIN, 1);
	
	// Turn on LED
	gpio_pin_set(rled_dev, LEDR_PIN, 0);

	// Setup other GPIOs here
	const struct device *charge_stat_dev = DEVICE_DT_GET(DT_GPIO_CTLR(CHARGE_STAT_GPIO, gpios));
	if (!device_is_ready(charge_stat_dev)) {
		return -ENODEV;
	}
	gpio_pin_configure(charge_stat_dev, CHARGE_STAT_GPIO_PIN, GPIO_INPUT | CHARGE_STAT_GPIO_FLAGS);
	
	const struct device *sw_dev = DEVICE_DT_GET(DT_GPIO_CTLR(SW0_NODE, gpios));
	if (!device_is_ready(sw_dev)) {
		return -ENODEV;
	}
	gpio_pin_configure(sw_dev, SW0_GPIO_PIN, GPIO_INPUT | SW0_GPIO_FLAGS);

	return 0;
}

#if CONFIG_BOARD_VDD_PWR_CTRL_INIT_PRIORITY <= CONFIG_GPIO_NRF_INIT_PRIORITY
#error GPIO_NRF_INIT_PRIORITY must be lower than BOARD_VDD_PWR_CTRL_INIT_PRIORITY
#endif

SYS_INIT(pwr_ctrl_init, POST_KERNEL, CONFIG_BOARD_VDD_PWR_CTRL_INIT_PRIORITY);
