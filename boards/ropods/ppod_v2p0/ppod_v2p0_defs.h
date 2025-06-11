#ifndef _PPOD_V2P0_DEFS_H_
#define _PPOD_V2P0_DEFS_H_

#include <zephyr/devicetree.h>

#define LEDR_NODE DT_ALIAS(ledr)
#define LEDR_PIN	DT_GPIO_PIN(LEDR_NODE, gpios)
#define LEDR_FLAGS	DT_GPIO_FLAGS(LEDR_NODE, gpios)

#define LEDG_NODE DT_ALIAS(ledg)
#define LEDG_PIN	DT_GPIO_PIN(LEDG_NODE, gpios)
#define LEDG_FLAGS	DT_GPIO_FLAGS(LEDG_NODE, gpios)

#define LEDB_NODE DT_ALIAS(ledb)
#define LEDB_PIN	DT_GPIO_PIN(LEDB_NODE, gpios)
#define LEDB_FLAGS	DT_GPIO_FLAGS(LEDB_NODE, gpios)

#define PWR_HOLD_GPIO                 DT_NODELABEL(pwr_hold)
#define PWR_HOLD_GPIO_FLAGS        DT_GPIO_FLAGS(PWR_HOLD_GPIO, gpios)
#define PWR_HOLD_GPIO_PIN          DT_GPIO_PIN(PWR_HOLD_GPIO, gpios)

#define CHARGE_STAT_GPIO             DT_NODELABEL(charge_stat)
#define CHARGE_STAT_GPIO_PIN         DT_GPIO_PIN(CHARGE_STAT_GPIO, gpios)
#define CHARGE_STAT_GPIO_FLAGS       DT_GPIO_FLAGS(CHARGE_STAT_GPIO, gpios)

#define SW0_NODE            DT_NODELABEL(sw0)
#define SW0_GPIO_FLAGS      DT_GPIO_FLAGS(SW0_NODE, gpios)
#define SW0_GPIO_PIN        DT_GPIO_PIN(SW0_NODE, gpios)

#define BQ274xx_LABEL           DT_LABEL(DT_INST(0, ti_bq274xx))

#define DRV2605_INPUT_NODE                DT_NODELABEL(drv2605_input)
#define DRV2605_INPUT_GPIO_FLAGS    DT_GPIO_FLAGS(DRV2605_INPUT_NODE, gpios)
#define DRV2605_INPUT_GPIO_PIN      DT_GPIO_PIN(DRV2605_INPUT_NODE, gpios)

#define DRV2605_ENABLE_NODE                DT_NODELABEL(drv2605_enable)
#define DRV2605_ENABLE_GPIO_FLAGS   DT_GPIO_FLAGS(DRV2605_ENABLE_NODE, gpios)
#define DRV2605_ENABLE_GPIO_PIN     DT_GPIO_PIN(DRV2605_ENABLE_NODE, gpios)

/* PPOD_HW_CYCLES_TO_MS64 converts CPU clock cycles to milliseconds */
#define PPOD_HW_CYCLES_TO_MS64(X) \
	(((uint64_t)(X) * MSEC_PER_SEC) / sys_clock_hw_cycles_per_sec())

#define PPOD_HW_CYCLES_TO_MS(X) (uint32_t)(PPOD_HW_CYCLES_TO_MS64(X))

#ifdef __cplusplus
extern "C" {
#endif

void setup_power_off(void);
void setup_charge_status(void);

#ifdef __cplusplus
}
#endif

#endif
