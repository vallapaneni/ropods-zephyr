#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "ppod_v2p0_defs.h"

static struct gpio_callback charge_stat_cb_data;
static struct gpio_callback gpio_cb;
static struct k_work power_off_work;

// Battery charge full indication
static void charge_stat_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	const struct device *gled_dev = DEVICE_DT_GET(DT_GPIO_CTLR(LEDG_NODE, gpios));
	if (device_is_ready(gled_dev)) {
		gpio_pin_set(gled_dev, LEDG_PIN, 0);
	}
}

void setup_charge_status(void)
{
	const struct device *charge_stat_dev = DEVICE_DT_GET(DT_GPIO_CTLR(CHARGE_STAT_GPIO, gpios));
	if (!device_is_ready(charge_stat_dev)) {
		printk("Charge status device not ready!\n");
		return;
	}
	
	gpio_init_callback(&charge_stat_cb_data, charge_stat_cb, BIT(CHARGE_STAT_GPIO_PIN));
	int ret = gpio_add_callback(charge_stat_dev, &charge_stat_cb_data);
	if (ret) {
		printk("Cannot setup callback!\n");
	}
	ret = gpio_pin_interrupt_configure(charge_stat_dev, CHARGE_STAT_GPIO_PIN, GPIO_INT_EDGE_RISING);
	if (ret) {
		printk("Error enabling callback!\n");
	}
}

static void power_off_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pin_pos)
{
	k_work_submit(&power_off_work);
}

static void power_off(struct k_work *work)
{
	// FIXME: need to do power down gracefully
	const struct device *sw_dev = DEVICE_DT_GET(DT_GPIO_CTLR(SW0_NODE, gpios));
	if (!device_is_ready(sw_dev)) {
		return;
	}
	
	uint8_t sw_status;
	int cnt = 0;
	bool led_is_on = true;
	const struct device *rled_dev = DEVICE_DT_GET(DT_GPIO_CTLR(LEDR_NODE, gpios));
	
	if (!device_is_ready(rled_dev)) {
		return;
	}

	do {
		gpio_pin_set(rled_dev, LEDR_PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		cnt++;
		if (cnt == CONFIG_POWER_OFF_FLASH_TIMES) {
			const struct device *hold_dev = DEVICE_DT_GET(DT_GPIO_CTLR(PWR_HOLD_GPIO, gpios));
			if (device_is_ready(hold_dev)) {
				gpio_pin_set(hold_dev, PWR_HOLD_GPIO_PIN, 0);
				gpio_pin_set(rled_dev, LEDR_PIN, 1);
			}
			break;
		}
		k_sleep(K_MSEC(CONFIG_POWER_OFF_LED_FLASH_DELAY));
		sw_status = gpio_pin_get(sw_dev, SW0_GPIO_PIN);
	} while (sw_status == 0);
}

void setup_power_off(void)
{
	const struct device *sw_dev = DEVICE_DT_GET(DT_GPIO_CTLR(SW0_NODE, gpios));
	if (!device_is_ready(sw_dev)) {
		printk("Switch device not ready!\n");
		return;
	}
	
	gpio_init_callback(&gpio_cb, power_off_cb, BIT(SW0_GPIO_PIN));
	int ret = gpio_add_callback(sw_dev, &gpio_cb);
	if (ret) {
		printk("Cannot setup callback for switch!\n");
	}
	ret = gpio_pin_interrupt_configure(sw_dev, SW0_GPIO_PIN, GPIO_INT_EDGE_FALLING);
	if (ret) {
		printk("Error enabling callback for switch!\n");
	}
	k_work_init(&power_off_work, power_off);
}
