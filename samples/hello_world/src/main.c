/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

extern void sdi_console_init(void);
extern void sdi_console_printf(const char *format, ...);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;

    sdi_console_init();
    sdi_console_printf("Driver Port Verification @ %u Hz (SDI)\n", sys_clock_hw_cycles_per_sec());

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
        sdi_console_printf("Ping: %lld ms\n", k_uptime_get());
		k_msleep(1000);
	}
	return 0;
}
