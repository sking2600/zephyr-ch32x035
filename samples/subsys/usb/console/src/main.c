/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

// BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
// 	     "Console device is not ACM CDC UART device");

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	/* Initialize LED */
	if (device_is_ready(led.port)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	}

	/* Immediate blink to show we are alive */
	for (int i = 0; i < 10; i++) {
		gpio_pin_toggle_dt(&led);
		k_busy_wait(100000);
	}

	printk("--- CH32L103 USB Debug Start ---\n");

	// const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	// uint32_t dtr = 0;

	/* Debug RCC Configuration */
	// volatile uint32_t *rcc_cfgr0 = (uint32_t *)0x40021004;

	while (1) {
		gpio_pin_toggle_dt(&led);
		printk("Hello World! %s\n", CONFIG_ARCH);
		k_sleep(K_SECONDS(1));
	}
}
