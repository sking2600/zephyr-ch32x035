/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

/* 20ms period = 50Hz frequency */
#define PWM_PERIOD_NS   20000000

int main(void)
{
	int ret;
	uint32_t pulse_width;
	uint8_t duty_cycle = 0;

	printk("PWM Blinky: Ramp 10%% every 0.5s\n");

	if (!pwm_is_ready_dt(&pwm_led0)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led0.dev->name);
		return 0;
	}

	while (1) {
		pulse_width = (uint32_t)((uint64_t)PWM_PERIOD_NS * duty_cycle / 100);

		ret = pwm_set_dt(&pwm_led0, PWM_PERIOD_NS, pulse_width);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}

		printk("Duty Cycle: %d%%\n", duty_cycle);

		duty_cycle += 10;
		if (duty_cycle > 100) {
			duty_cycle = 0;
		}

		k_sleep(K_MSEC(500));
	}
	return 0;
}
