/*
 * LPTIM Validation Test for CH32L103
 * Copyright (c) 2026 Scott King
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <sdi_console.h>

void alarm_callback(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data)
{
	sdi_console_printf("ALARM! Channel %u, ticks %u\n", chan_id, ticks);
}

#include <zephyr/drivers/gpio.h>

int main(void)
{
	const struct device *lptim0 = DEVICE_DT_GET(DT_NODELABEL(lptim));
	const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	uint32_t freq;

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

	sdi_console_init();
	sdi_console_puts("WCH LPTIM Validation Test (SDI Console)\n");

	if (!device_is_ready(lptim0)) {
		sdi_console_puts("FAIL: LPTIM device not ready\n");
		while(1) {
			gpio_pin_toggle_dt(&led);
			k_msleep(100);
		}
	}

	freq = counter_get_frequency(lptim0);
	sdi_console_printf("LPTIM Frequency: %u Hz\n", freq);

	struct counter_alarm_cfg alarm_cfg = {
		.ticks = freq, /* 1 second */
		.callback = alarm_callback,
		.user_data = NULL,
		.flags = 0,
	};

	sdi_console_puts("Starting LPTIM...\n");
	counter_start(lptim0);

	sdi_console_puts("Setting alarm for 1s...\n");
	counter_set_channel_alarm(lptim0, 0, &alarm_cfg);

	while (1) {
		uint32_t ticks;
		counter_get_value(lptim0, &ticks);
		sdi_console_printf("LPTIM CNT: %u\r", ticks);
		
		gpio_pin_toggle_dt(&led);
		k_msleep(100);
	}

	return 0;
}
