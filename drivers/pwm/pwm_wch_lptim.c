/*
 * Copyright (c) 2024 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_lptim_pwm

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <hal_ch32fun.h>

struct pwm_lptim_config {
	LPTIM_TypeDef *regs;
	const struct device *clock_dev;
	uint16_t clock_id;
};

struct pwm_lptim_data {
	uint32_t period_cycles;
};

static int pwm_lptim_set_cycles(const struct device *dev, uint32_t channel,
				 uint32_t period_cycles, uint32_t pulse_cycles,
				 pwm_flags_t flags)
{
	const struct pwm_lptim_config *config = dev->config;
	struct pwm_lptim_data *data = dev->data;

	if (channel != 0) {
		return -EINVAL;
	}

	if (period_cycles == 0 || period_cycles > 0xFFFF) {
		return -EINVAL;
	}

	if (pulse_cycles > period_cycles) {
		return -EINVAL;
	}

	/* Disable LPTIM */
	config->regs->CR = 0;

	/* Configure for PWM mode
	 * WAVE (bit 20): PWM mode
	 * PRELOAD (bit 22): Enable preload for glitch-free updates
	 * Internal clock source (bits 25-26): 00 = PCLK1
	 * Prescaler: /128 (bits 9-11): 111 = /128
	 * Continuous mode will be set in CR via CNTSTRT
	 * TIMOUT disabled as per example
	 */
	config->regs->CFGR = LPTIM_CFGR_WAVE | LPTIM_CFGR_PRELOAD | (7 << 9); /* PSC = /128 */

	/* Apply polarity */
	if (flags & PWM_POLARITY_INVERTED) {
		config->regs->CFGR |= LPTIM_CFGR_WAVPOL;
	}

	/* Enable LPTIM */
	config->regs->CR |= LPTIM_CR_ENABLE;

	/* Set period (ARR) */
	config->regs->ARR = (uint16_t)(period_cycles - 1);

	/* Wait for ARROK */
	uint32_t timeout = 100000;
	while (!(config->regs->ISR & LPTIM_ISR_ARROK) && timeout--);
	config->regs->ICR |= LPTIM_ICR_ARROKCF;

	/* Set pulse width (CMP) */
	config->regs->CMP = (uint16_t)pulse_cycles;

	/* Wait for CMPOK */
	timeout = 100000;
	while (!(config->regs->ISR & LPTIM_ISR_CMPOK) && timeout--);
	config->regs->ICR |= LPTIM_ICR_CMPOKCF;

	/* Enable PWM output and start continuous mode */
	config->regs->CR |= LPTIM_CR_OUTEN | LPTIM_CR_CNTSTRT;

	data->period_cycles = period_cycles;

	return 0;
}

static int pwm_lptim_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					 uint64_t *cycles)
{
	const struct pwm_lptim_config *config = dev->config;
	uint32_t rate;
	int err;

	ARG_UNUSED(channel);

	/* Get PCLK1 frequency from the clock controller */
	err = clock_control_get_rate(config->clock_dev,
				      (clock_control_subsys_t)(uintptr_t)config->clock_id,
				      &rate);
	if (err < 0) {
		return err;
	}

	/* Account for /128 prescaler */
	*cycles = rate / 128;

	return 0;
}

static int pwm_lptim_init(const struct device *dev)
{
	const struct pwm_lptim_config *config = dev->config;
	int err;

	/* Enable LPTIM clock */
	err = clock_control_on(config->clock_dev,
			       (clock_control_subsys_t)(uintptr_t)config->clock_id);
	if (err < 0) {
		return err;
	}

	/* Configure PB15 as alternate function for LPTIM output */
	/* Enable GPIOB and AFIO clock */
	RCC->APB2PCENR |= RCC_PB2Periph_GPIOB | RCC_PB2Periph_AFIO;

	/* Ensure LPTIM is not remapped (Default: PB15) */
	AFIO->PCFR2 &= ~AFIO_PCFR2_LPTIM_RM;
	
	/* Configure PB15 as AF push-pull */
	GPIOB->CFGHR &= ~(0xF << ((15 - 8) * 4));
	GPIOB->CFGHR |= (0xB << ((15 - 8) * 4)); /* AF push-pull, 50MHz */

	return 0;
}

static const struct pwm_driver_api pwm_lptim_driver_api = {
	.set_cycles = pwm_lptim_set_cycles,
	.get_cycles_per_sec = pwm_lptim_get_cycles_per_sec,
};

#define PWM_LPTIM_INIT(n)							\
										\
	static const struct pwm_lptim_config pwm_lptim_config_##n = {		\
		.regs = (LPTIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)),	\
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(n))),	\
		.clock_id = DT_CLOCKS_CELL(DT_INST_PARENT(n), id),		\
	};									\
										\
	static struct pwm_lptim_data pwm_lptim_data_##n;			\
										\
	DEVICE_DT_INST_DEFINE(n, pwm_lptim_init, NULL,				\
			      &pwm_lptim_data_##n, &pwm_lptim_config_##n,	\
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,		\
			      &pwm_lptim_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_LPTIM_INIT)
