/*
 * Copyright (c) 2025 Michael Hope <michaelh@juju.nz>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_gptm_pwm

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/pwm/pwm.h>

#include <hal_ch32fun.h>

#ifdef TIM2_CTLR1_CEN
/* ch32fun.h uses a different set of names for the CH32V00x series. Remap. */
typedef GPTM_TypeDef TIM_TypeDef;
#define TIM_CEN  TIM2_CTLR1_CEN
#define TIM_OC1M TIM2_CHCTLR1_OC1M
#define TIM_OC2M TIM2_CHCTLR1_OC2M
#define TIM_OC3M TIM2_CHCTLR2_OC3M
#define TIM_OC4M TIM2_CHCTLR2_OC4M
#define TIM_CC1P TIM2_CCER_CC1P
#define TIM_CC1E TIM2_CCER_CC1E
#define TIM_ARPE TIM2_CTLR1_ARPE
#endif

/* Standard TIM register bits if not defined */
#ifndef TIM_OCPE
#define TIM_OCPE 0x08
#endif

#ifndef TIM_UG
#define TIM_UG 0x01
#endif

#ifndef TIM_CEN
#define TIM_CEN 0x01
#endif

#ifndef TIM_MOE
#define TIM_MOE 0x8000
#endif

struct pwm_wch_gptm_config {
	TIM_TypeDef *regs;
	const struct device *clock_dev;
	uint8_t clock_id;
	uint16_t prescaler;
	const struct pinctrl_dev_config *pin_cfg;
	uint8_t rptcr;
};

#define CHCTLR_OCXM_ODD_SHIFT  4
#define CHCTLR_OCXM_EVEN_SHIFT 12
#define CHCTLR_OCXM_MASK       0x7

static int pwm_wch_gptm_set_cycles(const struct device *dev, uint32_t channel,
				  uint32_t period_cycles, uint32_t pulse_cycles,
				  pwm_flags_t flags)
{
	const struct pwm_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;
	uint32_t ocxm;

	if (channel < 1 || channel > 4) {
		return -EINVAL;
	}

	if (period_cycles == 0) {
		return -EINVAL;
	}

	if (flags & PWM_POLARITY_INVERTED) {
		ocxm = 0x5; /* PWM mode 2 */
	} else {
		ocxm = 0x4; /* PWM mode 1 */
	}

	switch (channel) {
	case 1:
		regs->CHCTLR1 = (regs->CHCTLR1 & ~(CHCTLR_OCXM_MASK << CHCTLR_OCXM_ODD_SHIFT)) |
				(ocxm << CHCTLR_OCXM_ODD_SHIFT) | TIM_OCPE;
		regs->CH1CVR = pulse_cycles;
		break;
	case 2:
		regs->CHCTLR1 = (regs->CHCTLR1 & ~(CHCTLR_OCXM_MASK << CHCTLR_OCXM_EVEN_SHIFT)) |
				(ocxm << CHCTLR_OCXM_EVEN_SHIFT) | (TIM_OCPE << 8);
		regs->CH2CVR = pulse_cycles;
		break;
	case 3:
		regs->CHCTLR2 = (regs->CHCTLR2 & ~(CHCTLR_OCXM_MASK << CHCTLR_OCXM_ODD_SHIFT)) |
				(ocxm << CHCTLR_OCXM_ODD_SHIFT) | TIM_OCPE;
		regs->CH3CVR = pulse_cycles;
		break;
	case 4:
		regs->CHCTLR2 = (regs->CHCTLR2 & ~(CHCTLR_OCXM_MASK << CHCTLR_OCXM_EVEN_SHIFT)) |
				(ocxm << CHCTLR_OCXM_EVEN_SHIFT) | (TIM_OCPE << 8);
		regs->CH4CVR = pulse_cycles;
		break;
	}

	regs->ATRLR = period_cycles;
	regs->CTLR1 |= TIM_ARPE;
	regs->SWEVGR |= TIM_UG;

	return 0;
}

static int pwm_wch_gptm_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					  uint64_t *cycles)
{
	const struct pwm_wch_gptm_config *config = dev->config;
	uint32_t clock_rate;
	int err;

	err = clock_control_get_rate(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id, &clock_rate);
	if (err != 0) {
		return err;
	}

	*cycles = (uint64_t)clock_rate / (config->prescaler + 1);

	return 0;
}

#ifdef CONFIG_PWM_WITH_DMA
static int pwm_wch_gptm_enable_dma(const struct device *dev, uint32_t channel)
{
	const struct pwm_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;

	if (channel < 1 || channel > 4) {
		return -EINVAL;
	}

	/* Map channel to DMA request enable bit (TIM_DMA_CCx) */
	/* TIM_DMA_CC1 is typically bit 9, etc. */
	regs->DMAINTENR |= (1 << (8 + channel));

	return 0;
}

static int pwm_wch_gptm_disable_dma(const struct device *dev, uint32_t channel)
{
	const struct pwm_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;

	if (channel < 1 || channel > 4) {
		return -EINVAL;
	}

	regs->DMAINTENR &= ~(1 << (8 + channel));

	return 0;
}
#endif

static const struct pwm_driver_api pwm_wch_gptm_driver_api = {
	.set_cycles = pwm_wch_gptm_set_cycles,
	.get_cycles_per_sec = pwm_wch_gptm_get_cycles_per_sec,
#ifdef CONFIG_PWM_WITH_DMA
	.enable_dma = pwm_wch_gptm_enable_dma,
	.disable_dma = pwm_wch_gptm_disable_dma,
#endif
};

static int pwm_wch_gptm_init(const struct device *dev)
{
	const struct pwm_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;
	int err;

	clock_control_on(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	regs->PSC = config->prescaler;
	regs->CTLR1 = TIM_ARPE;

#ifdef TIM1_BASE
	if ((uintptr_t)regs == TIM1_BASE) {
		/* Advanced timer features */
		regs->RPTCR = config->rptcr;
		regs->BDTR = TIM_MOE;
	}
#endif

	regs->CCER = TIM_CC1E | TIM_CC2E | TIM_CC3E | TIM_CC4E;
	regs->CTLR1 |= TIM_CEN;

	return 0;
}

#define PWM_WCH_GPTM_INIT(idx)									   \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
												   \
	static const struct pwm_wch_gptm_config pwm_wch_gptm_config_##idx = {			   \
		.regs = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(idx)),			   \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(idx))),		   \
		.clock_id = DT_CLOCKS_CELL(DT_INST_PARENT(idx), id),				   \
		.prescaler = DT_PROP(DT_INST_PARENT(idx), prescaler),				   \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),					   \
		.rptcr = DT_INST_PROP_OR(idx, wch_repetition_counter, 0),			   \
	};											   \
												   \
	DEVICE_DT_INST_DEFINE(idx, pwm_wch_gptm_init, NULL, NULL,					   \
			      &pwm_wch_gptm_config_##idx, POST_KERNEL,				   \
			      CONFIG_PWM_INIT_PRIORITY, &pwm_wch_gptm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_WCH_GPTM_INIT)
