/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_lptim

#include <zephyr/drivers/counter.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <soc.h>

LOG_MODULE_REGISTER(counter_lptim, CONFIG_COUNTER_LOG_LEVEL);

struct counter_lptim_config {
	struct counter_config_info info;
	LPTIM_TypeDef *regs;
	const struct device *clock_dev;
	uint32_t clock_id;
	void (*irq_config)(const struct device *dev);
};

struct counter_lptim_data {
	counter_alarm_callback_t alarm_cb;
	uint32_t top_value;
	void *user_data;
};

static int counter_lptim_start(const struct device *dev)
{
	const struct counter_lptim_config *config = dev->config;

	config->regs->CR |= LPTIM_CR_ENABLE;
	return 0;
}

static int counter_lptim_stop(const struct device *dev)
{
	const struct counter_lptim_config *config = dev->config;

	config->regs->CR &= ~LPTIM_CR_ENABLE;
	return 0;
}

static int counter_lptim_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_lptim_config *config = dev->config;

	*ticks = config->regs->CNT;
	return 0;
}

static int counter_lptim_set_alarm(const struct device *dev, uint8_t chan_id,
				    const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;

	if (chan_id != 0) {
		return -ENOTSUP;
	}

	data->alarm_cb = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;

	/* Set compare value */
	config->regs->CMP = alarm_cfg->ticks;
	
	/* Enable compare match interrupt */
	config->regs->IER |= LPTIM_IER_CMPMIE;

	return 0;
}

static int counter_lptim_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;

	if (chan_id != 0) {
		return -ENOTSUP;
	}

	config->regs->IER &= ~LPTIM_IER_CMPMIE;
	data->alarm_cb = NULL;

	return 0;
}

static int counter_lptim_set_top_value(const struct device *dev,
					const struct counter_top_cfg *cfg)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;

	if (cfg->ticks > 0xFFFF) {
		return -EINVAL;
	}

	data->top_value = cfg->ticks;
	config->regs->ARR = cfg->ticks;

	return 0;
}

static uint32_t counter_lptim_get_top_value(const struct device *dev)
{
	struct counter_lptim_data *data = dev->data;

	return data->top_value;
}

static void counter_lptim_isr(const struct device *dev)
{
	const struct counter_lptim_config *config = dev->config;
	struct counter_lptim_data *data = dev->data;
	uint32_t isr = config->regs->ISR;

	if (isr & LPTIM_ISR_CMPM) {
		/* Clear flag */
		config->regs->ICR |= LPTIM_ICR_CMPMCF;

		if (data->alarm_cb) {
			data->alarm_cb(dev, 0, config->regs->CNT, data->user_data);
		}
	}

	if (isr & LPTIM_ISR_ARRM) {
		/* Clear flag */
		config->regs->ICR |= LPTIM_ICR_ARRMCF;
	}
}

static const struct counter_driver_api counter_lptim_api = {
	.start = counter_lptim_start,
	.stop = counter_lptim_stop,
	.get_value = counter_lptim_get_value,
	.set_alarm = counter_lptim_set_alarm,
	.cancel_alarm = counter_lptim_cancel_alarm,
	.set_top_value = counter_lptim_set_top_value,
	.get_top_value = counter_lptim_get_top_value,
};

static int counter_lptim_init(const struct device *dev)
{
	struct counter_lptim_config *config = (struct counter_lptim_config *)dev->config;
	struct counter_lptim_data *data = dev->data;
	uint32_t rate;
	int err;

	err = clock_control_on(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);
	if (err < 0) {
		return err;
	}

	err = clock_control_get_rate(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id, &rate);
	if (err < 0) {
		return err;
	}

	config->info.freq = rate;

	/* Default top value */
	data->top_value = 0xFFFF;
	config->regs->ARR = 0xFFFF;

	config->irq_config(dev);

	return 0;
}

#define LPTIM_DEVICE_INIT(inst)                                                                    \
	static void counter_lptim_irq_config_##inst(const struct device *dev)                      \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                       \
			    counter_lptim_isr, DEVICE_DT_INST_GET(inst), 0);                      \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
                                                                                                   \
	static struct counter_lptim_data counter_lptim_data_##inst;                                \
                                                                                                   \
	static struct counter_lptim_config counter_lptim_config_##inst = {                         \
		.info = {                                                                          \
			.max_top_value = 0xFFFF,                                                   \
			.freq = 0, /* Set at runtime */                                            \
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,                                     \
			.channels = 1,                                                             \
		},                                                                                 \
		.regs = (LPTIM_TypeDef *)DT_INST_REG_ADDR(inst),                                   \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                            \
		.clock_id = DT_INST_CLOCKS_CELL(inst, id),                                        \
		.irq_config = counter_lptim_irq_config_##inst,                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, counter_lptim_init, NULL, &counter_lptim_data_##inst,          \
			      &counter_lptim_config_##inst, POST_KERNEL,                           \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_lptim_api);
 

DT_INST_FOREACH_STATUS_OKAY(LPTIM_DEVICE_INIT)
