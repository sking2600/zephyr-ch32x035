/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_cmp

#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/mfd/wch_opacmp.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal_ch32fun.h>

LOG_MODULE_REGISTER(comparator_wch, CONFIG_COMPARATOR_LOG_LEVEL);

struct comparator_wch_config {
	struct wch_opacmp_regs *regs;
	uint8_t index;
	uint8_t psel;
	uint8_t nsel;
	uint8_t mode;
	uint8_t hyen;
};

struct comparator_wch_data {
	comparator_callback_t callback;
	void *user_data;
};

static int comparator_wch_get_output(const struct device *dev)
{
	/* The output status is not directly readable from a register in a 
	 * simple bitfield for CMP1/2/3 on CH32L103 in software without polling mode.
	 */
	return -ENOTSUP;
}

static int comparator_wch_set_trigger(const struct device *dev,
				      enum comparator_trigger trigger)
{
	const struct comparator_wch_config *config = dev->config;
	struct wch_opacmp_regs *regs = config->regs;
	uint32_t ctlr2;

	ctlr2 = regs->CTLR2;

	uint32_t mode = 0;
	switch (trigger) {
	case COMPARATOR_TRIGGER_NONE:
		mode = 0;
		break;
	case COMPARATOR_TRIGGER_BOTH_EDGES:
		mode = 1; /* CMP_WakeUp_Rising_Falling */
		break;
	case COMPARATOR_TRIGGER_RISING_EDGE:
		mode = 2; /* CMP_WakeUp_Rising */
		break;
	case COMPARATOR_TRIGGER_FALLING_EDGE:
		mode = 3; /* CMP_WakeUp_Falling */
		break;
	}

	ctlr2 &= ~CMP_CTLR2_WAKEUP_MASK;
	ctlr2 |= (mode << CMP_CTLR2_WAKEUP_SHIFT);

	regs->CTLR2 = ctlr2;

	return 0;
}

static int comparator_wch_set_trigger_callback(const struct device *dev,
						   comparator_callback_t callback,
						   void *user_data)
{
	struct comparator_wch_data *data = dev->data;

	data->callback = callback;
	data->user_data = user_data;

	return 0;
}

static const struct comparator_driver_api comparator_wch_api = {
	.get_output = comparator_wch_get_output,
	.set_trigger = comparator_wch_set_trigger,
	.set_trigger_callback = comparator_wch_set_trigger_callback,
};

static int comparator_wch_init(const struct device *dev)
{
	const struct comparator_wch_config *config = dev->config;
	struct wch_opacmp_regs *regs = config->regs;
	uint32_t ctlr2;
    uint8_t cmp_idx = config->index - 1;

	ctlr2 = regs->CTLR2;

	/* Configure CMP bits in CTLR2
	 * Bits per CMP:
	 * Enable [0] + idx*8
	 * Mode   [1:2] + idx*8
	 * NSEL   [3] + idx*8
	 * PSEL   [4] + idx*8
	 * HYEN   [5] + idx*8
	 * LP     [6] + idx*8
	 */
	uint32_t mask = (CMP_CTLR2_ALL_MASK << (cmp_idx * 8));
	uint32_t val = CMP_CTLR2_EN_MASK | 
	               (config->mode << 1) | 
	               (config->nsel << 3) | 
	               (config->psel << 4) | 
	               (config->hyen << 5);

	ctlr2 &= ~mask;
	ctlr2 |= (val << (cmp_idx * 8));

	regs->CTLR2 = ctlr2;

	return 0;
}

#define COMPARATOR_WCH_INIT(n)                                                 \
	static struct comparator_wch_data comparator_wch_data_##n;                 \
                                                                               \
	static const struct comparator_wch_config comparator_wch_config_##n = {    \
		.regs = (struct wch_opacmp_regs *)DT_REG_ADDR(DT_PARENT(DT_DRV_INST(n))), \
		.index = DT_INST_PROP(n, index),                                       \
		.psel = DT_INST_PROP_OR(n, wch_psel, 0),                               \
		.nsel = DT_INST_PROP_OR(n, wch_nsel, 0),                               \
		.mode = DT_INST_PROP_OR(n, wch_mode, 0),                               \
		.hyen = DT_INST_PROP_OR(n, wch_hyen, 0),                               \
	};                                                                         \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, comparator_wch_init, NULL,                        \
			      &comparator_wch_data_##n,                                \
			      &comparator_wch_config_##n, POST_KERNEL,                 \
			      CONFIG_COMPARATOR_INIT_PRIORITY, &comparator_wch_api);

DT_INST_FOREACH_STATUS_OKAY(COMPARATOR_WCH_INIT)
