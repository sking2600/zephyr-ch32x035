/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_opa

#include <zephyr/drivers/opamp.h>
#include <zephyr/drivers/mfd/wch_opacmp.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal_ch32fun.h>

LOG_MODULE_REGISTER(opamp_wch, CONFIG_OPAMP_LOG_LEVEL);

struct opamp_wch_config {
	OPA_TypeDef *regs;
	uint8_t index;
	uint8_t psel;
	uint8_t nsel;
	uint8_t mode;
};

static int opamp_wch_set_gain(const struct device *dev, enum opamp_gain gain)
{
	const struct opamp_wch_config *config = dev->config;
	OPA_TypeDef *regs = config->regs;
	uint32_t nsel_val;

	switch (gain) {
	case OPAMP_GAIN_2:  nsel_val = 0; break;
	case OPAMP_GAIN_3:  nsel_val = 1; break;
	case OPAMP_GAIN_4:  nsel_val = 2; break;
	case OPAMP_GAIN_7:  nsel_val = 3; break;
	case OPAMP_GAIN_8:  nsel_val = 4; break;
	case OPAMP_GAIN_15: nsel_val = 5; break;
	case OPAMP_GAIN_16: nsel_val = 6; break;
	case OPAMP_GAIN_31: nsel_val = 7; break;
	case OPAMP_GAIN_32: nsel_val = 8; break;
	case OPAMP_GAIN_63: nsel_val = 9; break;
	case OPAMP_GAIN_64: nsel_val = 10; break;
	default:
		return -ENOTSUP;
	}

	/* CH32L103 OPA bits in CTLR1:
	 * OPA1 bits 8:11 for NSEL (Gain in PGA mode)
	 */
	uint32_t ctlr1 = regs->CTLR1;
	ctlr1 &= ~OPA_CTLR1_NSEL1;
	ctlr1 |= (nsel_val << 8);
	regs->CTLR1 = ctlr1;

	return 0;
}

static const struct opamp_driver_api opamp_wch_api = {
	.set_gain = opamp_wch_set_gain,
};

static int opamp_wch_init(const struct device *dev)
{
	const struct opamp_wch_config *config = dev->config;
	OPA_TypeDef *regs = config->regs;
	uint32_t ctlr1;

	ctlr1 = regs->CTLR1;

	/* Configure OPA bits in CTLR1 
	 * OPA1 bits: 
	 * Mode [1:3]
	 * PSEL [4:6]
	 * FB   [7]
	 * NSEL [8:11]
	 */
	ctlr1 &= ~(0x7FF << 1); // Mask bits 1 to 11
	ctlr1 |= (config->mode << 1) | (config->psel << 4) | (0 << 7) | (config->nsel << 8);

	/* Enable OPA. For OPA1, it's bit 0. */
	ctlr1 |= (1 << 0);

	regs->CTLR1 = ctlr1;

	return 0;
}

#define OPAMP_WCH_INIT(n)                                                      \
	static const struct opamp_wch_config opamp_wch_config_##n = {              \
		.regs = (OPA_TypeDef *)DT_REG_ADDR(DT_PARENT(DT_DRV_INST(n))),         \
		.index = DT_INST_PROP(n, index),                                       \
		.psel = DT_INST_PROP_OR(n, wch_psel, 0),                               \
		.nsel = DT_INST_PROP_OR(n, wch_nsel, 0),                               \
		.mode = DT_INST_PROP_OR(n, wch_mode, 0),                               \
	};                                                                         \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, opamp_wch_init, NULL, NULL,                       \
			      &opamp_wch_config_##n, POST_KERNEL,                      \
			      CONFIG_OPAMP_INIT_PRIORITY, &opamp_wch_api);

DT_INST_FOREACH_STATUS_OKAY(OPAMP_WCH_INIT)
