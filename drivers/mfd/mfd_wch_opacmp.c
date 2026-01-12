/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_opacmp

#include <zephyr/device.h>
#include <zephyr/drivers/mfd/wch_opacmp.h>
#include <hal_ch32fun.h>

struct mfd_wch_opacmp_config {
	OPA_TypeDef *regs;
};

static int mfd_wch_opacmp_init(const struct device *dev)
{
	const struct mfd_wch_opacmp_config *config = dev->config;

	/* Unlock OPA/CMP registers if needed */
	config->regs->OPCMKEY = 0x45670123;
	config->regs->OPCMKEY = 0xCDEF89AB;

	return 0;
}

#define MFD_WCH_OPACMP_INIT(inst)                                              \
	static const struct mfd_wch_opacmp_config mfd_wch_opacmp_config_##inst = { \
		.regs = (OPA_TypeDef *)DT_INST_REG_ADDR(inst),                         \
	};                                                                         \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst, mfd_wch_opacmp_init, NULL, NULL,               \
			      &mfd_wch_opacmp_config_##inst, PRE_KERNEL_1,             \
			      CONFIG_MFD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MFD_WCH_OPACMP_INIT)
