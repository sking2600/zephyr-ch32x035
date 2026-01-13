/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_pwr

#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <soc.h>

LOG_MODULE_REGISTER(pwr_wch, CONFIG_PM_LOG_LEVEL);

struct pwr_wch_config {
	PWR_TypeDef *regs;
};

static int pwr_wch_init(const struct device *dev)
{
	/* Clocks are handled by Device Tree and clock_control if needed, 
	 * but PWR is usually always on or enabled during early boot.
	 */

	return 0;
}

/* Standalone helper to enable backup domain access */
void z_wch_pwr_enable_bkp_access(void)
{
	PWR->CTLR |= PWR_CTLR_DBP;
}

void z_wch_pwr_disable_bkp_access(void)
{
	PWR->CTLR &= ~PWR_CTLR_DBP;
}

#define PWR_WCH_INIT(inst)                                                                         \
	static const struct pwr_wch_config pwr_wch_cfg_##inst = {                                  \
		.regs = (PWR_TypeDef *)DT_INST_REG_ADDR(inst),                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, pwr_wch_init, NULL, NULL, &pwr_wch_cfg_##inst, PRE_KERNEL_1,   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(PWR_WCH_INIT);
