/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_pwr

#include <zephyr/drivers/pm/pwr_wch.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <errno.h>

LOG_MODULE_REGISTER(pwr_wch, CONFIG_PM_LOG_LEVEL);

struct pwr_wch_config {
	PWR_TypeDef *regs;
#if DT_INST_NODE_HAS_PROP(0, clocks)
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
#endif
};

static int pwr_wch_init(const struct device *dev)
{
	const struct pwr_wch_config *cfg = dev->config;

#if DT_INST_NODE_HAS_PROP(0, clocks)
	if (cfg->clock_dev != NULL) {
		int err = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
		if (err != 0) {
			LOG_ERR("Failed to enable clock (%d)", err);
			return err;
		}
	}
#endif

	/* Optional PVD configuration */
#if DT_INST_NODE_HAS_PROP(0, pvd_level)
	uint32_t level = DT_INST_PROP(0, pvd_level);
	cfg->regs->CTLR = (cfg->regs->CTLR & ~PWR_CTLR_PLS) | (level << 5);
#endif

	return 0;
}

int wch_pwr_set_backup_access(const struct device *dev, bool enable)
{
	const struct pwr_wch_config *cfg = dev->config;

	if (enable) {
		cfg->regs->CTLR |= PWR_CTLR_DBP;
	} else {
		cfg->regs->CTLR &= ~PWR_CTLR_DBP;
	}

	return 0;
}

int wch_pwr_enter_low_power(const struct device *dev, uint32_t mode)
{
	const struct pwr_wch_config *cfg = dev->config;

	switch (mode) {
	case 0: /* Sleep */
		/* WFI/WFE handled by kernel usually */
		break;
	case 1: /* Stop */
		cfg->regs->CTLR |= PWR_CTLR_PDDS;
		cfg->regs->CTLR &= ~PWR_CTLR_LPDS;
		break;
	case 2: /* Standby */
		cfg->regs->CTLR |= PWR_CTLR_PDDS;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define PWR_WCH_INIT(inst)                                                                         \
	static const struct pwr_wch_config pwr_wch_cfg_##inst = {                                  \
		.regs = (PWR_TypeDef *)DT_INST_REG_ADDR(inst),                                     \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, clocks), (                                 \
			.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                     \
			.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, id),   \
		))                                                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, pwr_wch_init, NULL, NULL, &pwr_wch_cfg_##inst, PRE_KERNEL_1,   \
			      CONFIG_PM_WCH_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PWR_WCH_INIT);
