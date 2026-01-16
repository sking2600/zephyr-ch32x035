/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_opacmp

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/mfd/wch_opacmp.h>

LOG_MODULE_REGISTER(mfd_wch_opacmp, CONFIG_MFD_LOG_LEVEL);

struct mfd_wch_opacmp_config {
	struct wch_opacmp_regs *regs;
	const struct device *clock_dev;
	const struct clock_control_subsys clock_subsys;
};

static int mfd_wch_opacmp_init(const struct device *dev)
{
	const struct mfd_wch_opacmp_config *config = dev->config;
	int ret;

#if DT_INST_NODE_HAS_PROP(0, clocks)
	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Could not enable OPACMP clock (%d)", ret);
		return ret;
	}
#endif

	/* Unlock OPA/CMP registers */
	/* Note: Access size must be 32-bit for OPCMKEY? Reference manual suggests 32-bit writes for KEY. */
	config->regs->OPCMKEY = WCH_OPCM_KEY1;
	config->regs->OPCMKEY = WCH_OPCM_KEY2;
	
	LOG_DBG("WCH OPACMP MFD initialized, regs at %p", config->regs);

	return 0;
}

#define MFD_WCH_OPACMP_INIT(inst)                                              \
	static const struct mfd_wch_opacmp_config mfd_wch_opacmp_config_##inst = { \
		.regs = (struct wch_opacmp_regs *)DT_INST_REG_ADDR(inst),              \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, clocks),                       \
			(                                                                  \
			 .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),            \
			 .clock_subsys = (void *)DT_INST_CLOCKS_CELL(inst, bit),           \
			),                                                                 \
			()                                                                 \
		)                                                                      \
	};                                                                         \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst, mfd_wch_opacmp_init, NULL, NULL,               \
			      &mfd_wch_opacmp_config_##inst, POST_KERNEL,              \
			      CONFIG_MFD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MFD_WCH_OPACMP_INIT)
