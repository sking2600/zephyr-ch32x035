/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_crc

#include <zephyr/drivers/crc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/pm/device.h>
#include <hal_ch32fun.h>

struct crc_wch_config {
	CRC_TypeDef *regs;
	const struct device *clock_dev;
	uint32_t clock_subsys;
};

static int crc_wch_begin(const struct device *dev, struct crc_ctx *ctx)
{
	const struct crc_wch_config *cfg = dev->config;

	if (ctx->type != CRC_32) {
		return -ENOTSUP;
	}

	/* WCH CRC unit is fixed to CRC-32 (0x04C11DB7) with 0xFFFFFFFF init val */
	if (ctx->polynomial != 0x04C11DB7 || ctx->seed != 0xFFFFFFFF) {
		return -ENOTSUP;
	}

	cfg->regs->CTLR |= CRC_CTLR_RESET;
	ctx->state = CRC_STATE_IN_PROGRESS;
	ctx->result = 0xFFFFFFFF;

	return 0;
}

static int crc_wch_update(const struct device *dev, struct crc_ctx *ctx,
			  const void *buffer, size_t bufsize)
{
	const struct crc_wch_config *cfg = dev->config;
	const uint32_t *p32 = buffer;
	size_t words = bufsize / 4;
	size_t rem = bufsize % 4;

	if (ctx->state != CRC_STATE_IN_PROGRESS) {
		return -EALREADY;
	}

	for (size_t i = 0; i < words; i++) {
		cfg->regs->DATAR = *p32++;
	}

	if (rem > 0) {
		/* WCH CRC unit only supports 32-bit writes for calculation.
		 * If we have non-aligned data, we might need to handle it in software
		 * or pad it, but standard 32-bit CRC hardware usually works on words.
		 * For now, return error or implement software fallback for remainder.
		 */
		return -ENOTSUP;
	}

	return 0;
}

static int crc_wch_finish(const struct device *dev, struct crc_ctx *ctx)
{
	const struct crc_wch_config *cfg = dev->config;

	ctx->result = cfg->regs->DATAR;
	ctx->state = CRC_STATE_IDLE;

	return 0;
}

static const struct crc_driver_api crc_wch_api = {
	.begin = crc_wch_begin,
	.update = crc_wch_update,
	.finish = crc_wch_finish,
};

#ifdef CONFIG_PM_DEVICE
static int crc_wch_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct crc_wch_config *cfg = dev->config;
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		err = clock_control_off(cfg->clock_dev, (clock_control_subsys_t)(uintptr_t)cfg->clock_subsys);
		if (err < 0) {
			return err;
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		err = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)(uintptr_t)cfg->clock_subsys);
		if (err < 0) {
			return err;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

static int crc_wch_init(const struct device *dev)
{
	const struct crc_wch_config *cfg = dev->config;

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	clock_control_on(cfg->clock_dev, (clock_control_subsys_t)(uintptr_t)cfg->clock_subsys);

	return 0;
}

#define CRC_WCH_INIT(n)                                                                    \
	static const struct crc_wch_config crc_wch_config_##n = {                          \
		.regs = (CRC_TypeDef *)DT_INST_REG_ADDR(n),                                \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                        \
		.clock_subsys = DT_INST_CLOCKS_CELL(n, id),                                \
	};                                                                                 \
                                                                                           \
	PM_DEVICE_DT_INST_DEFINE(n, crc_wch_pm_action);                                    \
                                                                                           \
	DEVICE_DT_INST_DEFINE(n, crc_wch_init, PM_DEVICE_DT_INST_GET(n), NULL,             \
			      &crc_wch_config_##n, POST_KERNEL,                            \
			      CONFIG_CRC_INIT_PRIORITY, &crc_wch_api);

DT_INST_FOREACH_STATUS_OKAY(CRC_WCH_INIT)
