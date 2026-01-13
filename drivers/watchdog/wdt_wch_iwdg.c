/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2017 RnDity Sp. z o.o.
 */

#define DT_DRV_COMPAT wch_ch32_watchdog

#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <soc.h>
#include <stdint.h>

LOG_MODULE_REGISTER(wdt_iwdg, CONFIG_WDT_LOG_LEVEL);

#define IWDG_KEY_RELOAD         0xAAAA
#define IWDG_KEY_ENABLE         0xCCCC
#define IWDG_KEY_WRITE_ACCESS   0x5555
#define IWDG_KEY_DISABLE        0x0000

#define IWDG_SR_PVU             BIT(0)
#define IWDG_SR_RVU             BIT(1)

struct wdt_wch_data {
	bool installed;
};

struct wdt_wch_config {
	IWDG_TypeDef *iwdg;
};

static void wdt_wch_wait_ready(IWDG_TypeDef *iwdg)
{
	while (iwdg->STATR & (IWDG_SR_PVU | IWDG_SR_RVU)) {
	}
}

static int wdt_wch_disable(const struct device *dev)
{
	/* IWDG cannot be disabled once enabled by software */
	return -EPERM;
}

static int wdt_wch_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_wch_config *cfg = dev->config;

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		/* Not supported by standard IWDG without DBGMCU magic */
	}

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		/* Not fully supported without additional configuration */
	}

	wdt_wch_wait_ready(cfg->iwdg);
	cfg->iwdg->CTLR = IWDG_KEY_ENABLE;

	return 0;
}

static int wdt_wch_install_timeout(const struct device *dev,
				   const struct wdt_timeout_cfg *config)
{
	const struct wdt_wch_config *cfg = dev->config;
	struct wdt_wch_data *data = dev->data;
	uint32_t prescaler = 0; /* Align with PSCR values: 0->4, 1->8 ... */
	uint32_t reload = 0;
	uint32_t lsi_freq = 40000; /* Approximate LSI frequency (40kHz) */
	uint64_t ticks;
	int i;

	if (config->window.min != 0 || config->window.max == 0) {
		return -EINVAL;
	}

	if (data->installed) {
		return -EBUSY;
	}

	/* WCH LSI is typically 40kHz (L103) or 32kHz (V003/X035) */ 
	/* We assume 40kHz for now, better to get from RCC if possible */
	/* Prescaler divider: 4, 8, 16, ..., 256 */
	/* PSCR value: 0->4, 1->8, 2->16, 3->32, 4->64, 5->128, 6->256 */

	for (i = 0; i <= 6; i++) {
		uint32_t div = 4 << i;
		ticks = (uint64_t)config->window.max * lsi_freq / 1000 / div;
		if (ticks <= 0xFFF) {
			reload = ticks;
			prescaler = i;
			break;
		}
	}

	if (i > 6) {
		return -EINVAL;
	}

	wdt_wch_wait_ready(cfg->iwdg);
	cfg->iwdg->CTLR = IWDG_KEY_WRITE_ACCESS;
	cfg->iwdg->PSCR = prescaler;
	cfg->iwdg->RLDR = reload;
	cfg->iwdg->CTLR = IWDG_KEY_RELOAD;
	
	wdt_wch_wait_ready(cfg->iwdg);

	data->installed = true;

	return 0;
}

static int wdt_wch_feed(const struct device *dev, int channel_id)
{
	const struct wdt_wch_config *cfg = dev->config;

	ARG_UNUSED(channel_id);

	cfg->iwdg->CTLR = IWDG_KEY_RELOAD;

	return 0;
}

static const struct wdt_driver_api wdt_wch_api = {
	.setup = wdt_wch_setup,
	.disable = wdt_wch_disable,
	.install_timeout = wdt_wch_install_timeout,
	.feed = wdt_wch_feed,
};

static int wdt_wch_init(const struct device *dev)
{
	/* LSI is enabled by default or by RCC, IWDG is enabled by KEY */
	return 0;
}

#define WCH_IWDG_INIT(n)                                                       \
	static struct wdt_wch_data wdt_wch_data_##n;                           \
	static const struct wdt_wch_config wdt_wch_config_##n = {              \
		.iwdg = (IWDG_TypeDef *)DT_INST_REG_ADDR(n),                   \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, wdt_wch_init, NULL,                           \
			      &wdt_wch_data_##n, &wdt_wch_config_##n,          \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &wdt_wch_api);

DT_INST_FOREACH_STATUS_OKAY(WCH_IWDG_INIT)
