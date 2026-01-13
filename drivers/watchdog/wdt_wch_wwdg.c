/*
 * Copyright (c) 2019 Centaur Analytics, Inc
 * Copyright (c) 2024 WCH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_wwdg

#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <soc.h>
#include <stdint.h>

LOG_MODULE_REGISTER(wdt_wwdg, CONFIG_WDT_LOG_LEVEL);

/* WWDG Control Register (CTLR) */
#define WWDG_CTLR_T_6           BIT(6)
#define WWDG_CTLR_WDGA          BIT(7)

/* WWDG Configuration Register (CFGR) */
#define WWDG_CFGR_W_6           BIT(6)
#define WWDG_CFGR_EWI           BIT(9)

/* WWDG Status Register (STATR) */
#define WWDG_STATR_EWIF         BIT(0)

/* Counter valid range: 0x40 to 0x7F */
#define WWDG_COUNTER_MIN        0x40
#define WWDG_COUNTER_MAX        0x7F

struct wdt_wch_data {
	wdt_callback_t callback;
	uint32_t timeout;
	uint8_t window;
	uint8_t counter;
	bool installed;
};

struct wdt_wch_config {
	WWDG_TypeDef *wwdg;
};

static uint32_t wdt_wch_get_pclk(void)
{
	/* Assuming PCLK1 is used for WWDG and is 144MHz/2 = 72MHz or similar configuration */
	/* Ideally we get this from RCC driver or system core clock */
	return 72000000; /* Placeholder: Should be SystemCoreClock >> PPRE1 */
}

static int wdt_wch_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_wch_config *cfg = dev->config;
	struct wdt_wch_data *data = dev->data;

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		/* Not supported without DBGMCU */
	}

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		/* Not fully supported */
	}

	/* Write window value to CFGR */
	cfg->wwdg->CFGR = (data->window & 0x7F) | WWDG_CFGR_EWI;

	/* Write counter value and enable bit to CTLR */
	cfg->wwdg->CTLR = WWDG_CTLR_WDGA | (data->counter & 0x7F);

	return 0;
}

static int wdt_wch_disable(const struct device *dev)
{
	return -EPERM;
}

static int wdt_wch_install_timeout(const struct device *dev,
				   const struct wdt_timeout_cfg *config)
{
	struct wdt_wch_data *data = dev->data;
	uint32_t pclk = wdt_wch_get_pclk();
	uint32_t prescaler = 8; /* Fixed /8 prescaler in logic for simplicity or from WDGTB */
	uint32_t wdgtb = 3; /* WDGTB=3 -> Div 8 */
	uint64_t ticks;
	
	/* WWDG clock = PCLK1 / 4096 / Prescaler */
	/* Prescaler = 1, 2, 4, 8 via WDGTB[1:0] */
	
	if (config->window.max == 0) {
		return -EINVAL;
	}
	
	/* Simple logic: Use max prescaler (8) to get max timeout */
	/* T_wwdg = 1/PCLK * 4096 * 8 * (Counter - 0x40 + 1) */
	
	/* Calculate counter value needed for timeout */
	/* timeout_ms = t_wwdg_cycle_ms * (counter - 0x3F) */
	
	/* We need deeper logic for precise timeout calc, but for now: */
	data->counter = WWDG_COUNTER_MAX; 
	data->window  = WWDG_COUNTER_MAX; /* Open window */
	data->callback = config->callback;
	data->installed = true;

	return 0;
}

static int wdt_wch_feed(const struct device *dev, int channel_id)
{
	const struct wdt_wch_config *cfg = dev->config;
	struct wdt_wch_data *data = dev->data;

	ARG_UNUSED(channel_id);

	/* Refresh counter, keeping WDGA bit set */
	cfg->wwdg->CTLR = WWDG_CTLR_WDGA | (data->counter & 0x7F);

	return 0;
}

static void wdt_wch_isr(const struct device *dev)
{
	const struct wdt_wch_config *cfg = dev->config;
	struct wdt_wch_data *data = dev->data;

	if (cfg->wwdg->STATR & WWDG_STATR_EWIF) {
		cfg->wwdg->STATR &= ~WWDG_STATR_EWIF; /* Clear flag */
		if (data->callback) {
			data->callback(dev, 0);
		}
	}
}

static const struct wdt_driver_api wdt_wch_api = {
	.setup = wdt_wch_setup,
	.disable = wdt_wch_disable,
	.install_timeout = wdt_wch_install_timeout,
	.feed = wdt_wch_feed,
};

static int wdt_wch_init(const struct device *dev)
{
	const struct wdt_wch_config *cfg = dev->config;
	
	/* Connect IRQ */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		    wdt_wch_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
	
	/* Enable APB1 clock for WWDG - should be done by Zephyr clock control but doing manually if needed for WCH */
	/* RCC->APB1PCENR |= RCC_APB1Periph_WWDG; */

	return 0;
}

#define WCH_WWDG_INIT(n)                                                       \
	static struct wdt_wch_data wdt_wch_data_##n;                           \
	static const struct wdt_wch_config wdt_wch_config_##n = {              \
		.wwdg = (WWDG_TypeDef *)DT_INST_REG_ADDR(n),                   \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, wdt_wch_init, NULL,                           \
			      &wdt_wch_data_##n, &wdt_wch_config_##n,          \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &wdt_wch_api);

DT_INST_FOREACH_STATUS_OKAY(WCH_WWDG_INIT)
