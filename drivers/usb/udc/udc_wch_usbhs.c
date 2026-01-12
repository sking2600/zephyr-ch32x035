/*
 * Copyright (c) 2026 Scott King
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udc_wch_usbhs.h"
#include "udc_common.h"
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>

LOG_MODULE_REGISTER(udc_wch_usbhs, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define DT_DRV_COMPAT wch_usbhs

static int wch_usbhs_ep_enqueue(const struct device *dev,
				struct udc_ep_config *cfg,
				struct net_buf *buf)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbhs_get_ep_idx(cfg->addr);
	
	LOG_DBG("enqueue ep 0x%02x len %u", cfg->addr, buf->len);

	udc_buf_put(cfg, buf);
	
	if (udc_ep_is_busy(cfg)) {
		return 0;
	}
	
	udc_ep_set_busy(cfg, true);

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		wch_usbhs_set_tx_dma(usb, ep_idx, (uint32_t)buf->data);
		wch_usbhs_set_tx_len(usb, ep_idx, buf->len);
		
		uint8_t ctrl = wch_usbhs_get_tx_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBHS_UEP_T_RES_MASK;
		ctrl |= WCH_USBHS_UEP_T_RES_ACK;
		wch_usbhs_set_tx_ctrl(usb, ep_idx, ctrl);
	} else {
		wch_usbhs_set_rx_dma(usb, ep_idx, (uint32_t)buf->data);
		
		uint8_t ctrl = wch_usbhs_get_rx_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBHS_UEP_R_RES_MASK;
		ctrl |= WCH_USBHS_UEP_R_RES_ACK;
		wch_usbhs_set_rx_ctrl(usb, ep_idx, ctrl);
	}

	return 0;
}

static int wch_usbhs_ep_dequeue(const struct device *dev,
				struct udc_ep_config *cfg)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbhs_get_ep_idx(cfg->addr);
	struct net_buf *buf;

	LOG_DBG("dequeue ep 0x%02x", cfg->addr);
	
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		uint8_t ctrl = wch_usbhs_get_tx_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBHS_UEP_T_RES_MASK;
		ctrl |= WCH_USBHS_UEP_T_RES_NAK;
		wch_usbhs_set_tx_ctrl(usb, ep_idx, ctrl);
	} else {
		uint8_t ctrl = wch_usbhs_get_rx_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBHS_UEP_R_RES_MASK;
		ctrl |= WCH_USBHS_UEP_R_RES_NAK;
		wch_usbhs_set_rx_ctrl(usb, ep_idx, ctrl);
	}

	udc_ep_set_busy(cfg, false);
	
	buf = udc_buf_get_all(cfg);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}
	
	return 0;
}

static int wch_usbhs_ep_enable(const struct device *dev,
			       struct udc_ep_config *cfg)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbhs_get_ep_idx(cfg->addr);
	
	LOG_DBG("enable ep 0x%02x", cfg->addr);

	if (ep_idx == 0) {
		return 0;
	}
	
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		usb->ENDP_CONFIG |= (1 << ep_idx); /* UEPn_T_EN */
		wch_usbhs_set_tx_ctrl(usb, ep_idx, WCH_USBHS_UEP_T_RES_NAK);
	} else {
		usb->ENDP_CONFIG |= (1 << (ep_idx + 16)); /* UEPn_R_EN */
		wch_usbhs_set_max_len(usb, ep_idx, cfg->mps);
		wch_usbhs_set_rx_ctrl(usb, ep_idx, WCH_USBHS_UEP_R_RES_NAK);
	}
	
	return 0;
}

static int wch_usbhs_ep_disable(const struct device *dev,
				struct udc_ep_config *cfg)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbhs_get_ep_idx(cfg->addr);

	LOG_DBG("disable ep 0x%02x", cfg->addr);
	
	if (ep_idx == 0) return 0;
	
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		usb->ENDP_CONFIG &= ~(1 << ep_idx);
	} else {
		usb->ENDP_CONFIG &= ~(1 << (ep_idx + 16));
	}
	return 0;
}

static int wch_usbhs_ep_set_halt(const struct device *dev,
				 struct udc_ep_config *cfg)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbhs_get_ep_idx(cfg->addr);
	
	LOG_DBG("set halt ep 0x%02x", cfg->addr);
	
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		uint8_t ctrl = wch_usbhs_get_tx_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBHS_UEP_T_RES_MASK;
		ctrl |= WCH_USBHS_UEP_T_RES_STALL;
		wch_usbhs_set_tx_ctrl(usb, ep_idx, ctrl);
	} else {
		uint8_t ctrl = wch_usbhs_get_rx_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBHS_UEP_R_RES_MASK;
		ctrl |= WCH_USBHS_UEP_R_RES_STALL;
		wch_usbhs_set_rx_ctrl(usb, ep_idx, ctrl);
	}
	
	return 0;
}

static int wch_usbhs_ep_clear_halt(const struct device *dev,
				   struct udc_ep_config *cfg)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbhs_get_ep_idx(cfg->addr);
	
	LOG_DBG("clear halt ep 0x%02x", cfg->addr);

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		wch_usbhs_set_tx_ctrl(usb, ep_idx, WCH_USBHS_UEP_T_RES_NAK | WCH_USBHS_UEP_T_TOG);
	} else {
		wch_usbhs_set_rx_ctrl(usb, ep_idx, WCH_USBHS_UEP_R_RES_NAK | WCH_USBHS_UEP_R_TOG);
	}
	
	return 0;
}

static int wch_usbhs_set_address(const struct device *dev, const uint8_t addr)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;

	LOG_DBG("set address %d", addr);
	usb->DEV_AD = (addr & 0x7F);
	return 0;
}

static enum udc_bus_speed wch_usbhs_device_speed(const struct device *dev)
{
	const struct wch_usbhs_config *cfg = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)cfg->base;
	uint8_t st = usb->SPEED_TYPE;

	if ((st & 0x03) == 0x01) {
		return UDC_BUS_SPEED_HS;
	}
	return UDC_BUS_SPEED_FS;
}

static int wch_usbhs_enable(const struct device *dev)
{
	const struct wch_usbhs_config *cfg = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)cfg->base;

	LOG_DBG("enable");

	usb->INT_EN = WCH_USBHS_UIE_SUSPEND | WCH_USBHS_UIE_BUS_RST | WCH_USBHS_UIE_TRANSFER;
	usb->CONTROL = WCH_USBHS_UC_DEV_PU_EN | WCH_USBHS_UC_INT_BUSY | WCH_USBHS_UC_DMA_EN;

	cfg->irq_enable_func(dev);

	return 0;
}

static int wch_usbhs_disable(const struct device *dev)
{
	const struct wch_usbhs_config *cfg = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)cfg->base;

	LOG_DBG("disable");

	usb->CONTROL = WCH_USBHS_UC_RESET_SIE | WCH_USBHS_UC_CLR_ALL;
	k_busy_wait(10);
	usb->CONTROL = 0x00;

	return 0;
}

static int wch_usbhs_init(const struct device *dev)
{
	const struct wch_usbhs_config *cfg = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)cfg->base;
	int ret;

	LOG_DBG("init");

	/* Enable clock */
	RCC->AHBPCENR |= RCC_AHBPeriph_USBHS;
	
	/* Configure USBHS PLL: 8MHz HSE -> 480MHz PLL */
	/* bits 24-26: DIV, 27: SRC(1=HSE), 28-29: REF(2=8M), 30: PLLEN, 31: USBHS_SEL(1=PLL) */
	RCC->CFGR2 &= ~(RCC_USBHSDIV_MASK | RCC_USBHSPLLSRC | (3 << 28) | RCC_USBHSPLL | RCC_USBHSSRC);
	RCC->CFGR2 |= (2 << 28) | RCC_USBHSPLLSRC; /* 8M Ref, HSE Source */
	RCC->CFGR2 |= RCC_USBHSPLL;
	
	/* Wait for USBHS PLL stability (bit 30 often reads back as status or we just wait) */
	k_busy_wait(1000); 
	
	RCC->CFGR2 |= RCC_USBHSSRC; /* Select PLL as source */

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) return ret;

	usb->CONTROL = WCH_USBHS_UC_RESET_SIE | WCH_USBHS_UC_CLR_ALL;
	k_busy_wait(10);
	usb->CONTROL = 0x00;
	
	usb->ENDP_CONFIG = 0x00010001; /* EP0 RX and TX EN */
	usb->UEP0_MAX_LEN = 64;
	
	struct wch_usbhs_data *priv = udc_get_private(dev);
	usb->UEP0_DMA = (uint32_t)priv->setup_buf;

	return 0;
}

static void wch_usbhs_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void wch_usbhs_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static const struct udc_api wch_usbhs_api = {
	.lock = wch_usbhs_lock,
	.unlock = wch_usbhs_unlock,
	.device_speed = wch_usbhs_device_speed,
	.init = wch_usbhs_init,
	.enable = wch_usbhs_enable,
	.disable = wch_usbhs_disable,
	.shutdown = NULL,
	.set_address = wch_usbhs_set_address,
	.host_wakeup = NULL,
	.ep_enable = wch_usbhs_ep_enable,
	.ep_disable = wch_usbhs_ep_disable,
	.ep_set_halt = wch_usbhs_ep_set_halt,
	.ep_clear_halt = wch_usbhs_ep_clear_halt,
	.ep_enqueue = wch_usbhs_ep_enqueue,
	.ep_dequeue = wch_usbhs_ep_dequeue,
};

static void wch_usbhs_isr_transfer(const struct device *dev)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;
	uint8_t intst = usb->INT_ST;
	uint8_t ep_idx = intst & WCH_USBHS_UIS_ENDP_MASK;
	uint8_t token = intst & WCH_USBHS_UIS_TOKEN_MASK;
	struct wch_usbhs_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;

	switch (token) {
	case WCH_USBHS_UIS_TOKEN_SETUP:
		buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);
		if (buf) {
			udc_ep_buf_set_setup(buf);
			net_buf_add_mem(buf, priv->setup_buf, 8);
			udc_ctrl_update_stage(dev, buf);
		}
		break;
	case WCH_USBHS_UIS_TOKEN_IN:
		ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | ep_idx);
		buf = udc_buf_get(ep_cfg);
		udc_ep_set_busy(ep_cfg, false);
		if (buf) {
			udc_submit_ep_event(dev, buf, 0);
		}
		break;
	case WCH_USBHS_UIS_TOKEN_OUT:
		ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | ep_idx);
		buf = udc_buf_get(ep_cfg);
		udc_ep_set_busy(ep_cfg, false);
		if (buf) {
			uint16_t rx_len = usb->RX_LEN;
			net_buf_add(buf, rx_len);
			udc_submit_ep_event(dev, buf, 0);
		}
		break;
	}
	usb->INT_FG = WCH_USBHS_UIF_TRANSFER;
}

static void wch_usbhs_isr(const struct device *dev)
{
	const struct wch_usbhs_config *config = dev->config;
	WCH_USBHS_RegDef *usb = (WCH_USBHS_RegDef *)config->base;
	uint8_t intflag = usb->INT_FG;
	
	if (intflag & WCH_USBHS_UIF_TRANSFER) {
		wch_usbhs_isr_transfer(dev);
	}
	if (intflag & WCH_USBHS_UIF_BUS_RST) {
		usb->INT_FG = WCH_USBHS_UIF_BUS_RST;
		usb->DEV_AD = 0;
		udc_submit_event(dev, UDC_EVT_RESET, 0);
	}
	if (intflag & WCH_USBHS_UIF_SUSPEND) {
		usb->INT_FG = WCH_USBHS_UIF_SUSPEND;
		udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
	}
}

static int wch_usbhs_driver_init(const struct device *dev)
{
	struct udc_data *data = dev->data;
	const struct wch_usbhs_config *cfg = dev->config;
	int i;

	k_mutex_init(&data->mutex);
	data->caps.rwup = 1;
	data->caps.mps0 = UDC_MPS0_64;
	data->caps.hs = 1;

	for (i = 0; i < cfg->num_of_eps; i++) {
		cfg->ep_cfg_out[i].caps.out = 1;
		cfg->ep_cfg_out[i].caps.mps = 512;
		if (i == 0) {
			cfg->ep_cfg_out[i].caps.control = 1;
			cfg->ep_cfg_out[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_out[i].caps.bulk = 1;
			cfg->ep_cfg_out[i].caps.interrupt = 1;
			cfg->ep_cfg_out[i].caps.iso = 1;
		}
		cfg->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		udc_register_ep(dev, &cfg->ep_cfg_out[i]);

		cfg->ep_cfg_in[i].caps.in = 1;
		cfg->ep_cfg_in[i].caps.mps = 512;
		if (i == 0) {
			cfg->ep_cfg_in[i].caps.control = 1;
			cfg->ep_cfg_in[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_in[i].caps.bulk = 1;
			cfg->ep_cfg_in[i].caps.interrupt = 1;
			cfg->ep_cfg_in[i].caps.iso = 1;
		}
		cfg->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		udc_register_ep(dev, &cfg->ep_cfg_in[i]);
	}

	return 0;
}

#define WCH_USBHS_INIT(n)						\
	PINCTRL_DT_INST_DEFINE(n);					\
	static void wch_usbhs_irq_enable_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    wch_usbhs_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
	static struct wch_usbhs_data wch_usbhs_data_##n;		\
	static struct udc_ep_config ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)]; \
	static struct udc_ep_config ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)]; \
	static const struct wch_usbhs_config wch_usbhs_config_##n = {	\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),	\
		.ep_cfg_in = ep_cfg_in_##n,				\
		.ep_cfg_out = ep_cfg_out_##n,				\
		.base = DT_INST_REG_ADDR(n),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.irq_enable_func = wch_usbhs_irq_enable_##n,		\
	};								\
	static struct udc_data udc_data_##n = {				\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),	\
		.priv = &wch_usbhs_data_##n,				\
	};								\
	DEVICE_DT_INST_DEFINE(n, wch_usbhs_driver_init, NULL,		\
			    &udc_data_##n, &wch_usbhs_config_##n,	\
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			    &wch_usbhs_api);

DT_INST_FOREACH_STATUS_OKAY(WCH_USBHS_INIT)
