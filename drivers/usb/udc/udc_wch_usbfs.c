/*
 * Copyright (c) 2025 Scott King
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udc_wch_usbfs.h"
#include "udc_common.h"
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>

LOG_MODULE_REGISTER(udc_wch_usbfs, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define DT_DRV_COMPAT wch_usbfs




static int wch_usbfs_ep_enqueue(const struct device *dev,
				struct udc_ep_config *cfg,
				struct net_buf *buf)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbfs_get_ep_idx(cfg->addr);
	
	LOG_DBG("enqueue ep 0x%02x len %u", cfg->addr, buf->len);

	if (ep_idx == 0) {
		/* Handle EP0 specially? Usually handled by UDC core or setup ISR */
		/* But for Data stage, we need to setup DMA */
	}

	udc_buf_put(cfg, buf);
	
	if (udc_ep_is_busy(cfg)) {
		return 0;
	}
	
	udc_ep_set_busy(cfg, true);

	wch_usbfs_set_dma(usb, ep_idx, (uint32_t)buf->data);

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		wch_usbfs_set_tx_len(usb, ep_idx, buf->len);
		/* Set RESP to ACK to verify transmission */
		/* Hardware toggles bit automatically if configured? */
		/* Valid behavior: TOG is managed, just set ACK */
		uint8_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBFS_UEP_T_RES_MASK;
		ctrl |= WCH_USBFS_UEP_T_RES_ACK;
		/* Assuming TOG is already correct or auto-managed by HW on ACK */
		wch_usbfs_set_ctrl(usb, ep_idx, ctrl);
	} else {
		/* For OUT, we just set DMA and wait, but ensure RESP is ACK */
		/* Normally we preset ACK in previous step? Or here? */
		/* If we were NAKing, now we ACK */
		uint8_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBFS_UEP_R_RES_MASK;
		ctrl |= WCH_USBFS_UEP_R_RES_ACK;
		wch_usbfs_set_ctrl(usb, ep_idx, ctrl);
	}

	return 0;
}

static int wch_usbfs_ep_dequeue(const struct device *dev,
				struct udc_ep_config *cfg)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbfs_get_ep_idx(cfg->addr);
	struct net_buf *buf;

	LOG_DBG("dequeue ep 0x%02x", cfg->addr);
	
	/* Stop transfer: Set NAK */
	uint8_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		ctrl &= ~WCH_USBFS_UEP_T_RES_MASK;
		ctrl |= WCH_USBFS_UEP_T_RES_NAK;
	} else {
		ctrl &= ~WCH_USBFS_UEP_R_RES_MASK;
		ctrl |= WCH_USBFS_UEP_R_RES_NAK;
	}
	wch_usbfs_set_ctrl(usb, ep_idx, ctrl);

	udc_ep_set_busy(cfg, false);
	
	buf = udc_buf_get_all(cfg);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}
	
	return 0;
}

static int wch_usbfs_ep_enable(const struct device *dev,
			       struct udc_ep_config *cfg)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbfs_get_ep_idx(cfg->addr);
	
	LOG_DBG("enable ep 0x%02x", cfg->addr);

	if (ep_idx == 0) {
		/* EP0 is always enabled */
		return 0;
	}
	
	wch_usbfs_ep_set_mod(usb, ep_idx, true);
	
	/* Reset TOG */
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		wch_usbfs_set_ctrl(usb, ep_idx, 
			(wch_usbfs_get_ctrl(usb, ep_idx) & ~WCH_USBFS_UEP_T_RES_MASK) | WCH_USBFS_UEP_T_RES_NAK | WCH_USBFS_UEP_T_TOG);
	} else {
		wch_usbfs_set_ctrl(usb, ep_idx, 
			(wch_usbfs_get_ctrl(usb, ep_idx) & ~WCH_USBFS_UEP_R_RES_MASK) | WCH_USBFS_UEP_R_RES_NAK | WCH_USBFS_UEP_R_TOG);
	}
	
	return 0;
}

static int wch_usbfs_ep_disable(const struct device *dev,
				struct udc_ep_config *cfg)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbfs_get_ep_idx(cfg->addr);

	LOG_DBG("disable ep 0x%02x", cfg->addr);
	
	if (ep_idx == 0) return 0;
	
	wch_usbfs_ep_set_mod(usb, ep_idx, false);
	return 0;
}

static int wch_usbfs_ep_set_halt(const struct device *dev,
				 struct udc_ep_config *cfg)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbfs_get_ep_idx(cfg->addr);
	
	LOG_DBG("set halt ep 0x%02x", cfg->addr);
	
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		wch_usbfs_set_ctrl(usb, ep_idx, 
			(wch_usbfs_get_ctrl(usb, ep_idx) & ~WCH_USBFS_UEP_T_RES_MASK) | WCH_USBFS_UEP_T_RES_STALL);
	} else {
		wch_usbfs_set_ctrl(usb, ep_idx, 
			(wch_usbfs_get_ctrl(usb, ep_idx) & ~WCH_USBFS_UEP_R_RES_MASK) | WCH_USBFS_UEP_R_RES_STALL);
	}
	
	return 0;
}

static int wch_usbfs_ep_clear_halt(const struct device *dev,
				   struct udc_ep_config *cfg)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbfs_get_ep_idx(cfg->addr);
	
	LOG_DBG("clear halt ep 0x%02x", cfg->addr);

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		wch_usbfs_set_ctrl(usb, ep_idx, 
			(wch_usbfs_get_ctrl(usb, ep_idx) & ~WCH_USBFS_UEP_T_RES_MASK) | WCH_USBFS_UEP_T_RES_NAK | WCH_USBFS_UEP_T_TOG);
	} else {
		wch_usbfs_set_ctrl(usb, ep_idx, 
			(wch_usbfs_get_ctrl(usb, ep_idx) & ~WCH_USBFS_UEP_R_RES_MASK) | WCH_USBFS_UEP_R_RES_NAK | WCH_USBFS_UEP_R_TOG);
	}
	
	return 0;
}

static int wch_usbfs_set_address(const struct device *dev, const uint8_t addr)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;

	LOG_DBG("set address %d", addr);
	usb->DEV_ADDR = (addr & 0x7F);
	return 0;
}

static int wch_usbfs_host_wakeup(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP;
}

static enum udc_bus_speed wch_usbfs_device_speed(const struct device *dev)
{
	ARG_UNUSED(dev);
	return UDC_BUS_SPEED_FS;
}

static int wch_usbfs_enable(const struct device *dev)
{
	const struct wch_usbfs_config *cfg = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)cfg->base;

	LOG_DBG("enable");

	/* Enable USBFS interrupts */
	usb->INT_EN = WCH_USBFS_UIE_SUSPEND | WCH_USBFS_UIE_BUS_RST | WCH_USBFS_UIE_TRANSFER;

	/* Enable Device, DMA, and Interrupts at the controller level */
	usb->BASE_CTRL = WCH_USBFS_UC_DEV_PU_EN | WCH_USBFS_UC_INT_BUSY | WCH_USBFS_UC_DMA_EN;

	/* Enable Pull-up to signal connection */
	usb->UDEV_CTRL = WCH_USBFS_UD_PD_DIS | WCH_USBFS_UD_PORT_EN;

	cfg->irq_enable_func(dev);

	return 0;
}

static int wch_usbfs_disable(const struct device *dev)
{
	const struct wch_usbfs_config *cfg = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)cfg->base;

	LOG_DBG("disable");

	/* Disable Pull-up */
	usb->UDEV_CTRL = WCH_USBFS_UD_PD_DIS;
	
	/* Reset controller */
	usb->BASE_CTRL = WCH_USBFS_UC_RESET_SIE | WCH_USBFS_UC_CLR_ALL;
	k_busy_wait(10);
	usb->BASE_CTRL = 0x00;

	return 0;
}

static int wch_usbfs_init(const struct device *dev)
{
	const struct wch_usbfs_config *cfg = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)cfg->base;
	int ret;

	LOG_DBG("init");

	if (cfg->clock_enable_func) {
		ret = cfg->clock_enable_func(dev);
		if (ret < 0) {
			return ret;
		}
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Reset the USBFS peripheral */
	usb->BASE_CTRL = WCH_USBFS_UC_RESET_SIE | WCH_USBFS_UC_CLR_ALL;
	k_busy_wait(10);
	usb->BASE_CTRL = 0x00;
	
	/* Basic configuration */
#if defined(CONFIG_SOC_CH32X035)
	/* X035: UEP4_1_MOD, UEP2_3_MOD, UEP567_MOD */
	usb->UEP4_1_MOD = WCH_USBFS_UEP1_TX_EN | WCH_USBFS_UEP1_RX_EN |
			     WCH_USBFS_UEP4_TX_EN | WCH_USBFS_UEP4_RX_EN;
	usb->UEP2_3_MOD = WCH_USBFS_UEP2_TX_EN | WCH_USBFS_UEP2_RX_EN |
			     WCH_USBFS_UEP3_TX_EN | WCH_USBFS_UEP3_RX_EN;
	usb->UEP567_MOD = WCH_USBFS_UEP5_TX_EN | WCH_USBFS_UEP5_RX_EN |
			     WCH_USBFS_UEP6_TX_EN | WCH_USBFS_UEP6_RX_EN |
			     WCH_USBFS_UEP7_TX_EN | WCH_USBFS_UEP7_RX_EN;
#else
	/* L103 */
	usb->UEP4_1_MOD = WCH_USBFS_UEP1_TX_EN | WCH_USBFS_UEP1_RX_EN |
			     WCH_USBFS_UEP4_TX_EN | WCH_USBFS_UEP4_RX_EN;
	usb->UEP2_3_MOD = WCH_USBFS_UEP2_TX_EN | WCH_USBFS_UEP2_RX_EN |
			     WCH_USBFS_UEP3_TX_EN | WCH_USBFS_UEP3_RX_EN;
	usb->UEP5_6_MOD = WCH_USBFS_UEP5_TX_EN | WCH_USBFS_UEP5_RX_EN |
			     WCH_USBFS_UEP6_TX_EN | WCH_USBFS_UEP6_RX_EN;
	usb->UEP7_MOD = WCH_USBFS_UEP7_TX_EN | WCH_USBFS_UEP7_RX_EN;
#endif

	/* EP0 setup buffer is special, fixed location */
	struct wch_usbfs_data *priv = udc_get_private(dev);
	wch_usbfs_set_dma(usb, 0, (uint32_t)priv->setup_buf);
	
	/* Initialize Control EP0 */
	usb->UEP0_TX_LEN = 0;

	return 0;
}

static int wch_usbfs_shutdown(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static void wch_usbfs_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void wch_usbfs_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static const struct udc_api wch_usbfs_api = {
	.lock = wch_usbfs_lock,
	.unlock = wch_usbfs_unlock,
	.device_speed = wch_usbfs_device_speed,
	.init = wch_usbfs_init,
	.enable = wch_usbfs_enable,
	.disable = wch_usbfs_disable,
	.shutdown = wch_usbfs_shutdown,
	.set_address = wch_usbfs_set_address,
	.host_wakeup = wch_usbfs_host_wakeup,
	.ep_enable = wch_usbfs_ep_enable,
	.ep_disable = wch_usbfs_ep_disable,
	.ep_set_halt = wch_usbfs_ep_set_halt,
	.ep_clear_halt = wch_usbfs_ep_clear_halt,
	.ep_enqueue = wch_usbfs_ep_enqueue,
	.ep_dequeue = wch_usbfs_ep_dequeue,
};

static int wch_usbfs_driver_init(const struct device *dev)
{
	struct udc_data *data = dev->data;
	const struct wch_usbfs_config *cfg = dev->config;
	int i;

	k_mutex_init(&data->mutex);
	data->caps.rwup = 1;
	data->caps.mps0 = UDC_MPS0_64;

	/* Register endpoints */
	for (i = 0; i < cfg->num_of_eps; i++) {
		cfg->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			cfg->ep_cfg_out[i].caps.control = 1;
			cfg->ep_cfg_out[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_out[i].caps.bulk = 1;
			cfg->ep_cfg_out[i].caps.interrupt = 1;
			cfg->ep_cfg_out[i].caps.iso = 1;
			cfg->ep_cfg_out[i].caps.mps = 64;
		}
		cfg->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		udc_register_ep(dev, &cfg->ep_cfg_out[i]);

		cfg->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			cfg->ep_cfg_in[i].caps.control = 1;
			cfg->ep_cfg_in[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_in[i].caps.bulk = 1;
			cfg->ep_cfg_in[i].caps.interrupt = 1;
			cfg->ep_cfg_in[i].caps.iso = 1;
			cfg->ep_cfg_in[i].caps.mps = 64;
		}
		cfg->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		udc_register_ep(dev, &cfg->ep_cfg_in[i]);
	}

	return 0;
}

static void wch_usbfs_isr_transfer(const struct device *dev)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t intst = usb->INT_ST;
	uint8_t ep_idx = intst & WCH_USBFS_UIS_ENDP_MASK;
	uint8_t token = intst & WCH_USBFS_UIS_TOKEN_MASK;
	struct wch_usbfs_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;

	switch (token) {
	case WCH_USBFS_UIS_TOKEN_SETUP:
		/* EP0 Setup */
		LOG_DBG("SETUP ep 0");
		buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);
		if (buf) {
			udc_ep_buf_set_setup(buf);
			net_buf_add_mem(buf, priv->setup_buf, 8);
			udc_ctrl_update_stage(dev, buf);
		}
		break;

	case WCH_USBFS_UIS_TOKEN_IN:
		LOG_DBG("IN ep %d", ep_idx);
		ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | ep_idx);
		buf = udc_buf_get(ep_cfg);
		udc_ep_set_busy(ep_cfg, false);
		if (buf) {
			udc_submit_ep_event(dev, buf, 0);
		}
		break;

	case WCH_USBFS_UIS_TOKEN_OUT:
		LOG_DBG("OUT ep %d", ep_idx);
		ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | ep_idx);
		buf = udc_buf_get(ep_cfg);
		udc_ep_set_busy(ep_cfg, false);
		if (buf) {
			uint16_t rx_len = usb->RX_LEN;
			net_buf_add(buf, rx_len);
			udc_submit_ep_event(dev, buf, 0);
		}
		break;
		
	case WCH_USBFS_UIS_TOKEN_SOF:
		break;
	}
	
	usb->INT_FG = WCH_USBFS_UIF_TRANSFER;
}

static void wch_usbfs_isr(const struct device *dev)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t intflag = usb->INT_FG;
	
	if (intflag & WCH_USBFS_UIF_TRANSFER) {
		wch_usbfs_isr_transfer(dev);
	}
	
	if (intflag & WCH_USBFS_UIF_BUS_RST) {
		LOG_DBG("BUS_RST");
		usb->INT_FG = WCH_USBFS_UIF_BUS_RST;
		
		/* Reset device address */
		usb->DEV_ADDR = 0;
		
		/* Notify UDC */
		udc_submit_event(dev, UDC_EVT_RESET, 0);
	}
	
	if (intflag & WCH_USBFS_UIF_SUSPEND) {
		LOG_DBG("SUSPEND");
		usb->INT_FG = WCH_USBFS_UIF_SUSPEND;
		udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
	}
}

#define WCH_USBFS_INIT(n)						\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	static void wch_usbfs_irq_enable_##n(const struct device *dev)	\
	{								\
		ARG_UNUSED(dev);					\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    wch_usbfs_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
									\
	static struct wch_usbfs_data wch_usbfs_data_##n = {		\
	};								\
									\
	static struct udc_ep_config ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)]; \
	static struct udc_ep_config ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)]; \
									\
	static const struct wch_usbfs_config wch_usbfs_config_##n = {	\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),	\
		.ep_cfg_in = ep_cfg_in_##n,				\
		.ep_cfg_out = ep_cfg_out_##n,				\
		.base = DT_INST_REG_ADDR(n),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.irq_enable_func = wch_usbfs_irq_enable_##n,		\
		.clock_enable_func = NULL, /* TODO */			\
	};								\
									\
	static struct udc_data udc_data_##n = {				\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),	\
		.priv = &wch_usbfs_data_##n,				\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, wch_usbfs_driver_init, NULL,		\
			    &udc_data_##n, &wch_usbfs_config_##n,	\
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			    &wch_usbfs_api);

DT_INST_FOREACH_STATUS_OKAY(WCH_USBFS_INIT)
