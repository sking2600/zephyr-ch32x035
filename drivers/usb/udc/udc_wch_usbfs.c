/*
 * Copyright (c) 2025 Scott King
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udc_wch_usbfs.h"
#include "udc_common.h"
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/ch32l103_clock.h>
#include <zephyr/usb/usb_ch9.h>
#include <soc.h>

/* Helper definitions not in Zephyr headers yet */
#define WCH_USBFS_UIF_SOF       0x08
#define WCH_USBFS_UIE_SOF       0x08

LOG_MODULE_REGISTER(udc_wch_usbfs, CONFIG_UDC_DRIVER_LOG_LEVEL);

#ifndef DT_DRV_COMPAT
#define DT_DRV_COMPAT wch_usbfs
#endif


static void wch_usbfs_thread_handler(void *arg1, void *arg2, void *arg3);

#include <zephyr/usb/class/usb_cdc.h>

/* 
 * Fast-Path Descriptors
 * These are used by the ISR to handle enumeration immediately, bypassing the 
 * Zephyr UDC thread latency which causes timeouts on this platform.
 * 
 * NOTE: We use raw byte arrays here to ensure exact binary layout and avoid 
 * any potential struct padding/alignment issues that were observed with 
 * Zephyr structs on this platform (-110 timeouts).
 */
static const uint8_t wch_fs_dev_desc[] = {
	18,             /* bLength */
	1,              /* bDescriptorType: Device */
	0x10, 0x01,     /* bcdUSB: 1.10 */
	0x02,           /* bDeviceClass: CDC */
	0x00,           /* bDeviceSubClass */
	0x00,           /* bDeviceProtocol */
	64,             /* bMaxPacketSize0 */
	0x86, 0x1A,     /* idVendor: WCH */
	0x23, 0x57,     /* idProduct: CDC-ACM */
	0x00, 0x01,     /* bcdDevice: 1.00 */
	0x00,           /* iManufacturer: None */
	0x00,           /* iProduct: None */
	0x00,           /* iSerialNumber: None */
	0x01,           /* bNumConfigurations */
};

static const uint8_t wch_fs_cfg_desc[] = {
	/* Configuration Descriptor */
	9, 2, 67, 0, 2, 1, 0, 0x80, 50,
	/* Interface 0: CDC Control */
	9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,
	/* CDC Header */
	5, 0x24, 0x00, 0x10, 0x01,
	/* CDC Call Management */
	5, 0x24, 0x01, 0x00, 1,
	/* CDC ACM */
	4, 0x24, 0x02, 0x02,
	/* CDC Union */
	5, 0x24, 0x06, 0, 1,
	/* EP1 IN (Interrupt) */
	7, 5, 0x81, 0x03, 8, 0, 10,
	/* Interface 1: CDC Data */
	9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
	/* EP2 OUT (Bulk) */
	7, 5, 0x02, 0x02, 64, 0, 0,
	/* EP3 IN (Bulk) */
	7, 5, 0x83, 0x02, 64, 0, 0,
};





static int wch_usbfs_clock_on(const struct device *dev)
{
	const struct device *rcc = DEVICE_DT_GET(DT_NODELABEL(rcc));
	uint32_t cfgr0 = RCC->CFGR0;

	if (!device_is_ready(rcc)) {
		return -ENODEV;
	}

	/*
	 * Configure USB clock source from PLL (critical for USB PHY timing).
	 * USB requires 48MHz. RCC->CFGR0 bits [23:22] control USB prescaler:
	 *   00 = PLL/1  (for 48MHz sysclk)
	 *   01 = PLL/2  (for 96MHz sysclk)
	 *   10 = PLL/1.5 (for 72MHz sysclk)
	 * This matches WCH EXAM: RCC_USBCLKConfig()
	 */
	cfgr0 &= ~(0x3 << 22); /* Clear USBPRE bits */

	uint32_t sysclk = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;

	switch (sysclk) {
	case 96000000:
		cfgr0 |= (0x1 << 22); /* Div2: 96/2 = 48MHz */
		printk("UDC: USB clock 96MHz / 2\n");
		break;
	case 72000000:
		cfgr0 |= (0x2 << 22); /* Div1.5: 72/1.5 = 48MHz */
		printk("UDC: USB clock 72MHz / 1.5\n");
		break;
	case 48000000:
		cfgr0 |= (0x0 << 22); /* Div1: 48/1 = 48MHz */
		printk("UDC: USB clock 48MHz / 1\n");
		break;
	default:
		cfgr0 |= (0x1 << 22); /* Default Div2 */
		printk("UDC: WARN Unsupported sysclk %u, defaulting to Div2\n", sysclk);
		break;
	}

	RCC->CFGR0 = cfgr0;

	/* 
	 * Explicitly enable USBFS clock in HBPCENR (aliases to AHBPCENR).
	 * Match WCH EXAM: RCC_HBPeriphClockCmd(RCC_HBPeriph_USBFS, ENABLE)
	 */
#if defined(CONFIG_SOC_CH32L103)
	RCC->HBPCENR |= RCC_USBFSEN;
#endif

	return clock_control_on(rcc, (clock_control_subsys_t)RCC_AHB_USBFS);
}

static int wch_usbfs_ep_enqueue(const struct device *dev,
				struct udc_ep_config *cfg,
				struct net_buf *buf)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t ep_idx = wch_usbfs_get_ep_idx(cfg->addr);
	
	LOG_DBG("enqueue ep 0x%02x len %u", cfg->addr, buf->len);

	udc_buf_put(cfg, buf);
	
	if (udc_ep_is_busy(cfg)) {
		return 0;
	}
	
	udc_ep_set_busy(cfg, true);

	wch_usbfs_set_dma(usb, ep_idx, (uint32_t)buf->data);

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		wch_usbfs_set_tx_len(usb, ep_idx, buf->len);
		uint16_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
		ctrl &= ~WCH_USBFS_UEP_T_RES_MASK;
		ctrl |= WCH_USBFS_UEP_T_RES_ACK;
		wch_usbfs_set_ctrl(usb, ep_idx, ctrl);
	} else {
		/* OUT endpoint: Ready to receive */
		uint16_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
		ctrl &= ~(WCH_USBFS_UEP_R_RES_MASK << 8);
		ctrl |= (WCH_USBFS_UEP_R_RES_ACK << 8);
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
	uint16_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		ctrl &= ~WCH_USBFS_UEP_T_RES_MASK;
		ctrl |= WCH_USBFS_UEP_T_RES_NAK;
	} else {
		ctrl &= ~(WCH_USBFS_UEP_R_RES_MASK << 8);
		ctrl |= (WCH_USBFS_UEP_R_RES_NAK << 8);
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
		return 0;
	}
	
	/* Enable endpoint in MOD registers */
	wch_usbfs_ep_set_mod(usb, ep_idx, true);
	
	/* Set DATA1 toggle and NAK for initial state */
	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		wch_usbfs_set_ctrl(usb, ep_idx, 
			(wch_usbfs_get_ctrl(usb, ep_idx) & ~WCH_USBFS_UEP_T_RES_MASK) | 
			WCH_USBFS_UEP_T_RES_NAK | WCH_USBFS_UEP_T_TOG);
	} else {
		wch_usbfs_set_ctrl(usb, ep_idx, 
			(wch_usbfs_get_ctrl(usb, ep_idx) & ~WCH_USBFS_UEP_R_RES_MASK) | 
			WCH_USBFS_UEP_R_RES_NAK | WCH_USBFS_UEP_R_TOG);
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

	printk("UDC: enable\n");

	/* Enable device pull-up */
	usb->UDEV_CTRL = WCH_USBFS_UD_PD_DIS | WCH_USBFS_UD_PORT_EN;

	cfg->irq_enable_func(dev);

	return 0;
}

static int wch_usbfs_disable(const struct device *dev)
{
	const struct wch_usbfs_config *cfg = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)cfg->base;

	LOG_INF("disable");

	/* Reset controller to default state */
	usb->UDEV_CTRL = WCH_USBFS_UD_PD_DIS;
	usb->INT_EN = 0;
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

	LOG_INF("init");

	if (cfg->clock_enable_func) {
		ret = cfg->clock_enable_func(dev);
		if (ret < 0) {
			LOG_ERR("Clock enable failed: %d", ret);
			return ret;
		}
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Pinctrl apply failed: %d", ret);
		return ret;
	}

	/* Reset the USBFS peripheral (matches EXAM) */
	usb->BASE_CTRL = WCH_USBFS_UC_RESET_SIE | WCH_USBFS_UC_CLR_ALL;
	k_busy_wait(10);
	usb->BASE_CTRL = 0x00;
	
	/* 
	 * Initialize all endpoint MOD registers at startup to ensure 
	 * hardware state matches EXAM expectations for all endpoints.
	 */
	usb->UEP4_1_MOD = WCH_USBFS_UEP4_RX_EN | WCH_USBFS_UEP4_TX_EN | 
	                  WCH_USBFS_UEP1_RX_EN | WCH_USBFS_UEP1_TX_EN;
	usb->UEP2_3_MOD = WCH_USBFS_UEP2_RX_EN | WCH_USBFS_UEP2_TX_EN | 
	                  WCH_USBFS_UEP3_RX_EN | WCH_USBFS_UEP3_TX_EN;
	usb->UEP5_6_MOD = WCH_USBFS_UEP5_RX_EN | WCH_USBFS_UEP5_TX_EN | 
	                  WCH_USBFS_UEP6_RX_EN | WCH_USBFS_UEP6_TX_EN;
	usb->UEP7_MOD   = WCH_USBFS_UEP7_RX_EN | WCH_USBFS_UEP7_TX_EN;

	/* 
	 * DISABLED for debugging: HSI calibration
	 * wch_usbfs_hsi_calibrate_boot();
	 */
	
	/* Enable Interrupts: Match EXAM exactly (no SOF) */
	usb->INT_EN = WCH_USBFS_UIE_SUSPEND | WCH_USBFS_UIE_BUS_RST | WCH_USBFS_UIE_TRANSFER;
	
	/* Enable Device, DMA, and Interrupts */
	usb->BASE_CTRL = WCH_USBFS_UC_DEV_PU_EN | WCH_USBFS_UC_INT_BUSY | WCH_USBFS_UC_DMA_EN;

	/* Setup EP0 DMA and default State */
	struct wch_usbfs_data *priv = udc_get_private(dev);
	wch_usbfs_set_dma(usb, 0, (uint32_t)priv->ep0_dma_buf);
	
	usb->UEP0_TX_LEN = 0;
	usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_RES_NAK;
	usb->UEP0_RX_CTRL = WCH_USBFS_UEP_R_RES_ACK;

	/* Start worker thread */
	k_thread_create(&priv->thread_data, priv->thread_stack,
			K_KERNEL_STACK_SIZEOF(priv->thread_stack),
			wch_usbfs_thread_handler,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
    
    printk("UDC: Thread created\n");

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
	struct wch_usbfs_data *priv = udc_get_private(dev);
	const struct wch_usbfs_config *cfg = dev->config;
	int i;

	k_mutex_init(&data->mutex);
	k_msgq_init(&priv->msgq, priv->msgq_buf, sizeof(struct usbfs_wch_msg), 8);
	priv->pending_address = 0;

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

#define ISR_LOG(e, v) do { struct usbfs_wch_msg m = {.type=USBFS_WCH_DEBUG, .ep=(e), .debug_val=(v)}; k_msgq_put(&priv->msgq, &m, K_NO_WAIT); } while(0)
static void wch_usbfs_isr_transfer(const struct device *dev)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	struct wch_usbfs_data *priv = udc_get_private(dev);
	uint8_t intst = usb->INT_ST;
	uint8_t ep_idx = intst & WCH_USBFS_UIS_ENDP_MASK;
	uint8_t token = intst & WCH_USBFS_UIS_TOKEN_MASK;
	struct usbfs_wch_msg msg;

	/* ISR_LOG(ep_idx, 0x15000000 | token); */
	ISR_LOG(ep_idx, 0x15000000 | token);

	msg.ep = ep_idx;
	msg.rx_count = 0;

	/* Logic adapted from WCH EXAM USBFS_IRQHandler */
	switch (token) {
	case WCH_USBFS_UIS_TOKEN_IN:
		/* Toggle TX toggle bit and set NAK */
		if (ep_idx == 0) {
			/* EP0 IN completed */
			/* If pending address from SETUP, acknowledge it now in HW */
			if (priv->address_change_pending) {
				ISR_LOG(0, 0xAD000000 | priv->pending_address);
				usb->DEV_ADDR = (priv->pending_address & 0x7F);
				priv->address_change_pending = false;
				priv->pending_address = 0;
			}
			
				/* If ISR was handling a GET_DESCRIPTOR, prepare for status OUT */
				if (priv->isr_handling) {
					if (priv->isr_setup_req_len == 0) {
						/* All data sent, prepare for status OUT */
						usb->UEP0_RX_CTRL = WCH_USBFS_UEP_R_TOG | WCH_USBFS_UEP_R_RES_ACK;
						priv->isr_handling = 0;
					} else {
						/* More data to send (multi-packet) */
						uint8_t mlen = (priv->isr_setup_req_len > 64) ? 64 : priv->isr_setup_req_len;
						memcpy(priv->ep0_dma_buf, priv->isr_desc_ptr, mlen);
						priv->isr_desc_ptr += mlen;
						priv->isr_setup_req_len -= mlen;
						usb->UEP0_TX_LEN = mlen;
						/* Toggle and ACK in one write */
						uint16_t ctrl = wch_usbfs_get_ctrl(usb, 0);
						ctrl ^= WCH_USBFS_UEP_T_TOG;
						ctrl = (ctrl & ~WCH_USBFS_UEP_T_RES_MASK) | WCH_USBFS_UEP_T_RES_ACK;
						wch_usbfs_set_ctrl(usb, 0, ctrl);
					}
					break; /* Don't queue to thread */
				}
			}
		
		/* Toggle TOG and set NAK for next transfer */
		{
			uint16_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
			ctrl ^= WCH_USBFS_UEP_T_TOG;
			ctrl = (ctrl & ~WCH_USBFS_UEP_T_RES_MASK) | WCH_USBFS_UEP_T_RES_NAK;
			wch_usbfs_set_ctrl(usb, ep_idx, ctrl);
		}

		msg.type = USBFS_WCH_IN;
		k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
		break;

	case WCH_USBFS_UIS_TOKEN_OUT:
		/* Toggle RX toggle bit */
		{
			uint16_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
			ctrl ^= (WCH_USBFS_UEP_R_TOG << 8);
			/* Set NAK until buffer is available again */
			ctrl = (ctrl & ~(WCH_USBFS_UEP_R_RES_MASK << 8)) | (WCH_USBFS_UEP_R_RES_NAK << 8);
			wch_usbfs_set_ctrl(usb, ep_idx, ctrl);
		}

		msg.type = USBFS_WCH_OUT;
		msg.rx_count = usb->RX_LEN;
		k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
		break;

	case WCH_USBFS_UIS_TOKEN_SETUP:
		/* SETUP is always EP0 - Handle directly in ISR like WCH EXAM */
		/* Reset EP0 state to DATA1 for next Data stage */
		usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_TOG | WCH_USBFS_UEP_T_RES_NAK;
		usb->UEP0_RX_CTRL = WCH_USBFS_UEP_R_TOG | WCH_USBFS_UEP_R_RES_NAK;

		/* Handle standard requests directly in ISR for faster response */
		{
			struct usb_setup_packet *setup = (struct usb_setup_packet *)priv->ep0_dma_buf;
			ISR_LOG(0, 0x5E700000 | (setup->bRequest << 8) | setup->bmRequestType);
			uint8_t req_type = setup->bmRequestType;
			uint8_t req = setup->bRequest;
			uint16_t wValue = setup->wValue;
			uint16_t wLength = setup->wLength;
			uint8_t len = 0;
			uint8_t errflag = 0;
			bool handled = false;

			if (USB_REQTYPE_GET_TYPE(req_type) == USB_REQTYPE_TYPE_STANDARD) {
				switch (req) {
				case USB_SREQ_GET_DESCRIPTOR:
					{
						uint8_t desc_type = (wValue >> 8) & 0xFF;
						const uint8_t *desc_ptr = NULL;
						uint16_t desc_len = 0;
						
						if (desc_type == USB_DESC_DEVICE) {
							desc_ptr = wch_fs_dev_desc;
							desc_len = sizeof(wch_fs_dev_desc);
						} else if (desc_type == USB_DESC_CONFIGURATION) {
							desc_ptr = wch_fs_cfg_desc;
							desc_len = sizeof(wch_fs_cfg_desc);
						} else {
							errflag = 1;
						}
						
						if (!errflag && desc_ptr) {
							/* Limit to requested length */
							if (desc_len > wLength) desc_len = wLength;
							
							/* First packet */
							len = (desc_len > 64) ? 64 : desc_len;
							memcpy(priv->ep0_dma_buf, desc_ptr, len);
							
							/* Track remaining for multi-packet */
							priv->isr_desc_ptr = desc_ptr + len;
							priv->isr_setup_req_len = desc_len - len;
						}
						handled = true;
					}
					break;
					
				case USB_SREQ_SET_ADDRESS:
					priv->pending_address = wValue & 0x7F;
					priv->address_change_pending = true;
					len = 0; /* Zero-length status */
					handled = true;
					break;

				case USB_SREQ_SET_CONFIGURATION:
				case USB_SREQ_SET_INTERFACE:
				case USB_SREQ_CLEAR_FEATURE:
				case USB_SREQ_SET_FEATURE:
					/* Handle these in ISR to meet timing, but ALSO queue to thread */
					msg.type = USBFS_WCH_SETUP;
					k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
					len = 0;
					handled = true;
					break;
					
				case USB_SREQ_GET_STATUS:
				case USB_SREQ_GET_CONFIGURATION:
				case USB_SREQ_GET_INTERFACE:
					/* These have data stages, let thread handle for now or add ISR cases if needed */
					msg.type = USBFS_WCH_SETUP;
					k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
					handled = false;
					break;
					
				default:
					/* Unknown standard request - queue for thread handling */
					msg.type = USBFS_WCH_SETUP;
					k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
					handled = false; 
					break;
				}
			} else {
				/* Non-standard requests go to thread */
				msg.type = USBFS_WCH_SETUP;
				k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
			}

			/* Response: ACK or STALL if handled by ISR */
			if (handled) {
				if (errflag) {
					usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_TOG | WCH_USBFS_UEP_T_RES_STALL;
					usb->UEP0_RX_CTRL = WCH_USBFS_UEP_R_TOG | WCH_USBFS_UEP_R_RES_STALL;
					priv->isr_handling = 0;
				} else if (req_type & USB_EP_DIR_IN) {
					/* IN request with data stage (e.g. GET_DESCRIPTOR) */
					usb->UEP0_TX_LEN = len;
					usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_TOG | WCH_USBFS_UEP_T_RES_ACK;
					priv->isr_handling = 1;
				} else {
					/* OUT/No-Data request (SET_ADDRESS, SET_CONFIG etc): Status Stage IN */
					usb->UEP0_TX_LEN = 0;
					usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_TOG | WCH_USBFS_UEP_T_RES_ACK;
					priv->isr_handling = 0;
				}
			}
		}
		break;

	case WCH_USBFS_UIS_TOKEN_SOF:
		/* Ignore SOF for now */
		break;
	}

	usb->INT_FG = WCH_USBFS_UIF_TRANSFER;
}

static int handle_setup(const struct device *dev)
{
	struct wch_usbfs_data *priv = udc_get_private(dev);
	const struct wch_usbfs_config *cfg = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)cfg->base;
	struct net_buf *buf;

	struct usb_setup_packet *setup = (struct usb_setup_packet *)priv->ep0_dma_buf;
	
	/* EXAM: Handle SET_ADDRESS specially? */
	if (setup->bRequest == USB_SREQ_SET_ADDRESS && 
	    setup->bmRequestType == USB_REQTYPE_DIR_TO_DEVICE) {
		priv->pending_address = setup->wValue & 0x7F;
		priv->address_change_pending = true;
		
		/* Status stage IN */
		usb->UEP0_TX_LEN = 0;
		usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_TOG | WCH_USBFS_UEP_T_RES_ACK;
		return 0;
	}

	/* Drop any pending EP0 buffers */
	buf = udc_buf_get_all(udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
	if (buf) {
		net_buf_unref(buf);
	}
	buf = udc_buf_get_all(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
	if (buf) {
		net_buf_unref(buf);
	}

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);
	if (buf) {
		udc_ep_buf_set_setup(buf);
		net_buf_add_mem(buf, priv->ep0_dma_buf, 8);
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_data_in(dev)) {
			udc_ctrl_submit_s_in_status(dev);
		} else if (udc_ctrl_stage_is_data_out(dev)) {
			/* Prepare for data stage */
		} else {
			udc_ctrl_submit_s_status(dev);
		}
	}
	return 0;
}

static void handle_transfer_in(const struct device *dev, uint8_t ep_idx)
{
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;

	ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | ep_idx);
	buf = udc_buf_peek(ep_cfg);
	if (buf) {
		if (ep_idx == 0) {
			if (udc_ctrl_stage_is_status_in(dev) ||
			    udc_ctrl_stage_is_no_data(dev)) {
				udc_ctrl_submit_status(dev, buf);
			}
			udc_ctrl_update_stage(dev, buf);
			buf = udc_buf_get(ep_cfg);
			udc_ep_set_busy(ep_cfg, false);
			
			if (udc_ctrl_stage_is_status_out(dev)) {
				net_buf_unref(buf);
			}

            /* pending_address logic moved to ISR */
		} else {
			buf = udc_buf_get(ep_cfg);
			udc_ep_set_busy(ep_cfg, false);
			udc_submit_ep_event(dev, buf, 0);
		}
	}
}

static void handle_transfer_out(const struct device *dev, uint8_t ep_idx, uint16_t rx_len)
{
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;

	ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | ep_idx);
	buf = udc_buf_peek(ep_cfg);
	if (buf) {
		net_buf_add(buf, rx_len);
		if (ep_idx == 0) {
			if (udc_ctrl_stage_is_status_out(dev)) {
				udc_ctrl_update_stage(dev, buf);
				udc_ctrl_submit_status(dev, buf);
			} else {
				udc_ctrl_update_stage(dev, buf);
				udc_ctrl_submit_s_out_status(dev, buf);
			}
			udc_buf_get(ep_cfg);
			udc_ep_set_busy(ep_cfg, false);
		} else {
			udc_buf_get(ep_cfg);
			udc_ep_set_busy(ep_cfg, false);
			udc_submit_ep_event(dev, buf, 0);
		}
	}
}

static void wch_usbfs_thread_handler(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct wch_usbfs_data *priv = udc_get_private(dev);
	struct usbfs_wch_msg msg;

	while (1) {
		k_msgq_get(&priv->msgq, &msg, K_FOREVER);

		switch (msg.type) {
		case USBFS_WCH_SETUP:
			handle_setup(dev);
			break;
		case USBFS_WCH_IN:
			handle_transfer_in(dev, msg.ep);
			break;
		case USBFS_WCH_OUT:
			handle_transfer_out(dev, msg.ep, msg.rx_count);
			break;
		case USBFS_WCH_SOF:
			udc_submit_sof_event(dev);
			break;
		case USBFS_WCH_DEBUG:
			printk("ISR_LOG[%u] val %08x\n", msg.ep, msg.debug_val);
			break;
		default:
			break;
		}
	}
}

static void wch_usbfs_isr(const struct device *dev)
{
	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t intflag = usb->INT_FG;
	
	/* DEBUG: Verify ISR is being called */
	/* LOG_INF("ISR: intflag=0x%02x", intflag); */
	
	if (intflag & WCH_USBFS_UIF_TRANSFER) {
		wch_usbfs_isr_transfer(dev);
	} else if (intflag & WCH_USBFS_UIF_BUS_RST) {
		usb->INT_FG = WCH_USBFS_UIF_BUS_RST;
		
		usb->DEV_ADDR = 0;
		struct wch_usbfs_data *priv = udc_get_private(dev);
		priv->pending_address = 0;
		priv->address_change_pending = false;
		
		/* Re-init EP0 DMA and Ctrl state: Start with DATA0 (TOG=0) */
		usb->UEP0_DMA = (uint32_t)priv->ep0_dma_buf;
		usb->UEP0_TX_LEN = 0;
		usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_RES_NAK;
		usb->UEP0_RX_CTRL = WCH_USBFS_UEP_R_RES_ACK;

		udc_submit_event(dev, UDC_EVT_RESET, 0);
	} else if (intflag & WCH_USBFS_UIF_SUSPEND) {
		usb->INT_FG = WCH_USBFS_UIF_SUSPEND;
		udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
	} else if (intflag & WCH_USBFS_UIF_SOF) {
		usb->INT_FG = WCH_USBFS_UIF_SOF;
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
		.clock_enable_func = wch_usbfs_clock_on,		\
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
