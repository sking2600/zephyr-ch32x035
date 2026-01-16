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
/* Hacky LED debug: PA8 is LED. Direct register access. */
#define GPIOA_BASE 0x40010800
#define GPIOA_CFGHR (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_BSHR (*(volatile uint32_t *)(GPIOA_BASE + 0x10))
#define GPIOA_BCR  (*(volatile uint32_t *)(GPIOA_BASE + 0x14))
#define RCC_APB2PCENR (*(volatile uint32_t *)(0x40021000 + 0x18))
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/ch32l103_clock.h>
#include <zephyr/usb/usb_ch9.h>
#include <soc.h>

LOG_MODULE_REGISTER(udc_wch_usbfs, CONFIG_UDC_DRIVER_LOG_LEVEL);

#ifndef DT_DRV_COMPAT
#define DT_DRV_COMPAT wch_usbfs
#endif

static void wch_usbfs_thread_handler(void *arg1, void *arg2, void *arg3);

static int wch_usbfs_clock_on(const struct device *dev)
{
	const struct device *rcc = DEVICE_DT_GET(DT_NODELABEL(rcc));

	if (!device_is_ready(rcc)) {
		return -ENODEV;
	}

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

	if (ep_idx == 0 && USB_EP_DIR_IS_IN(cfg->addr)) {
		/* Debug: EP0 IN data stage */
		printk("EP0 IN enqueue: len=%u data[0]=0x%02x\n", buf->len, 
		       buf->len > 0 ? buf->data[0] : 0);
	}

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
	printk("DEBUG: wch_usbfs_enable\n");
    /* Flash LED to indicate ENABLE reached */
    GPIOA_BCR = (1 << 8); k_busy_wait(100000); GPIOA_BSHR = (1 << 8);
	const struct wch_usbfs_config *cfg = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)cfg->base;

	printk("USB: Enable start\n");
	/* Enable USBFS interrupts */
	usb->INT_EN = WCH_USBFS_UIE_SUSPEND | WCH_USBFS_UIE_BUS_RST | WCH_USBFS_UIE_TRANSFER;

	/* Enable Device, DMA, and Interrupts at the controller level */
	/* Match EXAM: PU_EN | INT_BUSY | DMA_EN (0x29). No SYS_CTRL3. */
	usb->BASE_CTRL = WCH_USBFS_UC_DEV_PU_EN | WCH_USBFS_UC_INT_BUSY | 
			 WCH_USBFS_UC_DMA_EN;

// ... (Moving to Init/Reset sections in separate chunks if needed, or combining if close)
// Wait, I can't combine non-contiguous. I'll do Enable first.

	/* Enable Pull-up to signal connection */
	usb->UDEV_CTRL = WCH_USBFS_UD_PD_DIS | WCH_USBFS_UD_PORT_EN;

	cfg->irq_enable_func(dev);
	printk("USB: Enable done\n");

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
	printk("DEBUG: wch_usbfs_init\n");
    /* Enable GPIOA and set PA8 as output push-pull */
    RCC_APB2PCENR |= (1 << 2); /* IOP A EN */
    GPIOA_CFGHR &= ~(0xF << 0); /* PA8 clear */
    GPIOA_CFGHR |= (0x3 << 0);  /* PA8 Output 50MHz Push-Pull */
    GPIOA_BSHR = (1 << 8);      /* PA8 High: Driver Loaded */
    
    /* PB3 Debug LED (User Request) */
    RCC_APB2PCENR |= (1 << 3); /* IOP B EN */
    GPIOB->CFGHR &= ~(0xF << 12); /* PB3 clear */
    GPIOB->CFGHR |= (0x3 << 12);  /* PB3 Output 50MHz Push-Pull */
    GPIOB->BSHR = (1 << 3);       /* PB3 High */
    
    /* PB3 Debug LED (User Request) */
    RCC_APB2PCENR |= (1 << 3); /* IOP B EN */
    GPIOB->CFGHR &= ~(0xF << 12); /* PB3 clear */
    GPIOB->CFGHR |= (0x3 << 12);  /* PB3 Output 50MHz Push-Pull */
    GPIOB->BSHR = (1 << 3);       /* PB3 High */

	const struct wch_usbfs_config *cfg = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)cfg->base;
	int ret;

	LOG_DBG("init");
    printk("WCH USBFS API Init\n");

	if (cfg->clock_enable_func) {
		ret = cfg->clock_enable_func(dev);
		if (ret < 0) {
            printk("USB: Clock enable failed: %d\n", ret);
			return ret;
		}
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
        printk("USB: Pinctrl apply failed: %d\n", ret);
		return ret;
	}
    printk("USB: Init sequence start\n");

	/* Reset the USBFS peripheral */
	usb->BASE_CTRL = WCH_USBFS_UC_RESET_SIE | WCH_USBFS_UC_CLR_ALL;
	k_busy_wait(10);
	usb->BASE_CTRL = 0x00; /* This clears DEV_PU_EN (Disconnect) */
	
	/* Force Disconnect Delay to ensure Host sees removal */
	k_msleep(500);
	
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
	wch_usbfs_set_dma(usb, 0, (uint32_t)priv->ep0_dma_buf);
	
	/* Initialize Control EP0 */
	usb->UEP0_TX_LEN = 0;
	/* Set EP0 to accept SETUP packets (RX=ACK, TX=NAK) */
	/* Set EP0 to accept SETUP packets (RX=ACK, TX=NAK). */
	usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_RES_NAK;
	usb->UEP0_RX_CTRL = WCH_USBFS_UEP_R_RES_ACK;

	/* Start worker thread */
	k_thread_create(&priv->thread_data, priv->thread_stack,
			K_KERNEL_STACK_SIZEOF(priv->thread_stack),
			wch_usbfs_thread_handler,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);

	printk("USB: Init sequence done\n");

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

#if defined(CONFIG_SOC_CH32L103)
	/* DEBUG: 3 SLOW blinks on PA8/PB3 at USB driver init entry */
	{
		volatile uint32_t d;
		int b;
        RCC_APB2PCENR |= (1 << 2) | (1 << 3); /* Enable IOPA, IOPB */
        GPIOA_CFGHR &= ~(0xF << 0); GPIOA_CFGHR |= (0x3 << 0); /* PA8 Output */
        GPIOB->CFGHR &= ~(0xF << 12); GPIOB->CFGHR |= (0x3 << 12); /* PB3 Output */
        
		for (b = 0; b < 3; b++) {
			GPIOA_BSHR = (1 << 8);  /* PA8 ON */
            GPIOB->BSHR = (1 << 3); /* PB3 ON */
			for (d = 0; d < 2000000; d++) { __asm__ volatile("nop"); }
			GPIOA_BCR = (1 << 8);   /* PA8 OFF */
            GPIOB->BCR = (1 << 3);  /* PB3 OFF */
			for (d = 0; d < 2000000; d++) { __asm__ volatile("nop"); }
		}
	}
#endif

    printk("WCH USBFS Driver Init (POST_KERNEL)\n");

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

    printk("WCH USBFS Driver Init Done\n");

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
	struct usbfs_wch_msg msg;

	msg.ep = ep_idx;
	msg.rx_count = 0;

	switch (token) {
	case WCH_USBFS_UIS_TOKEN_SETUP:
		if (ep_idx == 0) {
			struct usb_setup_packet *setup = (struct usb_setup_packet *)priv->ep0_dma_buf;
			/* Log only critical SETUP packets to avoid slowing down ISR */
			if (setup->bRequest != USB_SREQ_SET_ADDRESS && setup->bRequest != USB_SREQ_GET_DESCRIPTOR) {
				printk("ISR: SETUP req=0x%02x val=0x%04x\n", setup->bRequest, setup->wValue);
			}

			/* Handle SET_ADDRESS synchronously in ISR for timing */
			if (setup->bRequest == USB_SREQ_SET_ADDRESS && 
			    setup->bmRequestType == USB_REQTYPE_DIR_TO_DEVICE) {
				priv->pending_address = setup->wValue & 0x7F;
				
				/* Immediately prepare 0-length IN response for Status Stage */
				/* Status stage is ALWAYS DATA1 */
				usb->UEP0_TX_LEN = 0;
				wch_usbfs_set_ctrl(usb, 0, WCH_USBFS_UEP_T_TOG | WCH_USBFS_UEP_T_RES_ACK);
				return;
			}
			
			/* Handle initial GET_DESCRIPTOR (Device) synchronously to avoid host timeout */
			if (setup->bRequest == USB_SREQ_GET_DESCRIPTOR && setup->wValue == 0x0100) {
				printk("ISR: Sync GET_DESC(Device)\n");
				/* For now, just ensuring the toggles are reset to DATA1 and we ACK.
				 * The thread will soon call ep_enqueue to provide data. */
				wch_usbfs_set_ctrl(usb, 0, WCH_USBFS_UEP_T_TOG | WCH_USBFS_UEP_T_RES_NAK |
							 ((WCH_USBFS_UEP_R_TOG | WCH_USBFS_UEP_R_RES_ACK) << 8));
			} else {
				/* For other requests, next stage is always DATA1 */
				/* Reset both to DATA1, but keep NAK until thread prepares data */
				wch_usbfs_set_ctrl(usb, 0, WCH_USBFS_UEP_T_TOG | WCH_USBFS_UEP_T_RES_NAK |
							 ((WCH_USBFS_UEP_R_TOG | WCH_USBFS_UEP_R_RES_NAK) << 8));
			}
			
			/* Still notify the thread for proper UDC processing state machine */
			msg.type = USBFS_WCH_SETUP;
			k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
		}
		break;
	case WCH_USBFS_UIS_TOKEN_IN:
		/* Toggle TX bit for next transfer */
		{
			uint16_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
			ctrl ^= WCH_USBFS_UEP_T_TOG;
			/* Set to NAK until next ep_enqueue */
			ctrl = (ctrl & ~WCH_USBFS_UEP_T_RES_MASK) | WCH_USBFS_UEP_T_RES_NAK;
			wch_usbfs_set_ctrl(usb, ep_idx, ctrl);
		}

		msg.type = USBFS_WCH_IN;
		k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
		break;
	case WCH_USBFS_UIS_TOKEN_OUT:
		/* Toggle RX bit for next transfer */
		{
			uint16_t ctrl = wch_usbfs_get_ctrl(usb, ep_idx);
			ctrl ^= (WCH_USBFS_UEP_R_TOG << 8);
			/* Set to NAK until next OUT buffer is ready */
			ctrl = (ctrl & ~(WCH_USBFS_UEP_R_RES_MASK << 8)) | (WCH_USBFS_UEP_R_RES_NAK << 8);
			wch_usbfs_set_ctrl(usb, ep_idx, ctrl);
		}

		msg.type = USBFS_WCH_OUT;
		msg.rx_count = usb->RX_LEN;
		k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
		break;
	case WCH_USBFS_UIS_TOKEN_SOF:
		/* SOF can be ignored for now */
		break;
	default:
		break;
	}

	usb->INT_FG = WCH_USBFS_UIF_TRANSFER;
}

static int handle_setup(const struct device *dev)
{
	printk("DEBUG: handle_setup\n");
	struct wch_usbfs_data *priv = udc_get_private(dev);
	struct net_buf *buf;

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
	struct wch_usbfs_data *priv = udc_get_private(dev);
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
			udc_buf_get(ep_cfg);
			udc_ep_set_busy(ep_cfg, false);
			
			if (udc_ctrl_stage_is_status_out(dev)) {
				net_buf_unref(buf);
			}

			if (priv->pending_address != 0) {
				wch_usbfs_set_address(dev, priv->pending_address);
				priv->pending_address = 0;
			}
		} else {
			udc_buf_get(ep_cfg);
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
		default:
			break;
		}
	}
}

static void wch_usbfs_isr(const struct device *dev)
{
	/* printk("DEBUG: ISR\n"); */
    /* Toggle LED on ISR */
    static int toggle = 0;
    if (toggle) GPIOA_BSHR = (1 << 8); else GPIOA_BCR = (1 << 8);
    toggle = !toggle;

	const struct wch_usbfs_config *config = dev->config;
	WCH_USBFS_RegDef *usb = (WCH_USBFS_RegDef *)config->base;
	uint8_t intflag = usb->INT_FG;
	
	if (intflag & WCH_USBFS_UIF_TRANSFER) {
		wch_usbfs_isr_transfer(dev);
	}
	
	if (intflag & WCH_USBFS_UIF_BUS_RST) {
		/* LOG_DBG("BUS_RST"); */
		printk("ISR: BUS_RST\n");
		usb->INT_FG = WCH_USBFS_UIF_BUS_RST;
		
		/* Bus Reset: Reset Address and Re-init EP0 fully (Like EXAM) */
		usb->DEV_ADDR = 0;
		struct wch_usbfs_data *priv = udc_get_private(dev);
		priv->pending_address = 0;
		
		/* Re-init EP0 DMA */
		usb->UEP0_DMA = (uint32_t)priv->ep0_dma_buf;
		
		/* Re-init EP0 Control */
		usb->UEP0_TX_LEN = 0;
		usb->UEP0_TX_CTRL = WCH_USBFS_UEP_T_RES_NAK;
		usb->UEP0_RX_CTRL = WCH_USBFS_UEP_R_RES_ACK;
		
		/* DIAG: Dump registers to verify state */
		printk("RST: DMA=%08x CTRL=%02x T=%02x R=%02x\n", 
		       usb->UEP0_DMA, usb->BASE_CTRL, 
		       usb->UEP0_TX_CTRL, usb->UEP0_RX_CTRL);

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
