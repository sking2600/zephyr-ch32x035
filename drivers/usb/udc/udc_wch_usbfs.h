/*
 * Copyright (c) 2025 Scott King
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_USBFS_H_
#define ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_USBFS_H_

#include <zephyr/drivers/usb/udc.h>

#if defined(CONFIG_SOC_CH32L103)
#define CH32L103
#elif defined(CONFIG_SOC_CH32X035)
#define CH32X03x
#elif defined(CONFIG_SOC_CH32V307)
#define CH32V30x
#endif

#include <ch32fun.h>

#if defined(CONFIG_SOC_CH32L103)
typedef USBFSD_TypeDef WCH_USBFS_RegDef;
#elif defined(CONFIG_SOC_CH32X035)
typedef USBFS_TypeDef WCH_USBFS_RegDef;
#elif defined(CONFIG_SOC_CH32V307)
typedef USBOTG_FS_TypeDef WCH_USBFS_RegDef;
#endif

/* USBFS Register Access Helpers */
static inline uint8_t wch_usbfs_get_ep_idx(uint8_t ep)
{
	return ep & 0x7F;
}

static inline void wch_usbfs_set_dma(WCH_USBFS_RegDef *usb, uint8_t ep, uint32_t addr)
{
#if defined(CONFIG_SOC_CH32X035)
	switch (ep) {
	case 0: usb->UEP0_DMA = addr; break;
	case 1: usb->UEP1_DMA = addr; break;
	case 2: usb->UEP2_DMA = addr; break;
	case 3: usb->UEP3_DMA = addr; break;
	/* X035: EP4 DMA is 16-bit? or missing in struct but at 0x20. */
	/* WCH manual says EP4 shares DMA with EP0 in some modes? */
	/* For now, let's skip EP4 on X035 if not in struct */
	case 5: usb->UEP5_DMA = addr; break;
	case 6: usb->UEP6_DMA = addr; break;
	case 7: usb->UEP7_DMA = addr; break;
	}
#else
	switch (ep) {
	case 0: usb->UEP0_DMA = addr; break;
	case 1: usb->UEP1_DMA = addr; break;
	case 2: usb->UEP2_DMA = addr; break;
	case 3: usb->UEP3_DMA = addr; break;
	case 4: usb->UEP4_DMA = addr; break;
	case 5: usb->UEP5_DMA = addr; break;
	case 6: usb->UEP6_DMA = addr; break;
	case 7: usb->UEP7_DMA = addr; break;
	}
#endif
}

static inline void wch_usbfs_set_tx_len(WCH_USBFS_RegDef *usb, uint8_t ep, uint16_t len)
{
	switch (ep) {
	case 0: usb->UEP0_TX_LEN = len; break;
	case 1: usb->UEP1_TX_LEN = len; break;
	case 2: usb->UEP2_TX_LEN = len; break;
	case 3: usb->UEP3_TX_LEN = len; break;
	case 4: usb->UEP4_TX_LEN = len; break;
	case 5: usb->UEP5_TX_LEN = len; break;
	case 6: usb->UEP6_TX_LEN = len; break;
	case 7: usb->UEP7_TX_LEN = len; break;
	}
}

static inline uint16_t wch_usbfs_get_ctrl(WCH_USBFS_RegDef *usb, uint8_t ep)
{
#if defined(CONFIG_SOC_CH32X035)
	switch (ep) {
	case 0: return usb->UEP0_CTRL_H;
	case 1: return usb->UEP1_CTRL_H;
	case 2: return usb->UEP2_CTRL_H;
	case 3: return usb->UEP3_CTRL_H;
	case 4: return usb->UEP4_CTRL_H;
	case 5: return usb->UEP5_CTRL_H;
	case 6: return usb->UEP6_CTRL_H;
	case 7: return usb->UEP7_CTRL_H;
	}
#elif defined(CONFIG_SOC_CH32V307)
	switch (ep) {
	case 0: return (usb->UEP0_RX_CTRL << 8) | usb->UEP0_TX_CTRL;
	case 1: return (usb->UEP1_RX_CTRL << 8) | usb->UEP1_TX_CTRL;
	case 2: return (usb->UEP2_RX_CTRL << 8) | usb->UEP2_TX_CTRL;
	case 3: return (usb->UEP3_RX_CTRL << 8) | usb->UEP3_TX_CTRL;
	case 4: return (usb->UEP4_RX_CTRL << 8) | usb->UEP4_TX_CTRL;
	case 5: return (usb->UEP5_RX_CTRL << 8) | usb->UEP5_TX_CTRL;
	case 6: return (usb->UEP6_RX_CTRL << 8) | usb->UEP6_TX_CTRL;
	case 7: return (usb->UEP7_RX_CTRL << 8) | usb->UEP7_TX_CTRL;
	}
#else
	switch (ep) {
	case 0: return usb->UEP0_CTRL;
	case 1: return usb->UEP1_CTRL;
	case 2: return usb->UEP2_CTRL;
	case 3: return usb->UEP3_CTRL;
	case 4: return usb->UEP4_CTRL;
	case 5: return usb->UEP5_CTRL;
	case 6: return usb->UEP6_CTRL;
	case 7: return usb->UEP7_CTRL;
	}
#endif
	return 0;
}

static inline void wch_usbfs_set_ctrl(WCH_USBFS_RegDef *usb, uint8_t ep, uint16_t val)
{
#if defined(CONFIG_SOC_CH32X035)
	switch (ep) {
	case 0: usb->UEP0_CTRL_H = val; break;
	case 1: usb->UEP1_CTRL_H = val; break;
	case 2: usb->UEP2_CTRL_H = val; break;
	case 3: usb->UEP3_CTRL_H = val; break;
	case 4: usb->UEP4_CTRL_H = val; break;
	case 5: usb->UEP5_CTRL_H = val; break;
	case 6: usb->UEP6_CTRL_H = val; break;
	case 7: usb->UEP7_CTRL_H = val; break;
	}
#elif defined(CONFIG_SOC_CH32V307)
	switch (ep) {
	case 0: usb->UEP0_TX_CTRL = val & 0xFF; usb->UEP0_RX_CTRL = (val >> 8) & 0xFF; break;
	case 1: usb->UEP1_TX_CTRL = val & 0xFF; usb->UEP1_RX_CTRL = (val >> 8) & 0xFF; break;
	case 2: usb->UEP2_TX_CTRL = val & 0xFF; usb->UEP2_RX_CTRL = (val >> 8) & 0xFF; break;
	case 3: usb->UEP3_TX_CTRL = val & 0xFF; usb->UEP3_RX_CTRL = (val >> 8) & 0xFF; break;
	case 4: usb->UEP4_TX_CTRL = val & 0xFF; usb->UEP4_RX_CTRL = (val >> 8) & 0xFF; break;
	case 5: usb->UEP5_TX_CTRL = val & 0xFF; usb->UEP5_RX_CTRL = (val >> 8) & 0xFF; break;
	case 6: usb->UEP6_TX_CTRL = val & 0xFF; usb->UEP6_RX_CTRL = (val >> 8) & 0xFF; break;
	case 7: usb->UEP7_TX_CTRL = val & 0xFF; usb->UEP7_RX_CTRL = (val >> 8) & 0xFF; break;
	}
#else
	switch (ep) {
	case 0: usb->UEP0_CTRL = val; break;
	case 1: usb->UEP1_CTRL = val; break;
	case 2: usb->UEP2_CTRL = val; break;
	case 3: usb->UEP3_CTRL = val; break;
	case 4: usb->UEP4_CTRL = val; break;
	case 5: usb->UEP5_CTRL = val; break;
	case 6: usb->UEP6_CTRL = val; break;
	case 7: usb->UEP7_CTRL = val; break;
	}
#endif
}

/* WCH USBFS Bit Definitions (Internal to driver to avoid HAL conflicts) */
#define WCH_USBFS_UC_DEV_PU_EN       0x20
#define WCH_USBFS_UC_INT_BUSY        0x08
#define WCH_USBFS_UC_RESET_SIE       0x04
#define WCH_USBFS_UC_CLR_ALL         0x02
#define WCH_USBFS_UC_DMA_EN          0x01

#define WCH_USBFS_UIE_SUSPEND        0x04
#define WCH_USBFS_UIE_TRANSFER       0x02
#define WCH_USBFS_UIE_BUS_RST        0x01

#define WCH_USBFS_UIF_SUSPEND        0x04
#define WCH_USBFS_UIF_TRANSFER       0x02
#define WCH_USBFS_UIF_BUS_RST        0x01

#define WCH_USBFS_UIS_TOKEN_OUT      0x00
#define WCH_USBFS_UIS_TOKEN_SOF      0x10
#define WCH_USBFS_UIS_TOKEN_IN       0x20
#define WCH_USBFS_UIS_TOKEN_SETUP    0x30
#define WCH_USBFS_UIS_TOKEN_MASK     0x30
#define WCH_USBFS_UIS_ENDP_MASK      0x0F

#define WCH_USBFS_UD_PD_DIS          0x80
#define WCH_USBFS_UD_PORT_EN         0x01

#define WCH_USBFS_UEP_TX_EN          0x40
#define WCH_USBFS_UEP_RX_EN          0x80

/* Response Types (Note: X035 and L103 bits are same for RES_ACK/NAK/STALL) */
#define WCH_USBFS_UEP_T_RES_MASK     0x03
#define WCH_USBFS_UEP_T_RES_ACK      0x00
#define WCH_USBFS_UEP_T_RES_NAK      0x02
#define WCH_USBFS_UEP_T_RES_STALL    0x03
#define WCH_USBFS_UEP_T_TOG          0x04

#define WCH_USBFS_UEP_R_RES_MASK     0x03
#define WCH_USBFS_UEP_R_RES_ACK      0x00
#define WCH_USBFS_UEP_R_RES_NAK      0x02
#define WCH_USBFS_UEP_R_RES_STALL    0x03
#define WCH_USBFS_UEP_R_TOG          0x04

/* SOC Specific MOD Bitmasks */
#define WCH_USBFS_UEP1_TX_EN         0x40
#define WCH_USBFS_UEP1_RX_EN         0x80
#define WCH_USBFS_UEP2_TX_EN         0x04
#define WCH_USBFS_UEP2_RX_EN         0x08
#define WCH_USBFS_UEP3_TX_EN         0x40
#define WCH_USBFS_UEP3_RX_EN         0x80
#define WCH_USBFS_UEP4_TX_EN         0x04
#define WCH_USBFS_UEP4_RX_EN         0x08
#define WCH_USBFS_UEP5_TX_EN         0x04
#define WCH_USBFS_UEP5_RX_EN         0x08
#define WCH_USBFS_UEP6_TX_EN         0x40
#define WCH_USBFS_UEP6_RX_EN         0x80
#define WCH_USBFS_UEP7_TX_EN         0x04
#define WCH_USBFS_UEP7_RX_EN         0x08

static inline void wch_usbfs_ep_set_mod(WCH_USBFS_RegDef *usb, uint8_t ep_idx, bool enable)
{
	uint8_t mask = 0;
	
#if defined(CONFIG_SOC_CH32X035)
	if (ep_idx == 1) mask = WCH_USBFS_UEP1_TX_EN | WCH_USBFS_UEP1_RX_EN;
	else if (ep_idx == 2) mask = WCH_USBFS_UEP2_TX_EN | WCH_USBFS_UEP2_RX_EN;
	else if (ep_idx == 3) mask = WCH_USBFS_UEP3_TX_EN | WCH_USBFS_UEP3_RX_EN;
	else if (ep_idx == 4) mask = WCH_USBFS_UEP4_TX_EN | WCH_USBFS_UEP4_RX_EN;
	else if (ep_idx == 5) mask = WCH_USBFS_UEP5_TX_EN | WCH_USBFS_UEP5_RX_EN;
	else if (ep_idx == 6) mask = WCH_USBFS_UEP6_TX_EN | WCH_USBFS_UEP6_RX_EN;
	else if (ep_idx == 7) mask = WCH_USBFS_UEP7_TX_EN | WCH_USBFS_UEP7_RX_EN;

	if (enable) {
		if (ep_idx == 1 || ep_idx == 4) usb->UEP4_1_MOD |= mask;
		else if (ep_idx == 2 || ep_idx == 3) usb->UEP2_3_MOD |= mask;
		else if (ep_idx == 5 || ep_idx == 6 || ep_idx == 7) usb->UEP567_MOD |= mask;
	} else {
		mask = ~mask;
		if (ep_idx == 1 || ep_idx == 4) usb->UEP4_1_MOD &= mask;
		else if (ep_idx == 2 || ep_idx == 3) usb->UEP2_3_MOD &= mask;
		else if (ep_idx == 5 || ep_idx == 6 || ep_idx == 7) usb->UEP567_MOD &= mask;
	}
#else
	/* L103 */
	if (ep_idx == 1) mask = WCH_USBFS_UEP1_TX_EN | WCH_USBFS_UEP1_RX_EN;
	else if (ep_idx == 2) mask = WCH_USBFS_UEP2_TX_EN | WCH_USBFS_UEP2_RX_EN;
	else if (ep_idx == 3) mask = WCH_USBFS_UEP3_TX_EN | WCH_USBFS_UEP3_RX_EN;
	else if (ep_idx == 4) mask = WCH_USBFS_UEP4_TX_EN | WCH_USBFS_UEP4_RX_EN;
	else if (ep_idx == 5) mask = WCH_USBFS_UEP5_TX_EN | WCH_USBFS_UEP5_RX_EN;
	else if (ep_idx == 6) mask = WCH_USBFS_UEP6_TX_EN | WCH_USBFS_UEP6_RX_EN;
	else if (ep_idx == 7) mask = WCH_USBFS_UEP7_TX_EN | WCH_USBFS_UEP7_RX_EN;
	
	if (enable) {
		if (ep_idx == 1 || ep_idx == 4) usb->UEP4_1_MOD |= mask;
		else if (ep_idx == 2 || ep_idx == 3) usb->UEP2_3_MOD |= mask;
		else if (ep_idx == 5 || ep_idx == 6) usb->UEP5_6_MOD |= mask;
		else if (ep_idx == 7) usb->UEP7_MOD |= mask;
	} else {
		mask = ~mask;
		if (ep_idx == 1 || ep_idx == 4) usb->UEP4_1_MOD &= mask;
		else if (ep_idx == 2 || ep_idx == 3) usb->UEP2_3_MOD &= mask;
		else if (ep_idx == 5 || ep_idx == 6) usb->UEP5_6_MOD &= mask;
		else if (ep_idx == 7) usb->UEP7_MOD &= mask;
	}
#endif
}

#define USBFS_EP_NUM 8

struct wch_usbfs_ep_state {
	uint16_t mps;
	uint8_t type;
};

struct wch_usbfs_data {
	struct udc_data data;
	struct wch_usbfs_ep_state eps[USBFS_EP_NUM * 2]; /* IN and OUT */
	struct k_work work;
	uint8_t setup_buf[8] __aligned(4);
};

struct wch_usbfs_config {
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	uint32_t base;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_enable_func)(const struct device *dev);
	int (*clock_enable_func)(const struct device *dev);
};

#endif /* ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_USBFS_H_ */
