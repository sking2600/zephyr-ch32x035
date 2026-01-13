/*
 * Copyright (c) 2025 Scott King
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_USBHS_H_
#define ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_USBHS_H_

#include <zephyr/drivers/usb/udc.h>

#if defined(CONFIG_SOC_CH32V307)
#define CH32V30x
#endif

#include <ch32fun.h>

typedef USBHSD_TypeDef WCH_USBHS_RegDef;

/* USBHS Register Access Helpers */
static inline uint8_t wch_usbhs_get_ep_idx(uint8_t ep)
{
	return ep & 0x0F;
}

static inline void wch_usbhs_set_rx_dma(WCH_USBHS_RegDef *usb, uint8_t ep_idx, uint32_t addr)
{
	switch (ep_idx) {
	case 0: usb->UEP0_DMA = addr; break;
	case 1: usb->UEP1_RX_DMA = addr; break;
	case 2: usb->UEP2_RX_DMA = addr; break;
	case 3: usb->UEP3_RX_DMA = addr; break;
	case 4: usb->UEP4_RX_DMA = addr; break;
	case 5: usb->UEP5_RX_DMA = addr; break;
	case 6: usb->UEP6_RX_DMA = addr; break;
	case 7: usb->UEP7_RX_DMA = addr; break;
	case 8: usb->UEP8_RX_DMA = addr; break;
	case 9: usb->UEP9_RX_DMA = addr; break;
	case 10: usb->UEP10_RX_DMA = addr; break;
	case 11: usb->UEP11_RX_DMA = addr; break;
	case 12: usb->UEP12_RX_DMA = addr; break;
	case 13: usb->UEP13_RX_DMA = addr; break;
	case 14: usb->UEP14_RX_DMA = addr; break;
	case 15: usb->UEP15_RX_DMA = addr; break;
	}
}

static inline void wch_usbhs_set_tx_dma(WCH_USBHS_RegDef *usb, uint8_t ep_idx, uint32_t addr)
{
	switch (ep_idx) {
	case 0: usb->UEP0_DMA = addr; break;
	case 1: usb->UEP1_TX_DMA = addr; break;
	case 2: usb->UEP2_TX_DMA = addr; break;
	case 3: usb->UEP3_TX_DMA = addr; break;
	case 4: usb->UEP4_TX_DMA = addr; break;
	case 5: usb->UEP5_TX_DMA = addr; break;
	case 6: usb->UEP6_TX_DMA = addr; break;
	case 7: usb->UEP7_TX_DMA = addr; break;
	case 8: usb->UEP8_TX_DMA = addr; break;
	case 9: usb->UEP9_TX_DMA = addr; break;
	case 10: usb->UEP10_TX_DMA = addr; break;
	case 11: usb->UEP11_TX_DMA = addr; break;
	case 12: usb->UEP12_TX_DMA = addr; break;
	case 13: usb->UEP13_TX_DMA = addr; break;
	case 14: usb->UEP14_TX_DMA = addr; break;
	case 15: usb->UEP15_TX_DMA = addr; break;
	}
}

static inline void wch_usbhs_set_tx_len(WCH_USBHS_RegDef *usb, uint8_t ep_idx, uint16_t len)
{
	switch (ep_idx) {
	case 0: usb->UEP0_TX_LEN = len; break;
	case 1: usb->UEP1_TX_LEN = len; break;
	case 2: usb->UEP2_TX_LEN = len; break;
	case 3: usb->UEP3_TX_LEN = len; break;
	case 4: usb->UEP4_TX_LEN = len; break;
	case 5: usb->UEP5_TX_LEN = len; break;
	case 6: usb->UEP6_TX_LEN = len; break;
	case 7: usb->UEP7_TX_LEN = len; break;
	case 8: usb->UEP8_TX_LEN = len; break;
	case 9: usb->UEP9_TX_LEN = len; break;
	case 10: usb->UEP10_TX_LEN = len; break;
	case 11: usb->UEP11_TX_LEN = len; break;
	case 12: usb->UEP12_TX_LEN = len; break;
	case 13: usb->UEP13_TX_LEN = len; break;
	case 14: usb->UEP14_TX_LEN = len; break;
	case 15: usb->UEP15_TX_LEN = len; break;
	}
}

static inline uint8_t wch_usbhs_get_tx_ctrl(WCH_USBHS_RegDef *usb, uint8_t ep_idx)
{
	switch (ep_idx) {
	case 0: return usb->UEP0_TX_CTRL;
	case 1: return usb->UEP1_TX_CTRL;
	case 2: return usb->UEP2_TX_CTRL;
	case 3: return usb->UEP3_TX_CTRL;
	case 4: return usb->UEP4_TX_CTRL;
	case 5: return usb->UEP5_TX_CTRL;
	case 6: return usb->UEP6_TX_CTRL;
	case 7: return usb->UEP7_TX_CTRL;
	case 8: return usb->UEP8_TX_CTRL;
	case 9: return usb->UEP9_TX_CTRL;
	case 10: return usb->UEP10_TX_CTRL;
	case 11: return usb->UEP11_TX_CTRL;
	case 12: return usb->UEP12_TX_CTRL;
	case 13: return usb->UEP13_TX_CTRL;
	case 14: return usb->UEP14_TX_CTRL;
	case 15: return usb->UEP15_TX_CTRL;
	}
	return 0;
}

static inline void wch_usbhs_set_tx_ctrl(WCH_USBHS_RegDef *usb, uint8_t ep_idx, uint8_t val)
{
	switch (ep_idx) {
	case 0: usb->UEP0_TX_CTRL = val; break;
	case 1: usb->UEP1_TX_CTRL = val; break;
	case 2: usb->UEP2_TX_CTRL = val; break;
	case 3: usb->UEP3_TX_CTRL = val; break;
	case 4: usb->UEP4_TX_CTRL = val; break;
	case 5: usb->UEP5_TX_CTRL = val; break;
	case 6: usb->UEP6_TX_CTRL = val; break;
	case 7: usb->UEP7_TX_CTRL = val; break;
	case 8: usb->UEP8_TX_CTRL = val; break;
	case 9: usb->UEP9_TX_CTRL = val; break;
	case 10: usb->UEP10_TX_CTRL = val; break;
	case 11: usb->UEP11_TX_CTRL = val; break;
	case 12: usb->UEP12_TX_CTRL = val; break;
	case 13: usb->UEP13_TX_CTRL = val; break;
	case 14: usb->UEP14_TX_CTRL = val; break;
	case 15: usb->UEP15_TX_CTRL = val; break;
	}
}

static inline uint8_t wch_usbhs_get_rx_ctrl(WCH_USBHS_RegDef *usb, uint8_t ep_idx)
{
	switch (ep_idx) {
	case 0: return usb->UEP0_RX_CTRL;
	case 1: return usb->UEP1_RX_CTRL;
	case 2: return usb->UEP2_RX_CTRL;
	case 3: return usb->UEP3_RX_CTRL;
	case 4: return usb->UEP4_RX_CTRL;
	case 5: return usb->UEP5_RX_CTRL;
	case 6: return usb->UEP6_RX_CTRL;
	case 7: return usb->UEP7_RX_CTRL;
	case 8: return usb->UEP8_RX_CTRL;
	case 9: return usb->UEP9_RX_CTRL;
	case 10: return usb->UEP10_RX_CTRL;
	case 11: return usb->UEP11_RX_CTRL;
	case 12: return usb->UEP12_RX_CTRL;
	case 13: return usb->UEP13_RX_CTRL;
	case 14: return usb->UEP14_RX_CTRL;
	case 15: return usb->UEP15_RX_CTRL;
	}
	return 0;
}

static inline void wch_usbhs_set_rx_ctrl(WCH_USBHS_RegDef *usb, uint8_t ep_idx, uint8_t val)
{
	switch (ep_idx) {
	case 0: usb->UEP0_RX_CTRL = val; break;
	case 1: usb->UEP1_RX_CTRL = val; break;
	case 2: usb->UEP2_RX_CTRL = val; break;
	case 3: usb->UEP3_RX_CTRL = val; break;
	case 4: usb->UEP4_RX_CTRL = val; break;
	case 5: usb->UEP5_RX_CTRL = val; break;
	case 6: usb->UEP6_RX_CTRL = val; break;
	case 7: usb->UEP7_RX_CTRL = val; break;
	case 8: usb->UEP8_RX_CTRL = val; break;
	case 9: usb->UEP9_RX_CTRL = val; break;
	case 10: usb->UEP10_RX_CTRL = val; break;
	case 11: usb->UEP11_RX_CTRL = val; break;
	case 12: usb->UEP12_RX_CTRL = val; break;
	case 13: usb->UEP13_RX_CTRL = val; break;
	case 14: usb->UEP14_RX_CTRL = val; break;
	case 15: usb->UEP15_RX_CTRL = val; break;
	}
}

static inline void wch_usbhs_set_max_len(WCH_USBHS_RegDef *usb, uint8_t ep_idx, uint16_t len)
{
	switch (ep_idx) {
	case 0: usb->UEP0_MAX_LEN = len; break;
	case 1: usb->UEP1_MAX_LEN = len; break;
	case 2: usb->UEP2_MAX_LEN = len; break;
	case 3: usb->UEP3_MAX_LEN = len; break;
	case 4: usb->UEP4_MAX_LEN = len; break;
	case 5: usb->UEP5_MAX_LEN = len; break;
	case 6: usb->UEP6_MAX_LEN = len; break;
	case 7: usb->UEP7_MAX_LEN = len; break;
	case 8: usb->UEP8_MAX_LEN = len; break;
	case 9: usb->UEP9_MAX_LEN = len; break;
	case 10: usb->UEP10_MAX_LEN = len; break;
	case 11: usb->UEP11_MAX_LEN = len; break;
	case 12: usb->UEP12_MAX_LEN = len; break;
	case 13: usb->UEP13_MAX_LEN = len; break;
	case 14: usb->UEP14_MAX_LEN = len; break;
	case 15: usb->UEP15_MAX_LEN = len; break;
	}
}

/* WCH USBHS Bit Definitions */
#define WCH_USBHS_UC_HOST_MODE       0x80
#define WCH_USBHS_UC_SPEED_TYPE      0x60
#define WCH_USBHS_UC_SPEED_LOW       0x40
#define WCH_USBHS_UC_SPEED_FULL      0x00
#define WCH_USBHS_UC_SPEED_HIGH      0x20
#define WCH_USBHS_UC_DEV_PU_EN       0x10
#define WCH_USBHS_UC_INT_BUSY        0x08
#define WCH_USBHS_UC_RESET_SIE       0x04
#define WCH_USBHS_UC_CLR_ALL         0x02
#define WCH_USBHS_UC_DMA_EN          0x01

#define WCH_USBHS_UIE_DEV_NAK        0x80
#define WCH_USBHS_UIE_ISO_ACT        0x40
#define WCH_USBHS_UIE_SETUP_ACT      0x20
#define WCH_USBHS_UIE_FIFO_OV        0x10
#define WCH_USBHS_UIE_SOF_ACT        0x08
#define WCH_USBHS_UIE_SUSPEND        0x04
#define WCH_USBHS_UIE_TRANSFER       0x02
#define WCH_USBHS_UIE_BUS_RST        0x01

#define WCH_USBHS_UIF_ISO_ACT        0x40
#define WCH_USBHS_UIF_SETUP_ACT      0x20
#define WCH_USBHS_UIF_FIFO_OV        0x10
#define WCH_USBHS_UIF_HST_SOF        0x08
#define WCH_USBHS_UIF_SUSPEND        0x04
#define WCH_USBHS_UIF_TRANSFER       0x02
#define WCH_USBHS_UIF_BUS_RST        0x01

#define WCH_USBHS_UIS_IS_NAK         0x80
#define WCH_USBHS_UIS_TOG_OK         0x40
#define WCH_USBHS_UIS_TOKEN_OUT      0x00
#define WCH_USBHS_UIS_TOKEN_SOF      0x10
#define WCH_USBHS_UIS_TOKEN_IN       0x20
#define WCH_USBHS_UIS_TOKEN_SETUP    0x30
#define WCH_USBHS_UIS_TOKEN_MASK     0x30
#define WCH_USBHS_UIS_ENDP_MASK      0x0F

#define WCH_USBHS_UEP_T_RES_MASK     0x03
#define WCH_USBHS_UEP_T_RES_ACK      0x00
#define WCH_USBHS_UEP_T_RES_NAK      0x02
#define WCH_USBHS_UEP_T_RES_STALL    0x03
#define WCH_USBHS_UEP_T_TOG          0x04

#define WCH_USBHS_UEP_R_RES_MASK     0x03
#define WCH_USBHS_UEP_R_RES_ACK      0x00
#define WCH_USBHS_UEP_R_RES_NAK      0x02
#define WCH_USBHS_UEP_R_RES_STALL    0x03
#define WCH_USBHS_UEP_R_TOG          0x04

#define USBHS_EP_NUM 16

struct wch_usbhs_ep_state {
	uint16_t mps;
	uint8_t type;
};

struct wch_usbhs_data {
	struct udc_data data;
	struct wch_usbhs_ep_state eps[USBHS_EP_NUM * 2]; /* IN and OUT */
	struct k_work work;
	uint8_t setup_buf[8] __aligned(4);
};

struct wch_usbhs_config {
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	uint32_t base;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_enable_func)(const struct device *dev);
	const struct device *clock_dev;
	uint32_t clock_id;
};

#endif /* ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_USBHS_H_ */
