/*
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CH32X035_USBPD_H
#define __CH32X035_USBPD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <soc.h> /* Pulls in HAL definitions (USBPD_DETAILED_TypeDef, USBPD macro) */

/* Register Bit Definitions */
/* Note: These align with the USBPD_DETAILED_TypeDef provided by the HAL */

/* CONFIG */
#define PD_FILT_ED          (1<<0)             /* PD pin input filter enable */
#define PD_ALL_CLR          (1<<1)             /* Clear all interrupt flags */
#define CC_SEL              (1<<2)             /* Select PD communication port */
#define PD_DMA_EN           (1<<3)             /* Enable DMA for USBPD */
#define PD_RST_EN           (1<<4)             /* PD mode reset command enable */
#define WAKE_POLAR          (1<<5)             /* PD port wake-up level */
#define IE_PD_IO            (1<<10)            /* PD IO interrupt enable */
#define IE_RX_BIT           (1<<11)            /* Receive bit interrupt enable */
#define IE_RX_BYTE          (1<<12)            /* Receive byte interrupt enable */
#define IE_RX_ACT           (1<<13)            /* Receive completion interrupt enable */
#define IE_RX_RESET         (1<<14)            /* Reset interrupt enable */
#define IE_TX_END           (1<<15)            /* Transfer completion interrupt enable */

/* CONTROL */
#define PD_TX_EN            (1<<0)             /* USBPD transceiver mode and transmit enable */
#define BMC_START           (1<<1)             /* BMC send start signal */
#define RX_STATE_0          (1<<2)             /* PD received state bit 0 */
#define RX_STATE_1          (1<<3)             /* PD received state bit 1 */
#define RX_STATE_2          (1<<4)             /* PD received state bit 2 */
#define DATA_FLAG           (1<<5)             /* Cache data valid flag bit */
#define TX_BIT_BACK         (1<<6)             /* Indicates the current bit status of the BMC when sending the code */
#define BMC_BYTE_HI         (1<<7)             /* Indicates the current half-byte status of the PD data being sent and received */

/* TX_SEL */
#define TX_SEL1             (0<<0)
#define TX_SEL1_SYNC1       (0<<0)             /* 0-SYNC1 */
#define TX_SEL1_RST1        (1<<0)             /* 1-RST1 */
#define TX_SEL2_Mask        (3<<2)
#define TX_SEL2_SYNC1       (0<<2)             /* 00-SYNC1 */
#define TX_SEL2_SYNC3       (1<<2)             /* 01-SYNC3 */
#define TX_SEL2_RST1        (2<<2)             /* 1x-RST1 */
#define TX_SEL3_Mask        (3<<4)
#define TX_SEL3_SYNC1       (0<<4)             /* 00-SYNC1 */
#define TX_SEL3_SYNC3       (1<<4)             /* 01-SYNC3 */
#define TX_SEL3_RST1        (2<<4)             /* 1x-RST1 */
#define TX_SEL4_Mask        (3<<6)
#define TX_SEL4_SYNC2       (0<<6)             /* 00-SYNC2 */
#define TX_SEL4_SYNC3       (1<<6)             /* 01-SYNC3 */
#define TX_SEL4_RST2        (2<<6)             /* 1x-RST2 */

/* STATUS */
#define BMC_AUX_Mask        (3<<0)              /* Clear BMC auxiliary information */
#define BMC_AUX_INVALID     (0<<0)              /* 00-Invalid */
#define BMC_AUX_SOP0        (1<<0)              /* 01-SOP0 */
#define BMC_AUX_SOP1_HRST   (2<<0)              /* 10-SOP1 hard reset */
#define BMC_AUX_SOP2_CRST   (3<<0)              /* 11-SOP2 cable reset */
#define BUF_ERR             (1<<2)              /* BUFFER or DMA error interrupt flag */
#define IF_RX_BIT           (1<<3)              /* Receive bit or 5bit interrupt flag */
#define IF_RX_BYTE          (1<<4)              /* Receive byte or SOP interrupt flag */
#define IF_RX_ACT           (1<<5)              /* Receive completion interrupt flag */
#define IF_RX_RESET         (1<<6)              /* Receive reset interrupt flag */
#define IF_TX_END           (1<<7)              /* Transfer completion interrupt flag */

/* PORT_CC1 / PORT_CC2 */
#define PA_CC_AI            (1<<0)               /* CC port comparator analogue input */
#define CC_PD               (1<<1)               /* CC port pull-down resistor enable */
#define CC_PU_Mask          (3<<2)               /* Clear CC port pull-up current */
#define CC_NO_PU            (0<<2)               /* 00-Prohibit pull-up current */
#define CC_PU_330           (1<<2)               /* 01-330uA */
#define CC_PU_180           (2<<2)               /* 10-180uA */
#define CC_PU_80            (3<<2)               /* 11-80uA */
#define CC_LVE              (1<<4)               /* CC port output low voltage enable */
#define CC_CMP_Mask         (7<<5)               /* Clear CC_CMP*/
#define CC_NO_CMP           (0<<5)               /* 000-closed */
#define CC_CMP_22           (2<<5)               /* 010-0.22V */
#define CC_CMP_45           (3<<5)               /* 011-0.45V */
#define CC_CMP_55           (4<<5)               /* 100-0.55V */
#define CC_CMP_66           (5<<5)               /* 101-0.66V */
#define CC_CMP_95           (6<<5)               /* 110-0.95V */
#define CC_CMP_123          (7<<5)               /* 111-1.23V */
#define USBPD_PHY_V33       (1<<8)               /* PD PHY pull-up limit config */
#define USBPD_IN_HVT        (1<<9)               /* PD pin high threshold input */

#ifdef __cplusplus
}
#endif




#endif /* __CH32X035_USBPD_H */

