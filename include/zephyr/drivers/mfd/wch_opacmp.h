/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MFD_WCH_OPACMP_H_
#define ZEPHYR_INCLUDE_DRIVERS_MFD_WCH_OPACMP_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <hal_ch32fun.h>

/**
 * @brief Register structure for WCH OPA/CMP block
 *        Based on CH32L103 EVT reference
 */
struct wch_opacmp_regs {
	volatile uint16_t CFGR1;
	volatile uint16_t CFGR2;
	volatile uint32_t CTLR1;
	volatile uint32_t CTLR2;
	volatile uint32_t RESERVED0;
	volatile uint32_t RESERVED1;
	volatile uint32_t OPCMKEY;
};

/* OPCM Key Values */
#define WCH_OPCM_KEY1 0x45670123
#define WCH_OPCM_KEY2 0xCDEF89AB

/* OPA Control Register 1 (CTLR1) Bits */
#define OPA_CTLR1_EN1_MASK      BIT(0)
#define OPA_CTLR1_MODE1_MASK    (BIT(1) | BIT(2) | BIT(3))
#define OPA_CTLR1_PSEL1_MASK    (BIT(4) | BIT(5) | BIT(6))
#define OPA_CTLR1_FBEN1_MASK    BIT(7)
#define OPA_CTLR1_NSEL1_MASK    (BIT(8) | BIT(9) | BIT(10) | BIT(11))
#define OPA_CTLR1_LP1_MASK      BIT(12)

/* Shift values for CTLR1 */
#define OPA_CTLR1_MODE1_SHIFT   1
#define OPA_CTLR1_PSEL1_SHIFT   4
#define OPA_CTLR1_FBEN1_SHIFT   7
#define OPA_CTLR1_NSEL1_SHIFT   8

/* NSEL (Negative Input/Gain) Enumerations */
#define OPA_NSEL_CHN_PGA_1x     0  /* CHN0 */
#define OPA_NSEL_CHN_PGA_2x     5  /* CHN0, Gain=2 (internal R) */
#define OPA_NSEL_CHN_PGA_4x     6  /* CHN0, Gain=4 (internal R) */
#define OPA_NSEL_CHN_PGA_8x     7  /* CHN0, Gain=8 (internal R) */
#define OPA_NSEL_CHN_PGA_16x    8  /* CHN0, Gain=16 (internal R) */
#define OPA_NSEL_CHN_PGA_32x    9  /* CHN0, Gain=32 (internal R) */
#define OPA_NSEL_CHN_PGA_64x    10 /* CHN0, Gain=64 (internal R) */

/* PSEL (Positive Input) Enumerations */
#define OPA_PSEL_CHP0           0  /* PA7 for OPA1? Check datasheet */
#define OPA_PSEL_CHP1           1

/* CMP Control Register 2 (CTLR2) Bits partial definitions per CMP instance */
/* Each CMP occupies 8 bits: [0]=EN, [1:2]=MODE, [3]=NSEL, [4]=PSEL, [5]=HYEN, [6]=LP */
#define CMP_CTLR2_EN_MASK       BIT(0)
#define CMP_CTLR2_MODE_MASK     (BIT(1) | BIT(2))
#define CMP_CTLR2_NSEL_MASK     BIT(3)
#define CMP_CTLR2_PSEL_MASK     BIT(4)
#define CMP_CTLR2_HYEN_MASK     BIT(5)
#define CMP_CTLR2_LP_MASK       BIT(6)
/* Full 8-bit mask for one CMP instance */
#define CMP_CTLR2_ALL_MASK      0x7F

/* Global CTLR2 bits */
#define CMP_CTLR2_WAKEUP_MASK   (BIT(24) | BIT(25))
#define CMP_CTLR2_WAKEUP_SHIFT  24
#define CMP_CTLR2_LOCK          BIT(31)

#endif /* ZEPHYR_INCLUDE_DRIVERS_MFD_WCH_OPACMP_H_ */
