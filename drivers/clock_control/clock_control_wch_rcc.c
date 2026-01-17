/*
 * Copyright (c) 2024 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_rcc

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/printk.h>

#include <hal_ch32fun.h>

#define WCH_RCC_CLOCK_ID_OFFSET(id) (((id) >> 5) & 0xFF)
#define WCH_RCC_CLOCK_ID_BIT(id)    ((id) & 0x1F)
#define WCH_RCC_PLLMUL_VAL(mul)     (((mul) << 0x12) & RCC_PLLMULL)
#define WCH_RCC_SYSCLK              DT_PROP(DT_NODELABEL(cpu0), clock_frequency)

#if DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32v00x_pll_clock) ||                          \
	DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32v20x_30x_pll_clock)
#define WCH_RCC_SRC_IS_PLL 1
#if DT_NODE_HAS_COMPAT(DT_CLOCKS_CTLR(DT_INST_CLOCKS_CTLR(0)), wch_ch32v00x_hse_clock)
#define WCH_RCC_PLL_SRC_IS_HSE 1
#elif DT_NODE_HAS_COMPAT(DT_CLOCKS_CTLR(DT_INST_CLOCKS_CTLR(0)), wch_ch32v00x_hsi_clock)
#define WCH_RCC_PLL_SRC_IS_HSI 1
#endif
#elif DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32v00x_hse_clock)
#define WCH_RCC_SRC_IS_HSE 1
#elif DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32v00x_hsi_clock)
#define WCH_RCC_SRC_IS_HSI 1
#endif

#if defined(CONFIG_DT_HAS_WCH_CH32V20X_30X_PLL_CLOCK_ENABLED)
#if defined(CONFIG_SOC_CH32V307)
/* TODO: Entry 13 is 6.5x (fractional multiple currently unsupported without
 * changes to RCC config datatype)
 */
static const uint8_t pllmul_lut[] = {18, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0, 15, 16};
#else
static const uint8_t pllmul_lut[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18};
#endif
#endif

struct clock_control_wch_rcc_config {
	RCC_TypeDef *regs;
	uint8_t mul;
};

static int clock_control_wch_rcc_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_wch_rcc_config *config = dev->config;
	RCC_TypeDef *regs = config->regs;
	uint8_t id = (uintptr_t)sys;
	uint32_t reg = (uint32_t)(&regs->AHBPCENR + WCH_RCC_CLOCK_ID_OFFSET(id));
	uint32_t val = sys_read32(reg);

	val |= BIT(WCH_RCC_CLOCK_ID_BIT(id));
	sys_write32(val, reg);

	return 0;
}

static int clock_control_wch_rcc_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_wch_rcc_config *config = dev->config;
	RCC_TypeDef *regs = config->regs;
	uint8_t id = (uintptr_t)sys;
	uint32_t reg = (uint32_t)(&regs->AHBPCENR + WCH_RCC_CLOCK_ID_OFFSET(id));
	uint32_t val = sys_read32(reg);

	val &= ~BIT(WCH_RCC_CLOCK_ID_BIT(id));
	sys_write32(val, reg);

	return 0;
}

static int clock_control_wch_rcc_get_rate(const struct device *dev, clock_control_subsys_t sys,
					  uint32_t *rate)
{
	const struct clock_control_wch_rcc_config *config = dev->config;
	RCC_TypeDef *regs = config->regs;
	uint32_t cfgr0 = regs->CFGR0;
	uint32_t sysclk = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
	uint32_t ahbclk = sysclk;

	if ((cfgr0 & RCC_HPRE_3) != 0) {
		/* The range 0b1000 divides by a power of 2, where 0b1000 is /2, 0b1001 is /4, etc.
		 */
		ahbclk /= 2 << ((cfgr0 & (RCC_HPRE_0 | RCC_HPRE_1 | RCC_HPRE_2)) >> 4);
	} else {
		/* The range 0b0nnn divides by n + 1, where 0b0000 is /1, 0b001 is /2, etc. */
		ahbclk /= ((cfgr0 & (RCC_HPRE_0 | RCC_HPRE_1 | RCC_HPRE_2)) >> 4) + 1;
	}

	/* The datasheet says that AHB == APB1 == APB2, but the registers imply that APB1 and APB2
	 * can be divided from the AHB clock. Assume that the clock tree diagram is correct and
	 * always return AHB.
	 */
	*rate = ahbclk;
	return 0;
}

static void clock_control_wch_rcc_setup_flash(void)
{
#if defined(FLASH_ACTLR_LATENCY)
	uint32_t latency;

#if defined(CONFIG_SOC_CH32V003)
	if (WCH_RCC_SYSCLK <= 24000000) {
		latency = FLASH_ACTLR_LATENCY_0;
	} else {
		latency = FLASH_ACTLR_LATENCY_1;
	}
#elif defined(CONFIG_SOC_SERIES_CH32V00X)
	if (WCH_RCC_SYSCLK <= 15000000) {
		latency = FLASH_ACTLR_LATENCY_0;
	} else if (WCH_RCC_SYSCLK <= 24000000) {
		latency = FLASH_ACTLR_LATENCY_1;
	} else {
		latency = FLASH_ACTLR_LATENCY_2;
	}
#elif defined(CONFIG_SOC_CH32L103)
	if (WCH_RCC_SYSCLK <= 24000000) {
		latency = FLASH_ACTLR_LATENCY_0;
	} else if (WCH_RCC_SYSCLK <= 48000000) {
		latency = FLASH_ACTLR_LATENCY_1;
	} else {
		latency = FLASH_ACTLR_LATENCY_2;
	}
#else
#error Unrecognised SOC family
#endif
	FLASH->ACTLR = (FLASH->ACTLR & ~FLASH_ACTLR_LATENCY) | latency;
#endif
}

static const struct clock_control_driver_api clock_control_wch_rcc_api = {
	.on = clock_control_wch_rcc_on,
	.off = clock_control_wch_rcc_off,
	.get_rate = clock_control_wch_rcc_get_rate,
};

static int clock_control_wch_rcc_init(const struct device *dev)
{
	const struct clock_control_wch_rcc_config *config = dev->config;

	clock_control_wch_rcc_setup_flash();

#if defined(CONFIG_SOC_CH32L103)
	/* CH32L103 48MHz Target: HSI/2 (4MHz) * 12 = 48MHz */
    /* No EXTEN bit needed (Default is HSI/2) */
#endif

	clock_control_wch_rcc_setup_flash();

	/* Always enable the LSI. */
	RCC->RSTSCKR |= RCC_LSION;
	while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {
	}

	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V00X_HSI_CLOCK_ENABLED)) {
		RCC->CTLR |= RCC_HSION;
		while ((RCC->CTLR & RCC_HSIRDY) == 0) {
		}
	}

	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V00X_HSE_CLOCK_ENABLED)) {
		RCC->CTLR |= RCC_HSEON;
		while ((RCC->CTLR & RCC_HSERDY) == 0) {
		}
	}

	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V00X_PLL_CLOCK_ENABLED)) {
		if (IS_ENABLED(WCH_RCC_PLL_SRC_IS_HSE)) {
			RCC->CFGR0 |= RCC_PLLSRC;
		} else if (IS_ENABLED(WCH_RCC_PLL_SRC_IS_HSI)) {
			RCC->CFGR0 &= ~RCC_PLLSRC;
		}
		RCC->CTLR |= RCC_PLLON;
		while ((RCC->CTLR & RCC_PLLRDY) == 0) {
		}
	}
	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V20X_30X_PLL_CLOCK_ENABLED)) {
		if (IS_ENABLED(WCH_RCC_PLL_SRC_IS_HSE)) {
			RCC->CFGR0 |= RCC_PLLSRC;
		} else if (IS_ENABLED(WCH_RCC_PLL_SRC_IS_HSI)) {
			RCC->CFGR0 &= ~RCC_PLLSRC;
		}
#if defined(RCC_PLLMULL)
		uint8_t pllmul = 0x0; /* Default Reset Value */

		for (size_t i = 0; i < ARRAY_SIZE(pllmul_lut); i++) {
			if (pllmul_lut[i] == config->mul) {
				pllmul = i;
			}
		}


		RCC->CFGR0 &= ~RCC_PLLMULL;
		RCC->CFGR0 |= WCH_RCC_PLLMUL_VAL(pllmul);
#endif
		RCC->CTLR |= RCC_PLLON;
		while ((RCC->CTLR & RCC_PLLRDY) == 0) {
		}
	}

	/* HCLK = SYSCLK = APB1 */
	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV1;

#if defined(CONFIG_SOC_CH32L103)
	/* CH32L103 96MHz: APB1 (PPRE1) Max 36MHz. 96/4 = 24MHz (Safe) */
	if (WCH_RCC_SYSCLK > 72000000) {
		RCC->CFGR0 = (RCC->CFGR0 & ~RCC_PPRE1) | RCC_PPRE1_DIV4;
	} else if (WCH_RCC_SYSCLK > 36000000) {
        RCC->CFGR0 = (RCC->CFGR0 & ~RCC_PPRE1) | RCC_PPRE1_DIV2;
    }
#endif

	if (IS_ENABLED(WCH_RCC_SRC_IS_HSI)) {
		RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_HSI;
	} else if (IS_ENABLED(WCH_RCC_SRC_IS_HSE)) {
		RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_HSE;
	} else if (IS_ENABLED(WCH_RCC_SRC_IS_PLL)) {
		RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;
	}
	RCC->CTLR |= RCC_CSSON;

	/* Clear the interrupt flags. */
	RCC->INTR = RCC_CSSC | RCC_PLLRDYC | RCC_HSERDYC | RCC_LSIRDYC;

	/* Configure USBPRE based on SystemCoreClock */
	/* CH32L103: USBCLK = 48MHz required */
	RCC->CFGR0 &= ~RCC_CFGR0_USBPRE;
#if defined(CONFIG_SOC_CH32L103)
	/* 
	 * CH32L103 Reference Manual:
     * Bit [23:22] USBPRE[1:0]
     * 00: /1   (for PLL=48MHz)
     * 01: /2   (for PLL=96MHz)
     * 10: /1.5 (for PLL=72MHz)
     * 11: Reserved
     */
	if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 96000000) {
		/* 96MHz -> /2 = 48MHz (Val 1: 01b) -> Set Bit 22, Clear Bit 23 */
		RCC->CFGR0 = (RCC->CFGR0 & ~(1 << 23)) | (1 << 22);
	} else if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 72000000) {
		/* 72MHz -> /1.5 = 48MHz (Val 2: 10b) -> Clear Bit 22, Set Bit 23 */
		RCC->CFGR0 = (RCC->CFGR0 & ~(1 << 22)) | (1 << 23);
	} else if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 48000000) {
		/* 48MHz -> /1 = 48MHz (Val 0: 00b) -> Clear Bit 22, Clear Bit 23 */
		RCC->CFGR0 &= ~(3 << 22);
	}
#else
    /* Standard CH32V103/STM32F103 behavior (Single Bit 22) */
    /* 0: PLL/1.5, 1: PLL/1 */
	if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 48000000) {
        RCC->CFGR0 |= (1 << 22);
    } else {
        RCC->CFGR0 &= ~(1 << 22);
    }
#endif

	return 0;
}

#define CLOCK_CONTROL_WCH_RCC_INIT(idx)                                                            \
	static const struct clock_control_wch_rcc_config clock_control_wch_rcc_##idx##_config = {  \
		.regs = (RCC_TypeDef *)DT_INST_REG_ADDR(idx),                                      \
		.mul = DT_PROP_OR(DT_INST_CLOCKS_CTLR(idx), mul, 1),                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, clock_control_wch_rcc_init, NULL, NULL,                         \
			      &clock_control_wch_rcc_##idx##_config, PRE_KERNEL_1,                 \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_wch_rcc_api);

/* There is only ever one RCC */
CLOCK_CONTROL_WCH_RCC_INIT(0)
