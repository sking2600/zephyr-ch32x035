/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CH32X035-specific clock control driver.
 * 
 * The CH32X035 has a simple clock architecture:
 * - Fixed 48MHz internal HSI oscillator (no PLL, no HSE needed)
 * - AHB prescaler to divide HCLK from SYSCLK
 * - APB1 = APB2 = HCLK (all bus clocks are equal)
 */

#define DT_DRV_COMPAT wch_ch32x035_rcc

#include <stdint.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util_macro.h>

/* RCC Register offsets */
#define CH32X035_RCC_CTLR_OFFSET       0x00
#define CH32X035_RCC_CFGR0_OFFSET      0x04
#define CH32X035_RCC_APB2PRSTR_OFFSET  0x0C
#define CH32X035_RCC_APB1PRSTR_OFFSET  0x10
#define CH32X035_RCC_AHBPCENR_OFFSET   0x14
#define CH32X035_RCC_APB2PCENR_OFFSET  0x18
#define CH32X035_RCC_APB1PCENR_OFFSET  0x1C
#define CH32X035_RCC_RSTSCKR_OFFSET    0x24
#define CH32X035_RCC_AHBRSTR_OFFSET    0x28

/* Clock ID encoding from ch32x035_clock.h:
 * (bus_index << 5) | bit_position
 * bus_index: 0=AHB, 1=APB2, 2=APB1
 */
#define CLOCK_ID_BUS(id)  (((id) >> 5) & 0x3)
#define CLOCK_ID_BIT(id)  ((id) & 0x1F)

/* Bus indices matching ch32x035_clock.h */
#define BUS_AHB   0
#define BUS_APB2  1
#define BUS_APB1  2

/* CFGR0 register bits */
#define RCC_HPRE_MASK   0x000000F0
#define RCC_HPRE_SHIFT  4

/* HSI frequency - CH32X035 uses fixed 48MHz internal oscillator */
#define CH32X035_HSI_FREQ  48000000

struct clock_control_ch32x035_config {
	uint32_t base;
};

/* Get the peripheral clock enable register address for a given bus */
static inline uint32_t get_pcenr_reg(uint32_t base, uint8_t bus)
{
	switch (bus) {
	case BUS_AHB:
		return base + CH32X035_RCC_AHBPCENR_OFFSET;
	case BUS_APB2:
		return base + CH32X035_RCC_APB2PCENR_OFFSET;
	case BUS_APB1:
		return base + CH32X035_RCC_APB1PCENR_OFFSET;
	default:
		return 0;
	}
}

static int clock_control_ch32x035_on(const struct device *dev,
				     clock_control_subsys_t sys)
{
	const struct clock_control_ch32x035_config *config = dev->config;
	uint32_t id = (uintptr_t)sys;
	uint8_t bus = CLOCK_ID_BUS(id);
	uint8_t bit = CLOCK_ID_BIT(id);
	uint32_t reg = get_pcenr_reg(config->base, bus);
	
	if (reg == 0) {
		return -EINVAL;
	}

	sys_set_bit(reg, bit);
	return 0;
}

static int clock_control_ch32x035_off(const struct device *dev,
				      clock_control_subsys_t sys)
{
	const struct clock_control_ch32x035_config *config = dev->config;
	uint32_t id = (uintptr_t)sys;
	uint8_t bus = CLOCK_ID_BUS(id);
	uint8_t bit = CLOCK_ID_BIT(id);
	uint32_t reg = get_pcenr_reg(config->base, bus);
	
	if (reg == 0) {
		return -EINVAL;
	}

	sys_clear_bit(reg, bit);
	return 0;
}

static int clock_control_ch32x035_get_rate(const struct device *dev,
					   clock_control_subsys_t sys,
					   uint32_t *rate)
{
	const struct clock_control_ch32x035_config *config = dev->config;
	uint32_t cfgr0 = sys_read32(config->base + CH32X035_RCC_CFGR0_OFFSET);
	uint32_t hpre = (cfgr0 & RCC_HPRE_MASK) >> RCC_HPRE_SHIFT;
	uint32_t hclk = CH32X035_HSI_FREQ;

	/* 
	 * CH32X035 HPRE encoding (from datasheet):
	 * 0000: SYSCLK not divided
	 * 0001: SYSCLK / 2
	 * 0010: SYSCLK / 3
	 * 0011: SYSCLK / 4
	 * 0100: SYSCLK / 5
	 * 0101: SYSCLK / 6  (default after reset)
	 * 0110: SYSCLK / 7
	 * 0111: SYSCLK / 8
	 * 1000: SYSCLK / 2
	 * 1001: SYSCLK / 4
	 * 1010: SYSCLK / 8
	 * 1011: SYSCLK / 16
	 * 1100: SYSCLK / 32
	 * 1101: SYSCLK / 64
	 * 1110: SYSCLK / 128
	 * 1111: SYSCLK / 256
	 */
	if (hpre & 0x8) {
		/* Power of 2 division: 1xxx = div by 2^(xxx+1) */
		hclk >>= ((hpre & 0x7) + 1);
	} else if (hpre > 0) {
		/* Linear division: 0xxx = div by (xxx + 1) */
		hclk /= (hpre + 1);
	}
	/* hpre == 0 means no division */

	/* On CH32X035, APB1 = APB2 = HCLK */
	*rate = hclk;
	return 0;
}

static enum clock_control_status clock_control_ch32x035_get_status(
	const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_ch32x035_config *config = dev->config;
	uint32_t id = (uintptr_t)sys;
	uint8_t bus = CLOCK_ID_BUS(id);
	uint8_t bit = CLOCK_ID_BIT(id);
	uint32_t reg = get_pcenr_reg(config->base, bus);
	
	if (reg == 0) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	if (sys_test_bit(reg, bit)) {
		return CLOCK_CONTROL_STATUS_ON;
	}
	return CLOCK_CONTROL_STATUS_OFF;
}

static DEVICE_API(clock_control, clock_control_ch32x035_api) = {
	.on = clock_control_ch32x035_on,
	.off = clock_control_ch32x035_off,
	.get_rate = clock_control_ch32x035_get_rate,
	.get_status = clock_control_ch32x035_get_status,
};

static int clock_control_ch32x035_init(const struct device *dev)
{
	/* 
	 * CH32X035 comes up with HSI already enabled and stable.
	 * The default HPRE is 0101 (div by 6), giving 8MHz HCLK.
	 * 
	 * For full speed operation, we set HPRE to 0 (no division)
	 * to get 48MHz HCLK.
	 */
	const struct clock_control_ch32x035_config *config = dev->config;
	uint32_t cfgr0 = sys_read32(config->base + CH32X035_RCC_CFGR0_OFFSET);
	
	/* Clear HPRE bits to get SYSCLK = HCLK (no division) */
	cfgr0 &= ~RCC_HPRE_MASK;
	sys_write32(cfgr0, config->base + CH32X035_RCC_CFGR0_OFFSET);

	return 0;
}

#define CLOCK_CONTROL_CH32X035_INIT(idx)                                      \
	static const struct clock_control_ch32x035_config                     \
		clock_control_ch32x035_##idx##_config = {                     \
			.base = DT_INST_REG_ADDR(idx),                        \
		};                                                            \
	DEVICE_DT_INST_DEFINE(idx, clock_control_ch32x035_init, NULL, NULL,   \
			      &clock_control_ch32x035_##idx##_config,         \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, \
			      &clock_control_ch32x035_api);

DT_INST_FOREACH_STATUS_OKAY(CLOCK_CONTROL_CH32X035_INIT)
