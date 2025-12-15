/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CH32X035-specific pinctrl driver.
 *
 * The CH32X035 has an AFIO (Alternate Function I/O) controller that handles
 * pin remapping for peripherals. GPIO configuration is done via CFGxR registers.
 */

#define DT_DRV_COMPAT wch_ch32x035_afio

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <soc.h>

/* GPIO Register offsets */
/* GPIO Register offsets - managed by HAL/SOC definitions where possible, or defined here if missing */
/* Note: ch32x035.h defines GPIO_TypeDef with fields like CFGLR, CFGHR etc. */
/* We can cast base to GPIO_TypeDef* */

#define GPIO_CFGLR_OFFSET   0x00
#define GPIO_CFGHR_OFFSET   0x04
#define GPIO_CFGXR_OFFSET   0x1C
#define GPIO_BSHR_OFFSET    0x10
#define GPIO_CFGHR_OFFSET   0x04  /* Config High (pins 8-15) */
#define GPIO_INDR_OFFSET    0x08  /* Input data */
#define GPIO_OUTDR_OFFSET   0x0C  /* Output data */
#define GPIO_BSHR_OFFSET    0x10  /* Bit set/reset */
#define GPIO_BCR_OFFSET     0x14  /* Bit clear */
#define GPIO_LCKR_OFFSET    0x18  /* Lock */
#define GPIO_CFGXR_OFFSET   0x1C  /* Config Extended (pins 16-23) */
#define GPIO_BSXR_OFFSET    0x20  /* Bit set/reset extended */

/* AFIO Register offsets */
#define AFIO_PCFR1_OFFSET   0x04  /* Port configuration register 1 */
#define AFIO_EXTICR_OFFSET  0x08  /* EXTI configuration */
#define AFIO_CTLR_OFFSET    0x18  /* Control register */

/* Pin configuration encoding (matches ch32x035-pinctrl.h):
 * GPIO_DEFINE(port, pin) = ((port & 0xFF) << 8) | (pin & 0xFF)
 */
#define PINCTRL_PORT(config)  (((config) >> 8) & 0xFF)
#define PINCTRL_PIN(config)   ((config) & 0xFF)

static uint32_t get_gpio_base(uint8_t port)
{
	switch (port) {
	case 0: return (uint32_t)GPIOA;
	case 1: return (uint32_t)GPIOB;
	case 2: return (uint32_t)GPIOC;
	default: return 0;
	}
}

static void enable_gpio_clock(uint8_t port)
{
	switch (port) {
	case 0:
		RCC->APB2PCENR |= RCC_APB2Periph_GPIOA;
		break;
	case 1:
		RCC->APB2PCENR |= RCC_APB2Periph_GPIOB;
		break;
	case 2:
		RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
		break;
	}
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (int i = 0; i < pin_cnt; i++) {
		const pinctrl_soc_pin_t *pin = &pins[i];
		/* config encoding: (port << 8) | pin_num (from ch32x035-pinctrl.h) */
		uint8_t port = PINCTRL_PORT(pin->config);
		uint8_t pin_num = PINCTRL_PIN(pin->config);
		uint32_t gpio_base = get_gpio_base(port);

		if (gpio_base == 0) {
			continue;
		}

		/* Enable GPIO clock */
		enable_gpio_clock(port);

		/* Build configuration value:
		 * Bits 0-1: Mode (00=input, 01=10MHz out, 10=2MHz out, 11=50MHz out)
		 * Bits 2-3: CNF (depends on mode)
		 *   Input:  00=analog, 01=floating, 10=pull-up/down
		 *   Output: 00=push-pull, 01=open-drain, 10=AF push-pull, 11=AF open-drain
		 */
		uint8_t cfg = 0;

		if (pin->output_high || pin->output_low) {
			/* Output mode - use slew_rate for speed */
			cfg = pin->slew_rate + 1;  /* 1=10MHz, 2=2MHz, 3=50MHz */

			if (pin->drive_open_drain) {
				cfg |= 0x04;  /* Open-drain */
			}

			/* Assume alternate function for peripheral pins */
			cfg |= 0x08;  /* Alternate function */
		} else {
			/* Input mode */
			if (pin->bias_pull_up || pin->bias_pull_down) {
				cfg = 0x08;  /* Input with pull-up/down (CNF=10, MODE=00) */
			} else {
				cfg = 0x04;  /* Floating input (CNF=01, MODE=00) */
			}
		}

		/* Configure the pin */
		volatile uint32_t *cfg_reg;
		uint8_t bit_pos;

		if (pin_num < 8) {
			cfg_reg = (volatile uint32_t *)(gpio_base + GPIO_CFGLR_OFFSET);
			bit_pos = pin_num * 4;
		} else if (pin_num < 16) {
			cfg_reg = (volatile uint32_t *)(gpio_base + GPIO_CFGHR_OFFSET);
			bit_pos = (pin_num - 8) * 4;
		} else {
			cfg_reg = (volatile uint32_t *)(gpio_base + GPIO_CFGXR_OFFSET);
			bit_pos = (pin_num - 16) * 4;
		}

		*cfg_reg = (*cfg_reg & ~(0x0F << bit_pos)) | (cfg << bit_pos);

		/* Set initial output value or pull direction */
		volatile uint32_t *bshr = (volatile uint32_t *)(gpio_base + GPIO_BSHR_OFFSET);

		if (pin->output_high || pin->bias_pull_up) {
			*bshr = BIT(pin_num);  /* Set bit / pull-up */
		} else if (pin->output_low || pin->bias_pull_down) {
			*bshr = BIT(pin_num + 16);  /* Reset bit / pull-down */
		}
	}

	return 0;
}

static int pinctrl_ch32x035_init(void)
{
	/* Enable AFIO clock */
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO;
	return 0;
}

SYS_INIT(pinctrl_ch32x035_init, PRE_KERNEL_1, 0);
