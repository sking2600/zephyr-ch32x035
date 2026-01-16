/*
 * Copyright (c) 2024 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_lptim

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <hal_ch32fun.h>

#define LPTIM ((LPTIM_TypeDef *)DT_INST_REG_ADDR(0))

static struct k_spinlock lock;
static uint32_t accumulated_cycles;
static uint32_t last_announcement_cycles;

/* Get LSI frequency from Device Tree */
#if DT_NODE_EXISTS(DT_NODELABEL(clk_lsi))
#define LSI_FREQUENCY DT_PROP(DT_NODELABEL(clk_lsi), clock_frequency)
#else
#warning "LSI clock node not found in DTS, defaulting to 40kHz"
#define LSI_FREQUENCY 40000
#endif

#define CYCLES_PER_TICK (LSI_FREQUENCY / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

static void lptim_irq_handler(const void *unused)
{
	ARG_UNUSED(unused);

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t isr = LPTIM->ISR;

	if (isr & LPTIM_ISR_ARRM) {
		/* Clear the flag */
		LPTIM->ICR |= LPTIM_ICR_ARRMCF;
		
		accumulated_cycles += (LPTIM->ARR + 1);
		
		uint32_t elapsed = (accumulated_cycles - last_announcement_cycles) / CYCLES_PER_TICK;
		if (elapsed > 0) {
			last_announcement_cycles += elapsed * CYCLES_PER_TICK;
			k_spin_unlock(&lock, key);
			sys_clock_announce(elapsed);
			return;
		}
	}

	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	
	uint32_t cnt;
	uint32_t prev;
	
	/* Double read for sync */
	cnt = LPTIM->CNT;
	do {
		prev = cnt;
		cnt = LPTIM->CNT;
	} while (cnt != prev);
	
	uint32_t total = accumulated_cycles + cnt;
	
	k_spin_unlock(&lock, key);
	return total;
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}
	
	/* For now, just return 0 to indicate no extra elapsed time since last announce */
	/* We will implement full tickless support later */
	return 0;
}

static int sys_clock_driver_init(void)
{
	/* Enable LPTIM clock in RCC */
	RCC->APB1PCENR |= RCC_PB1Periph_LPTIM;

	/* NVIC configuration */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), lptim_irq_handler, NULL, 0);
	irq_enable(DT_INST_IRQN(0));

	/* Configure LPTIM */
	LPTIM->CR = 0; // Disable
	
	/* Use LSI (11: selection bits 25-26), set prescaler 1, enable timeout */
	/* Bits 25-26: 0:PCLK, 1:HSI, 2:LSE, 3:LSI */
	LPTIM->CFGR = LPTIM_CFGR_TIMOUT | (3 << 25);
	
	LPTIM->CR |= LPTIM_CR_ENABLE;
	
	/* Set ARR for periodic tick */
	LPTIM->ARR = (uint16_t)(CYCLES_PER_TICK - 1);
	
	/* Wait for ARROK */
	uint32_t timeout = 100000;
	while (!(LPTIM->ISR & LPTIM_ISR_ARROK) && timeout--);
	LPTIM->ICR |= LPTIM_ICR_ARROKCF;
	
	/* Enable ARRM interrupt */
	LPTIM->IER |= LPTIM_IER_ARRMIE;
	
	/* Start in continuous mode */
	LPTIM->CR |= LPTIM_CR_CNTSTRT;

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
