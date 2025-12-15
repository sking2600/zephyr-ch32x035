/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/arch/riscv/irq.h>
#include "soc.h"
#include <zephyr/pm/pm.h>
#include <zephyr/irq.h>

/* CH32X035 runs at fixed 48MHz from internal oscillator */

static void clock_init(void)
{
    /* Configure flash wait states for 48MHz */
#ifdef FLASH_ACTLR_LATENCY
    FLASH->ACTLR = (FLASH->ACTLR & ~FLASH_ACTLR_LATENCY) | FLASH_ACTLR_LATENCY_2;
#endif

    /* HCLK = SYSCLK / 1 */
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV1;

    /* Enable DMA1 clock via direct register write (replaces RCC_AHBPeriphClockCmd) */
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

    /* Enable AFIO clock for pin remap and EXTI (replaces RCC_APB2PeriphClockCmd) */
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

    /* Enable GPIO clocks - needed for any GPIO operation */
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC;
}

/**
 * @brief SoC-level IRQ handler called from arch ISR
 * 
 * The WCH PFIC (Programmable Fast Interrupt Controller) handles IRQ dispatch.
 * This function is called by the RISC-V arch layer to acknowledge and handle
 * the interrupt at the SoC level.
 */
void __soc_handle_irq(unsigned long irq)
{
    /* For WCH PFIC, no special acknowledgment needed at SoC level.
     * The PFIC driver handles the interrupt controller specifics.
     * This stub satisfies the arch layer requirement.
     */
    ARG_UNUSED(irq);
}

/**
 * @brief SoC early initialization
 */
static int wch_ch32x035_init(void)
{
    clock_init();
    return 0;
}

SYS_INIT(wch_ch32x035_init, PRE_KERNEL_1, 0);

/* Power Management Hooks */

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		/* WFI puts the CPU in sleep mode until an interrupt occurs */
		arch_cpu_idle();
		break;
	case PM_STATE_STANDBY:
		/* Enter Standby mode */
		/* Set SLEEPDEEP bit in System Control Register (handled by arch if configured?) 
		 * For now, only WFI is safe without more complex context save/restore
		 */
		 __asm__ volatile("wfi");
		break;
	default:
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}
