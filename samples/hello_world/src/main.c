/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

/* 
 * NOTE: soc.h includes ch32fun headers (ch32l103hw.h) which define:
 * - RCC, FLASH, EXTEN, certain peripheral bases
 * - RCC_TypeDef, EXTEN_TypeDef, etc.
 * - Bit definitions like RCC_HSION, EXTEN_PLL_HSI_PRE, etc.
 */

/* -------------------------------------------------------------------------
 * LOW-LEVEL FUNCTIONS
 * ------------------------------------------------------------------------- */

void direct_uart_putc(char c) {
    while (!(USART1->STATR & (1 << 7))) {
        /* Wait for TXE (Transmit Data Register Empty) */
        /* Note: USART_STATR_TXE is (1<<7) in HAL */
    }
    USART1->DATAR = c;
}

void direct_uart_puts(const char *s) {
    while (*s) {
        direct_uart_putc(*s++);
    }
    direct_uart_putc('\r');
    direct_uart_putc('\n');
}

void print_hex(uint32_t val) {
    char hex_digits[] = "0123456789ABCDEF";
    direct_uart_putc('0');
    direct_uart_putc('x');
    for (int i = 7; i >= 0; i--) {
        direct_uart_putc(hex_digits[(val >> (i * 4)) & 0xF]);
    }
    direct_uart_puts(""); 
}

void vendor_clock_init_72mhz(void) {
    /* 1. Enable HSI */
    RCC->CTLR |= RCC_HSION;
    while (!(RCC->CTLR & RCC_HSIRDY));

    /* 2. Vendor Magic: Enable EXTEN HSI Pre-divide Bypass */
    /* ENABLE AFIO CLOCK FIRST - POTENTIAL FIX */
    RCC->PB2PCENR |= RCC_AFIOEN; 
    
    RCC->HBPCENR |= RCC_DMAEN | RCC_SRAMEN; 

    EXTEN->EXTEN_CTR |= EXTEN_PLL_HSI_PRE;

    /* 3. Configure PLL: HSI Source (via HSI/2 mux bit=0), Mul 9 */
    RCC->CFGR0 &= ~(RCC_PLLMULL); 
    RCC->CFGR0 |= RCC_PLLMULL9;   
    RCC->CFGR0 &= ~RCC_PLLSRC;    

    /* 4. Turn on PLL */
    RCC->CTLR |= RCC_PLLON;
    while (!(RCC->CTLR & RCC_PLLRDY));

    /* 5. Set Flash Latency for 72MHz */
    FLASH->ACTLR &= ~FLASH_ACTLR_LATENCY;
    FLASH->ACTLR |= FLASH_ACTLR_LATENCY_2;

    /* 6. Switch System Clock to PLL */
    RCC->CFGR0 &= ~RCC_SW;     
    RCC->CFGR0 |= RCC_SW_PLL;  
    while ((RCC->CFGR0 & RCC_SWS) != RCC_SWS_PLL);

    /* 7. Enable Peripherals */
    RCC->PB2PCENR |= RCC_IOPAEN | RCC_USART1EN | RCC_IOPCEN;
}

int main(void) {
    (void)irq_lock();

    vendor_clock_init_72mhz();

    /* GPIO INIT (PC13 LED) */
    GPIOC->CFGHR &= ~(0xF << 20); 
    GPIOC->CFGHR |= (0x3 << 20);

    /* UART INIT */
    GPIOA->CFGHR &= ~(0xF << 4);
    GPIOA->CFGHR |= (0xB << 4);
    
    RCC->PB2PCENR |= RCC_USART1EN;
    USART1->CTLR1 = (1 << 13) | (1 << 3) | (1 << 2); 
    
    /* DYNAMIC BAUD RATE & DELAY SELECTION */
    /* Check if EXTEN bit stuck */
    int is_72mhz = (EXTEN->EXTEN_CTR & EXTEN_PLL_HSI_PRE);
    int delay_count;
    
    if (is_72mhz) {
        /* 72MHz / 115200 = 625 = 0x271 << 4 | 0 = 0x2710 */
        USART1->BRR = 0x2710; 
        delay_count = 7200000;
    } else {
        /* 36MHz / 115200 = 312.5 = 0x138 << 4 | 8 = 0x1388 */
        USART1->BRR = 0x1388;
        delay_count = 3600000;
    }

    direct_uart_puts("\r\n\r\n*** CH32L103 DIAGNOSTIC FIRMWARE ***");
    direct_uart_puts("Detected Mode:");
    if (is_72mhz) direct_uart_puts("72MHz (EXTEN Set)");
    else          direct_uart_puts("36MHz (EXTEN Failed)");

    direct_uart_puts("EXTEN->EXTEN_CTR:");
    print_hex(EXTEN->EXTEN_CTR);
    direct_uart_puts("RCC->CFGR0:");
    print_hex(RCC->CFGR0);
    
    int count = 0;
    while (1) {
        GPIOC->BSHR = (1 << 13); /* LED OFF */
        volatile int d;
        for(d=0; d<delay_count; d++); 
        
        GPIOC->BCR = (1 << 13);  /* LED ON */
        
        if (is_72mhz) direct_uart_putc(0x72); /* 'r' for 72 */
        else          direct_uart_putc(0x36); /* '6' for 36 */
        
        for(d=0; d<delay_count; d++);
        
        count++;
    }
}
