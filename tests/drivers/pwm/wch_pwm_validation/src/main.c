#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <stdio.h>

/* CH32L103 TIM1 base address */
#define TIM1_BASE_ADDR 0x40012C00
#define TIM_RPTCR_OFFSET 0x30

#include <zephyr/drivers/console/sdi_console.h>

/* CH32L103 TIM1 base address */
#define TIM1_BASE_ADDR 0x40012C00
#define TIM_CTLR1_OFFSET 0x00
#define TIM_CTLR2_OFFSET 0x04
#define TIM_SMCFGR_OFFSET 0x08
#define TIM_DMAINTENR_OFFSET 0x0C
#define TIM_INTFR_OFFSET 0x10
#define TIM_SWEVGR_OFFSET 0x14
#define TIM_CHCTLR1_OFFSET 0x18
#define TIM_CHCTLR2_OFFSET 0x1C
#define TIM_CCER_OFFSET 0x20
#define TIM_CNT_OFFSET 0x24
#define TIM_PSC_OFFSET 0x28
#define TIM_ATRLR_OFFSET 0x2C
#define TIM_RPTCR_OFFSET 0x30
#define TIM_CH1CVR_OFFSET 0x34
#define TIM_CH2CVR_OFFSET 0x38
#define TIM_CH3CVR_OFFSET 0x3C
#define TIM_CH4CVR_OFFSET 0x40
#define TIM_BDTR_OFFSET 0x44

void dump_tim1_registers(void)
{
	sdi_console_printf("--- TIM1 Register Dump ---\n");
	sdi_console_printf("TIM1->CTLR1    = 0x%04x\n", *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_CTLR1_OFFSET));
	sdi_console_printf("TIM1->CTLR2    = 0x%04x\n", *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_CTLR2_OFFSET));
	sdi_console_printf("TIM1->CCER     = 0x%04x\n", *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_CCER_OFFSET));
	sdi_console_printf("TIM1->PSC      = 0x%04x\n", *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_PSC_OFFSET));
	sdi_console_printf("TIM1->ATRLR    = 0x%04x\n", *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_ATRLR_OFFSET));
	sdi_console_printf("TIM1->RPTCR    = 0x%04x\n", *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_RPTCR_OFFSET));
	sdi_console_printf("TIM1->CH1CVR   = 0x%04x\n", *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_CH1CVR_OFFSET));
	sdi_console_printf("TIM1->BDTR     = 0x%04x\n", *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_BDTR_OFFSET));
	sdi_console_printf("--------------------------\n");
}

int main(void)
{
	const struct device *pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm1));
	uint16_t rptcr_val;

	sdi_console_init();
	sdi_console_puts("WCH TIM1 RCR Validation Test (SDI Console)\n");

	if (!device_is_ready(pwm_dev)) {
		sdi_console_puts("FAIL: PWM device not ready\n");
		return 1;
	}

	while (1) {
		dump_tim1_registers();
		
		rptcr_val = *(volatile uint16_t *)(TIM1_BASE_ADDR + TIM_RPTCR_OFFSET);
		sdi_console_printf("CHECK: TIM1->RPTCR = %u\n", rptcr_val);

#ifdef EXPECTED_RPTCR
		if (rptcr_val == EXPECTED_RPTCR) {
			sdi_console_printf("PASS: Match expected value %u\n", (uint16_t)EXPECTED_RPTCR);
		} else {
			sdi_console_printf("FAIL: Expected %u, got %u\n", (uint16_t)EXPECTED_RPTCR, rptcr_val);
		}
#endif
		k_msleep(2000);
	}

	return 0;
}
