#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/console/console.h>
#include <zephyr/drivers/console/sdi_console.h>
#include <hal_ch32fun.h> /* For LPTIM_TypeDef */

#define DT_DRV_COMPAT wch_ch32_lptim_pwm
#define PWM_DEVICE DT_INST(0, wch_ch32_lptim_pwm)

int main(void)
{
	const struct device *pwm = DEVICE_DT_GET(PWM_DEVICE);
	uint32_t period_us = 1000; /* 1 kHz */
	uint64_t cycles_per_sec;
	int ret;

	sdi_console_init();
	sdi_console_puts("WCH LPTIM PWM Test\n");

	if (!device_is_ready(pwm)) {
		sdi_console_puts("FAIL: PWM device not ready\n");
		return 1;
	}

	sdi_console_puts("PWM device ready\n");

	/* Enable GPIOB clock (Required for AFIO remapping/configuration) */
	RCC->APB2PCENR |= RCC_PB2Periph_GPIOB;

	/* Configure PB15 as AF Push-Pull for PWM (0xB, 50MHz) */
	/* Note: This should ideally be handled by pinctrl, but keeping for direct verification */
	GPIOB->CFGHR &= ~(0xF << ((15 - 8) * 4));
	GPIOB->CFGHR |= (0xB << ((15 - 8) * 4)); 


	/* Get PWM frequency */
	ret = pwm_get_cycles_per_sec(pwm, 0, &cycles_per_sec);
	if (ret) {
		sdi_console_printf("Failed to get cycles/sec: %d\n", ret);
		return ret;
	}
	sdi_console_printf("PWM freq: %u Hz\n", (uint32_t)cycles_per_sec);

	/* Set fixed 50% duty cycle first to test */
	ret = pwm_set(pwm, 0, PWM_USEC(period_us), PWM_USEC(period_us / 2), 0);
	if (ret) {
		sdi_console_printf("PWM set failed: %d\n", ret);
		return ret;
	}

	/* Sine Wave LUT (100 steps, scaled 0-1000) for symmetric breathing */
	const uint16_t sine_wave[] = {
		0, 0, 3, 8, 15, 24, 35, 47, 61, 77, 95, 114, 135, 157, 181, 206, 232, 259, 287, 315, 
		345, 375, 406, 437, 468, 499, 531, 562, 593, 624, 654, 684, 712, 740, 767, 793, 818, 
		842, 864, 885, 904, 922, 938, 952, 964, 975, 984, 991, 996, 999, 1000, 999, 996, 991, 
		984, 975, 964, 952, 938, 922, 904, 885, 864, 842, 818, 793, 767, 740, 712, 684, 654, 
		624, 593, 562, 531, 500, 468, 437, 406, 375, 345, 315, 287, 259, 232, 206, 181, 157, 
		135, 114, 95, 77, 61, 47, 35, 24, 15, 8, 3, 0
	};
	
	/* Target: ~3s period (Slow breathing) */
	/* Steps: 100. Real delay needed: 30ms/step. */
	/* SysTick ~9x fast -> Programmed delay: 30*9 = 270ms. 300ms safe. */
	const int sleep_ms = 300; 
	int lut_idx = 0;

	sdi_console_puts("Starting Sine Wave Breathing Loop (Slow)...\n");

	while (1) {
		/* LED is Active Low (0=On, Period=Off). 
		 * Invert logic: Pulse width defines "Off" time? 
		 * Standard PWM: Pulse = High time. 
		 * Active Low LED: High = Off. Low = On.
		 * So Pulse (High) = Off brightness.
		 * To be ON (Low), we want Short Pulse (mostly Low).
		 * WAIT. 
		 * If 0 is ON.
		 * Sine wave: 0 -> 1000 -> 0.
		 * Logic 0 -> 1000 -> 0.
		 * LED On -> Off -> On.
		 * That's what we see.
		 * We want: Off -> On -> Off.
		 * So we need logic: 1000 -> 0 -> 1000.
		 * pulse = period_us - sine_val.
		 */ 
		uint32_t sine_val = sine_wave[lut_idx];
		uint32_t pulse = period_us - sine_val;
		
		ret = pwm_set(pwm, 0, PWM_USEC(period_us), PWM_USEC(pulse), 0);
		if (ret) {
			sdi_console_printf("PWM set failed: %d\n", ret);
			return ret;
		}
		
		lut_idx++;
		if (lut_idx >= 100) {
			lut_idx = 0;
		}
		
		k_msleep(sleep_ms);
	}

	return 0;
}
