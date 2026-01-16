#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/pm/pwr_wch.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/timeutil.h>
#include <time.h>
#include <soc.h>

extern void sdi_console_init(void);
extern void sdi_console_printf(const char *format, ...);

#define LOG_INF(fmt, ...) sdi_console_printf("INF: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) sdi_console_printf("ERR: " fmt "\n", ##__VA_ARGS__)

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	const struct device *rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtc));
	const struct device *pwr_dev = DEVICE_DT_GET(DT_NODELABEL(pwr));
	struct rtc_time time;
	int ret;

	sdi_console_init();
    
    if (device_is_ready(led.port)) {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    }

	k_sleep(K_MSEC(100)); // Wait for SDI terminal to pick up

	LOG_INF("WCH PM/RTC Verification App");
	LOG_INF("Core Clock: %u Hz", sys_clock_hw_cycles_per_sec());
    LOG_INF("Starting timing test at t=0");

	// 1. Verify PWR Device
	if (!device_is_ready(pwr_dev)) {
		LOG_ERR("PWR device not ready!");
		return 0;
	}
	LOG_INF("PWR device is ready.");

	// 2. Verify RTC Device
	if (!device_is_ready(rtc_dev)) {
		LOG_ERR("RTC device not ready!");
		return 0;
	}
	LOG_INF("RTC device is ready.");

	// 3. Set RTC Time
	// Use a fixed timestamp: 2026-01-01 12:00:00 (1767268800)
	time.tm_year = 126; // Years since 1900
	time.tm_mon = 0;   // January
	time.tm_mday = 1;
	time.tm_hour = 12;
	time.tm_min = 0;
	time.tm_sec = 0;
	time.tm_isdst = -1;

	LOG_INF("Setting RTC time to 2026-01-01 12:00:00...");
	ret = rtc_set_time(rtc_dev, &time);
	if (ret < 0) {
		LOG_ERR("Failed to set RTC time: %d", ret);
	} else {
		LOG_INF("RTC time set successfully.");
	}

	LOG_INF("Waiting 2s for RTC to increment...");
    for (int i = 0; i < 4; i++) {
        k_sleep(K_MSEC(500));
        LOG_INF("... wait %d/4", i + 1);
    }

	// 4. Read RTC Time back
    LOG_INF("Reading back RTC Time...");
	ret = rtc_get_time(rtc_dev, &time);
	if (ret < 0) {
		LOG_ERR("Failed to get RTC time: %d", ret);
	} else {
		LOG_INF("Readback RTC Time: %04d-%02d-%02d %02d:%02d:%02d",
			time.tm_year + 1900, time.tm_mon + 1, time.tm_mday,
			time.tm_hour, time.tm_min, time.tm_sec);
	}

	// 5. Test Backup Domain Access via PWR API
	LOG_INF("Testing Backup Domain Write Access...");
	
	// Enable backup access
	wch_pwr_set_backup_access(pwr_dev, true);
	LOG_INF("PWR DBP bit set (Backup Access Enabled).");

	// BKP->DATAR1 is usually at 0x40006C04 for many WCH chips
	// But let's check BKP peripheral address
	#ifdef BKP
	BKP->DATAR1 = 0xABCD;
	uint16_t bkp_val = BKP->DATAR1;
	if (bkp_val == 0xABCD) {
		LOG_INF("BKP Register Write SUCCESS: 0x%04X", bkp_val);
	} else {
		LOG_ERR("BKP Register Write FAIL: expected 0xABCD, got 0x%04X", bkp_val);
	}
	#else
	LOG_INF("BKP peripheral not defined in HAL headers.");
	#endif

	// Disable backup access
	wch_pwr_set_backup_access(pwr_dev, false);
	LOG_INF("PWR DBP bit cleared (Backup Access Disabled).");

	#ifdef BKP
	// Try writing while disabled (should fail or be ignored depends on chip)
	BKP->DATAR1 = 0x1234;
	bkp_val = BKP->DATAR1;
	LOG_INF("BKP value after 'disabled' write: 0x%04X (should stay 0xABCD if protected)", bkp_val);
	#endif

	// 6. Test Low Power Mode (Sleep)
	LOG_INF("Entering SLEEP mode for 5 seconds (will wake up on LPTIM tick)...");
	// Note: wch_pwr_enter_low_power uses PWR_CTLR_PDDS and WFE/WFI
	// Mode 0 = Sleep, 1 = Stop, 2 = Standby in my driver implementation
	
	k_sleep(K_MSEC(100)); // Make sure UART/SDI finishes
	
	// We use our driver specifically once to verify it.
	wch_pwr_enter_low_power(pwr_dev, 0); // SLEEP
	
	LOG_INF("Woke up from SLEEP.");
	
    uint32_t loop_cnt = 0;
	while (1) {
		k_sleep(K_SECONDS(1));
        gpio_pin_toggle_dt(&led);
		rtc_get_time(rtc_dev, &time);
		LOG_INF("[%u] RTC: %02d:%02d:%02d", loop_cnt++, time.tm_hour, time.tm_min, time.tm_sec);
	}

	return 0;
}
