#include <zephyr/kernel.h>
#include <sdi_console.h>

int main(void)
{
	sdi_console_init();
	sdi_console_puts("WCH LPTIM System Timer Test\n");
	sdi_console_printf("Cycles per sec: %u\n", sys_clock_hw_cycles_per_sec());

	while (1) {
		sdi_console_printf("Uptime: %lld ms\n", k_uptime_get());
		k_msleep(1000);
	}

	return 0;
}
