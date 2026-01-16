#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	k_busy_wait(3000000);
	printk("WCH USBFS CDC ACM Test (Next Stack)\n");
	printk("USB Should be enabled by Kconfig...\n");

	while (1) {
		k_sleep(K_SECONDS(1));
		printk("Alive...\n");
	}

	return 0;
}
