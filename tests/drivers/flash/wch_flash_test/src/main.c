/*
 * WCH Flash Driver Verification Test
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/printk.h>

#define TEST_OFFSET  0xF800  /* Near end of 64KB flash */
#define TEST_SIZE    256     /* One erase page */

static uint8_t write_buf[TEST_SIZE];
static uint8_t read_buf[TEST_SIZE];

int main(void)
{
	const struct device *flash_dev;
	int rc;

	/* Delay for console init */
	k_sleep(K_SECONDS(2));

	printk("*** WCH Flash Driver Test ***\n");

	flash_dev = DEVICE_DT_GET(DT_NODELABEL(flash));
	if (!device_is_ready(flash_dev)) {
		printk("ERROR: Flash device not ready\n");
		return 0;
	}

	printk("Flash device: %s\n", flash_dev->name);

	/* Prepare test pattern */
	for (int i = 0; i < TEST_SIZE; i++) {
		write_buf[i] = (uint8_t)(i & 0xFF);
	}

	/* Erase */
	printk("Erasing %d bytes at offset 0x%X...\n", TEST_SIZE, TEST_OFFSET);
	rc = flash_erase(flash_dev, TEST_OFFSET, TEST_SIZE);
	if (rc != 0) {
		printk("ERROR: Erase failed: %d\n", rc);
		return 0;
	}
	printk("Erase OK.\n");

	/* Write */
	printk("Writing %d bytes...\n", TEST_SIZE);
	rc = flash_write(flash_dev, TEST_OFFSET, write_buf, TEST_SIZE);
	if (rc != 0) {
		printk("ERROR: Write failed: %d\n", rc);
		return 0;
	}
	printk("Write OK.\n");

	/* Read */
	printk("Reading %d bytes...\n", TEST_SIZE);
	rc = flash_read(flash_dev, TEST_OFFSET, read_buf, TEST_SIZE);
	if (rc != 0) {
		printk("ERROR: Read failed: %d\n", rc);
		return 0;
	}
	printk("Read OK.\n");

	/* Verify */
	printk("Verifying...\n");
	int errors = 0;
	for (int i = 0; i < TEST_SIZE; i++) {
		if (read_buf[i] != write_buf[i]) {
			if (errors < 5) {
				printk("  Mismatch at %d: wrote 0x%02X, read 0x%02X\n",
				       i, write_buf[i], read_buf[i]);
			}
			errors++;
		}
	}

	if (errors == 0) {
		printk("\n*** SUCCESS: Flash Driver Verified! ***\n");
	} else {
		printk("\n*** FAILED: %d mismatches ***\n", errors);
	}

	/* Keep printing so logs are captured */
	while (1) {
		printk("Test Complete.\n");
		k_sleep(K_SECONDS(2));
	}
	return 0;
}
