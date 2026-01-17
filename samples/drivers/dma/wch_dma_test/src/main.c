/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WCH DMA Driver Validation Test
 *
 * This sample validates the DMA driver by performing:
 * 1. Device readiness check
 * 2. Memory-to-memory transfer with callback verification
 * 3. Data integrity verification
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <string.h>

LOG_MODULE_REGISTER(dma_test, LOG_LEVEL_INF);

/* Test configuration */
#define DMA_TEST_CHANNEL     0
#define DMA_TEST_SIZE        64
#define DMA_TEST_TIMEOUT_MS  1000

/* LED for visual feedback (optional) */
#define LED0_NODE DT_ALIAS(led0)
#if DT_NODE_EXISTS(LED0_NODE)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define HAS_LED 1
#else
#define HAS_LED 0
#endif

/* DMA device */
#define DMA_NODE DT_NODELABEL(dma1)
static const struct device *dma_dev = DEVICE_DT_GET(DMA_NODE);

/* Test buffers - aligned for optimal DMA performance */
static __aligned(4) uint8_t src_buffer[DMA_TEST_SIZE];
static __aligned(4) uint8_t dst_buffer[DMA_TEST_SIZE];

/* Callback synchronization */
static K_SEM_DEFINE(dma_done_sem, 0, 1);
static volatile int dma_callback_status;

/*
 * DMA Callback
 */
static void dma_callback(const struct device *dev, void *user_data,
			 uint32_t channel, int status)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("DMA callback: channel=%u, status=%d", channel, status);
	dma_callback_status = status;
	k_sem_give(&dma_done_sem);
}

/*
 * Initialize test buffers
 */
static void init_buffers(void)
{
	/* Fill source with known pattern */
	for (int i = 0; i < DMA_TEST_SIZE; i++) {
		src_buffer[i] = (uint8_t)(i ^ 0xA5);
	}

	/* Clear destination */
	memset(dst_buffer, 0, DMA_TEST_SIZE);
}

/*
 * Verify transfer result
 */
static bool verify_transfer(void)
{
	for (int i = 0; i < DMA_TEST_SIZE; i++) {
		if (dst_buffer[i] != src_buffer[i]) {
			LOG_ERR("Data mismatch at index %d: expected 0x%02x, got 0x%02x",
				i, src_buffer[i], dst_buffer[i]);
			return false;
		}
	}
	return true;
}

/*
 * Run Memory-to-Memory DMA test
 */
static int test_m2m_transfer(void)
{
	struct dma_config dma_cfg = {0};
	struct dma_block_config block_cfg = {0};
	int ret;

	LOG_INF("=== Test: Memory-to-Memory Transfer ===");

	/* Initialize buffers */
	init_buffers();

	/* Configure DMA block */
	block_cfg.source_address = (uint32_t)src_buffer;
	block_cfg.dest_address = (uint32_t)dst_buffer;
	block_cfg.block_size = DMA_TEST_SIZE;
	block_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	block_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	/* Configure DMA channel */
	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = 1;  /* Byte transfers */
	dma_cfg.dest_data_size = 1;
	dma_cfg.source_burst_length = 1;
	dma_cfg.dest_burst_length = 1;
	dma_cfg.channel_priority = 0;
	dma_cfg.head_block = &block_cfg;
	dma_cfg.block_count = 1;
	dma_cfg.dma_callback = dma_callback;
	dma_cfg.user_data = NULL;
	dma_cfg.complete_callback_en = false;
	dma_cfg.error_callback_dis = false;

	/* Reset semaphore */
	k_sem_reset(&dma_done_sem);
	dma_callback_status = -EINPROGRESS;

	/* Configure the channel */
	ret = dma_config(dma_dev, DMA_TEST_CHANNEL, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("dma_config failed: %d", ret);
		return ret;
	}

	LOG_INF("Starting DMA transfer: src=0x%08x, dst=0x%08x, size=%u",
		(uint32_t)src_buffer, (uint32_t)dst_buffer, DMA_TEST_SIZE);

	/* Start the transfer */
	ret = dma_start(dma_dev, DMA_TEST_CHANNEL);
	if (ret != 0) {
		LOG_ERR("dma_start failed: %d", ret);
		return ret;
	}

	/* Wait for completion */
	ret = k_sem_take(&dma_done_sem, K_MSEC(DMA_TEST_TIMEOUT_MS));
	if (ret != 0) {
		LOG_ERR("DMA transfer timeout!");
		dma_stop(dma_dev, DMA_TEST_CHANNEL);
		return -ETIMEDOUT;
	}

	/* Check callback status */
	if (dma_callback_status != DMA_STATUS_COMPLETE) {
		LOG_ERR("DMA callback reported error: %d", dma_callback_status);
		return dma_callback_status;
	}

	/* Verify data */
	if (!verify_transfer()) {
		LOG_ERR("Data verification FAILED!");
		return -EIO;
	}

	LOG_INF("Memory-to-Memory transfer: PASSED");
	return 0;
}

/*
 * Run DMA status test
 */
static int test_get_status(void)
{
	struct dma_status status;
	int ret;

	LOG_INF("=== Test: Get Status ===");

	ret = dma_get_status(dma_dev, DMA_TEST_CHANNEL, &status);
	if (ret != 0) {
		LOG_ERR("dma_get_status failed: %d", ret);
		return ret;
	}

	LOG_INF("Channel %u status: busy=%d, dir=%d, pending_length=%u",
		DMA_TEST_CHANNEL, status.busy, status.dir, status.pending_length);

	LOG_INF("DMA Test Complete");

    extern void sdi_console_puts(const char *str);
    while (1) {
        // printk("Alive: %d ms\n", k_uptime_get_32());
        sdi_console_puts("Alive\n");
        k_sleep(K_SECONDS(1));
    }

	return 0;
}

/*
 * Run DMA attributes test
 */
static int test_get_attributes(void)
{
	uint32_t value;
	int ret;

	LOG_INF("=== Test: Get Attributes ===");

	ret = dma_get_attribute(dma_dev, DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT, &value);
	if (ret != 0) {
		LOG_ERR("Failed to get buffer alignment: %d", ret);
		return ret;
	}
	LOG_INF("Buffer alignment: %u", value);

	ret = dma_get_attribute(dma_dev, DMA_ATTR_MAX_BLOCK_COUNT, &value);
	if (ret != 0) {
		LOG_ERR("Failed to get max block count: %d", ret);
		return ret;
	}
	LOG_INF("Max block count: %u", value);

	LOG_INF("Get attributes: PASSED");
	return 0;
}

/*
 * Main entry point
 */
int main(void)
{
	int ret;
	int pass_count = 0;
	int fail_count = 0;

	/* Manual init to ensure console is ready */
	/* extern void sdi_console_init(void); */
	/* extern void sdi_console_puts(const char *str); */
	/* sdi_console_init(); */

    /* sdi_console_puts("\n\n*** WCH DMA Test Starting (Direct SDI) ***\n"); */


	LOG_INF("====================================");
	LOG_INF("WCH DMA Driver Validation Test");
	LOG_INF("====================================");

	/* Initialize LED (optional) */
#if HAS_LED
	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}
#endif

	/* Check DMA device readiness */
	if (!device_is_ready(dma_dev)) {
		LOG_ERR("FATAL: DMA device not ready!");
		return -ENODEV;
	}
	LOG_INF("DMA device ready: %s", dma_dev->name);

	/* Run tests */
	LOG_INF("");

	/* Test 1: Memory-to-Memory */
	ret = test_m2m_transfer();
	if (ret == 0) {
		pass_count++;
	} else {
		fail_count++;
	}

	LOG_INF("");

	/* Test 2: Get Status */
	ret = test_get_status();
	if (ret == 0) {
		pass_count++;
	} else {
		fail_count++;
	}

	LOG_INF("");

	/* Test 3: Get Attributes */
	ret = test_get_attributes();
	if (ret == 0) {
		pass_count++;
	} else {
		fail_count++;
	}

	/* Summary */
	LOG_INF("");
	LOG_INF("====================================");
	if (fail_count == 0) {
		LOG_INF("SUCCESS: All %d tests passed!", pass_count);
#if HAS_LED
		if (gpio_is_ready_dt(&led)) {
			gpio_pin_set_dt(&led, 1);  /* LED on for success */
		}
#endif
	} else {
		LOG_ERR("FAILED: %d passed, %d failed", pass_count, fail_count);
#if HAS_LED
		/* Blink LED for failure */
		while (1) {
			if (gpio_is_ready_dt(&led)) {
				gpio_pin_toggle_dt(&led);
			}
			k_msleep(250);
		}
#endif
	}
	LOG_INF("====================================");

	/* Keep alive for logging */
	while (1) {
		k_msleep(1000);
	}

	return 0;
}
