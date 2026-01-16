/*
 * Copyright (c) 2026 Scott King
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_flash_controller

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <soc.h>

/* Place critical functions in RAM via .data section */
#define RAMFUNC __attribute__((section(".data"), noinline))

LOG_MODULE_REGISTER(flash_wch, CONFIG_FLASH_LOG_LEVEL);

/* FLASH Keys */
#define FLASH_KEY1                 ((uint32_t)0x45670123)
#define FLASH_KEY2                 ((uint32_t)0xCDEF89AB)

/* Timeout in loop iterations (empirically tuned, ~50ms at 8MHz) */
#define FLASH_TIMEOUT_LOOPS        400000

/* Hardware Buffer Size for Fast Program */
#define FLASH_PAGE_SIZE            128

/* Erase Page Size (256 bytes for CH32L1xx) */
#define FLASH_ERASE_SIZE           256

struct flash_wch_config {
	FLASH_TypeDef *regs;
};

struct flash_wch_data {
	struct k_sem sem;
};

static const struct flash_parameters flash_wch_parameters = {
	.write_block_size = 4,
	.erase_value = 0xff,
};

/*
 * Simple delay loop, runs entirely from RAM.
 * Avoids any Flash access.
 */
static RAMFUNC void flash_wch_delay(volatile uint32_t count)
{
	while (count--) {
		__asm__ volatile("nop");
	}
}

static RAMFUNC void flash_wch_unlock(FLASH_TypeDef *regs)
{
	regs->KEYR = FLASH_KEY1;
	regs->KEYR = FLASH_KEY2;
	regs->MODEKEYR = FLASH_KEY1;
	regs->MODEKEYR = FLASH_KEY2;
}

static RAMFUNC void flash_wch_lock(FLASH_TypeDef *regs)
{
	regs->CTLR |= FLASH_CTLR_LOCK | FLASH_CTLR_FLOCK;
}

/*
 * Wait for Busy flag. Runs from RAM.
 * Returns 0 on success, -ETIMEDOUT on timeout.
 */
static RAMFUNC int flash_wch_wait_bsy(FLASH_TypeDef *regs)
{
	uint32_t timeout = FLASH_TIMEOUT_LOOPS;

	while (regs->STATR & FLASH_STATR_BSY) {
		if (--timeout == 0) {
			return -ETIMEDOUT;
		}
		flash_wch_delay(10);
	}
	return 0;
}

static int flash_wch_read(const struct device *dev, off_t offset,
			  void *data, size_t len)
{
	if (!len) {
		return 0;
	}

	/* Flash is memory mapped, just memcpy */
	memcpy(data, (void *)(FLASH_BASE + offset), len);

	return 0;
}

/*
 * Program a single block using Fast Program (up to 128 bytes).
 * Must be called with interrupts locked and flash unlocked.
 */
static RAMFUNC int flash_wch_program_block(FLASH_TypeDef *regs, off_t addr,
					   const uint32_t *data, size_t count)
{
	int rc;
	size_t i;

	/* 1. Set FTPG (Fast Program) */
	regs->CTLR |= FLASH_CTLR_FTPG;

	/* 2. Reset Buffer */
	regs->CTLR |= FLASH_CTLR_BUFRST;
	rc = flash_wch_wait_bsy(regs);
	if (rc < 0) {
		regs->CTLR &= ~FLASH_CTLR_FTPG;
		return rc;
	}

	/* 3. Load words into buffer */
	for (i = 0; i < count; i++) {
		volatile uint32_t *dest = (volatile uint32_t *)(FLASH_BASE + addr + (i * 4));
		*dest = data[i];

		/* 4. Trigger BUFLOAD for this word */
		regs->CTLR |= FLASH_CTLR_BUFLOAD;
		/* Brief delay for buffer load, no BSY wait per word */
		flash_wch_delay(5);
	}

	/* 5. Trigger programming Start */
	regs->ADDR = FLASH_BASE + addr;
	regs->CTLR |= FLASH_CTLR_STRT;

	/* 6. Wait for completion */
	rc = flash_wch_wait_bsy(regs);

	/* 7. Clear FTPG */
	regs->CTLR &= ~FLASH_CTLR_FTPG;

	return rc;
}

static RAMFUNC int flash_wch_write_ram(const struct device *dev, off_t offset,
				       const void *data, size_t len)
{
	const struct flash_wch_config *cfg = dev->config;
	int rc = 0;
	const uint8_t *src = data;

	if ((offset % 4 != 0) || (len % 4 != 0)) {
		return -EINVAL;
	}

	flash_wch_unlock(cfg->regs);

	rc = flash_wch_wait_bsy(cfg->regs);
	if (rc < 0) {
		goto out;
	}

	/* Write in 128-byte page chunks (Hardware Buffer Size) */
	while (len > 0) {
		off_t pg_offset = offset & (FLASH_PAGE_SIZE - 1);
		size_t write_len = FLASH_PAGE_SIZE - pg_offset;
		if (write_len > len) {
			write_len = len;
		}

		rc = flash_wch_program_block(cfg->regs, offset, (const uint32_t *)src, write_len / 4);
		if (rc < 0) {
			break;
		}

		offset += write_len;
		src += write_len;
		len -= write_len;
	}

out:
	flash_wch_lock(cfg->regs);
	return rc;
}

static int flash_wch_write(const struct device *dev, off_t offset,
			   const void *data, size_t len)
{
	struct flash_wch_data *dev_data = dev->data;
	unsigned int key;
	int rc;

	k_sem_take(&dev_data->sem, K_FOREVER);
	key = irq_lock();

	rc = flash_wch_write_ram(dev, offset, data, len);

	irq_unlock(key);
	k_sem_give(&dev_data->sem);

	return rc;
}

static RAMFUNC int flash_wch_erase_ram(const struct device *dev, off_t offset, size_t len)
{
	const struct flash_wch_config *cfg = dev->config;
	int rc = 0;

	flash_wch_unlock(cfg->regs);

	rc = flash_wch_wait_bsy(cfg->regs);
	if (rc < 0) {
		goto out;
	}

	while (len > 0) {
		/* Fast Page Erase (256 bytes) */
		cfg->regs->CTLR |= FLASH_CTLR_FTER;
		cfg->regs->ADDR = FLASH_BASE + offset;
		cfg->regs->CTLR |= FLASH_CTLR_STRT;

		rc = flash_wch_wait_bsy(cfg->regs);
		cfg->regs->CTLR &= ~FLASH_CTLR_FTER;

		if (rc < 0) {
			break;
		}

		offset += FLASH_ERASE_SIZE;
		len -= FLASH_ERASE_SIZE;
	}

out:
	flash_wch_lock(cfg->regs);
	return rc;
}

static int flash_wch_erase(const struct device *dev, off_t offset, size_t len)
{
	struct flash_wch_data *dev_data = dev->data;
	unsigned int key;
	int rc;

	/* Erase must be aligned to FLASH_ERASE_SIZE */
	if ((offset % FLASH_ERASE_SIZE != 0) || (len % FLASH_ERASE_SIZE != 0)) {
		LOG_ERR("Erase offset/len must be %d-byte aligned", FLASH_ERASE_SIZE);
		return -EINVAL;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);
	key = irq_lock();

	rc = flash_wch_erase_ram(dev, offset, len);

	irq_unlock(key);
	k_sem_give(&dev_data->sem);

	return rc;
}

static const struct flash_parameters *
flash_wch_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_wch_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout flash_wch_pages_layout_node = {
	.pages_count = DT_REG_SIZE(DT_NODELABEL(flash0)) / FLASH_ERASE_SIZE,
	.pages_size = FLASH_ERASE_SIZE,
};

static void flash_wch_pages_layout(const struct device *dev,
				   const struct flash_pages_layout **layout,
				   size_t *layout_size)
{
	*layout = &flash_wch_pages_layout_node;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api flash_driver_api = {
	.read = flash_wch_read,
	.write = flash_wch_write,
	.erase = flash_wch_erase,
	.get_parameters = flash_wch_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_wch_pages_layout,
#endif
};

static int flash_wch_init(const struct device *dev)
{
	struct flash_wch_data *data = dev->data;

	k_sem_init(&data->sem, 1, 1);

	return 0;
}

#define FLASH_WCH_INIT(n)						\
	static struct flash_wch_data flash_wch_data_##n;		\
	static const struct flash_wch_config flash_wch_config_##n = {	\
		.regs = (FLASH_TypeDef *)DT_INST_REG_ADDR(n),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, flash_wch_init, NULL,			\
			      &flash_wch_data_##n,			\
			      &flash_wch_config_##n, POST_KERNEL,	\
			      CONFIG_FLASH_INIT_PRIORITY,		\
			      &flash_driver_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_WCH_INIT)
