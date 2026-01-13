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

LOG_MODULE_REGISTER(flash_wch, CONFIG_FLASH_LOG_LEVEL);

/* FLASH Keys */
#define FLASH_KEY1                 ((uint32_t)0x45670123)
#define FLASH_KEY2                 ((uint32_t)0xCDEF89AB)

#define FLASH_TIMEOUT_COUNT        0x100000

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

static inline void flash_wch_unlock(FLASH_TypeDef *regs)
{
	regs->KEYR = FLASH_KEY1;
	regs->KEYR = FLASH_KEY2;
	regs->MODEKEYR = FLASH_KEY1;
	regs->MODEKEYR = FLASH_KEY2;
}

static inline void flash_wch_lock(FLASH_TypeDef *regs)
{
	regs->CTLR |= FLASH_CTLR_LOCK | FLASH_CTLR_FLOCK;
}

static int flash_wch_wait_bsy(FLASH_TypeDef *regs)
{
	int i;

	for (i = 0; i < FLASH_TIMEOUT_COUNT; i++) {
		if (!(regs->STATR & FLASH_STATR_BSY)) {
			return 0;
		}
	}
	return -ETIMEDOUT;
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

static int flash_wch_write_page(FLASH_TypeDef *regs, off_t pg_addr, const uint32_t *data, size_t count)
{
	int rc;
	size_t i;

	regs->CTLR |= FLASH_CTLR_FTPG;
	regs->CTLR |= FLASH_CTLR_BUFRST;
	rc = flash_wch_wait_bsy(regs);
	if (rc < 0) return rc;

	/* Load words into buffer */
	for (i = 0; i < count; i++) {
		volatile uint32_t *addr = (volatile uint32_t *)(FLASH_BASE + pg_addr + (i * 4));
		*addr = data[i];
		regs->CTLR |= FLASH_CTLR_BUFLOAD;
		rc = flash_wch_wait_bsy(regs);
		if (rc < 0) return rc;
	}

	/* Trigger programming */
	regs->ADDR = FLASH_BASE + pg_addr;
	regs->CTLR |= FLASH_CTLR_STRT;
	rc = flash_wch_wait_bsy(regs);
	
	regs->CTLR &= ~FLASH_CTLR_FTPG;

	return rc;
}

static int flash_wch_write(const struct device *dev, off_t offset,
			   const void *data, size_t len)
{
	struct flash_wch_data *dev_data = dev->data;
	const struct flash_wch_config *cfg = dev->config;
	int rc = 0;
	const uint8_t *src = data;

	if ((offset % 4 != 0) || (len % 4 != 0)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);
	flash_wch_unlock(cfg->regs);

	flash_wch_wait_bsy(cfg->regs);

	/* Write in 256-byte page chunks */
	while (len > 0) {
		off_t pg_offset = offset & (256 - 1);
		size_t write_len = 256 - pg_offset;
		if (write_len > len) {
			write_len = len;
		}

		/* Since we can't easily do partial page updates if we need to preserve other data in the same page
		   without reading it back, we rely on the fact that BUF_RST sets latches to '1'.
		   If NVS writes sequentially, it's fine.
		   The loop below only writes the 'new' data. The other latches in the page buffer
		   should effectively be No-Ops if they are '1'.
		   However, if write_len is small (e.g. 4 bytes), we still need to initiate the whole Page Program sequence.
		*/
		
		/* We must pass strictly 32-bit words */
		rc = flash_wch_write_page(cfg->regs, offset, (const uint32_t *)src, write_len / 4);
		if (rc < 0) {
			break;
		}

		offset += write_len;
		src += write_len;
		len -= write_len;
	}

	flash_wch_lock(cfg->regs);
	k_sem_give(&dev_data->sem);

	return rc;
}

static int flash_wch_erase(const struct device *dev, off_t offset,
			   size_t len)
{
	struct flash_wch_data *dev_data = dev->data;
	const struct flash_wch_config *cfg = dev->config;
	int rc = 0;

	/* Supports 256-byte page erase */
	if ((offset % 256 != 0) || (len % 256 != 0)) {
		LOG_ERR("Erase offset/len must be 256-byte aligned");
		return -EINVAL;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);
	flash_wch_unlock(cfg->regs);

	flash_wch_wait_bsy(cfg->regs);

	while (len > 0) {
		cfg->regs->CTLR |= FLASH_CTLR_FTER;
		cfg->regs->ADDR = FLASH_BASE + offset;
		cfg->regs->CTLR |= FLASH_CTLR_STRT;
		
		rc = flash_wch_wait_bsy(cfg->regs);
		cfg->regs->CTLR &= ~FLASH_CTLR_FTER;

		if (rc < 0) {
			break;
		}

		offset += 256;
		len -= 256;
	}

	flash_wch_lock(cfg->regs);
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
	.pages_count = DT_REG_SIZE(DT_NODELABEL(flash0)) / 256,
	.pages_size = 256,
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
									\
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
