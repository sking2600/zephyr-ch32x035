/*
 * Copyright (c) 2024 WCH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_bbram

#include <zephyr/drivers/bbram.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <errno.h>

LOG_MODULE_REGISTER(bbram, CONFIG_BBRAM_LOG_LEVEL);

#define WCH_BKP_REG_BYTES 2
#define WCH_BKP_REG_STRIDE 4

/** Device config */
struct bbram_wch_config {
	uintptr_t base_addr;
	int size;
};

/* Helper to get register address */
/* Register index: 0 -> DATAR1 (offset 0), 1 -> DATAR2 (offset 4) */
#define WCH_BKP_REG_ADDR(base, idx) ((volatile uint16_t *)((base) + (idx) * WCH_BKP_REG_STRIDE))

static int bbram_wch_read(const struct device *dev, size_t offset, size_t size, uint8_t *data)
{
	const struct bbram_wch_config *config = dev->config;
	size_t remaining = size;
	size_t current_off = offset;
	uint8_t *out = data;

	if (size < 1 || offset + size > config->size) {
		return -EFAULT;
	}

	while (remaining > 0) {
		size_t reg_idx = current_off / WCH_BKP_REG_BYTES;
		size_t byte_in_reg = current_off % WCH_BKP_REG_BYTES;
		
		volatile uint16_t *reg = WCH_BKP_REG_ADDR(config->base_addr, reg_idx);
		uint16_t val = *reg;

		/* Number of bytes we can take from this register: 1 or 2 */
		size_t to_copy = MIN(WCH_BKP_REG_BYTES - byte_in_reg, remaining);

		if (to_copy == 1) {
			*out = (val >> (byte_in_reg * 8)) & 0xFF;
		} else {
			/* Copy 2 bytes (aligned read case or full reg read) */
			/* Assuming Little Endian */
			out[0] = val & 0xFF;
			out[1] = (val >> 8) & 0xFF; // Only if byte_in_reg=0 and to_copy=2
		}

		remaining -= to_copy;
		current_off += to_copy;
		out += to_copy;
	}

	return 0;
}

static int bbram_wch_write(const struct device *dev, size_t offset, size_t size,
			     const uint8_t *data)
{
	const struct bbram_wch_config *config = dev->config;
	size_t remaining = size;
	size_t current_off = offset;
	const uint8_t *in = data;

	if (size < 1 || offset + size > config->size) {
		return -EFAULT;
	}

	/* Store requires modifying registers. Read-Modify-Write needed for partial bytes? */
	/* Since we only support byte steps, yes. */

	/* Enable Write Access to Backup Domain */
	/* Bit DBP (Disable Backup Domain Protection) in PWR_CTLR */
	PWR->CTLR |= PWR_CTLR_DBP;

	while (remaining > 0) {
		size_t reg_idx = current_off / WCH_BKP_REG_BYTES;
		size_t byte_in_reg = current_off % WCH_BKP_REG_BYTES;
		
		volatile uint16_t *reg = WCH_BKP_REG_ADDR(config->base_addr, reg_idx);
		uint16_t val = *reg;
		uint16_t new_val = val;

		size_t to_copy = MIN(WCH_BKP_REG_BYTES - byte_in_reg, remaining);

		if (to_copy == 1) {
			/* Modify single byte */
			uint16_t mask = 0xFF << (byte_in_reg * 8);
			new_val &= ~mask;
			new_val |= (*in) << (byte_in_reg * 8);
		} else {
			/* Modify both bytes */
			new_val = in[0] | (in[1] << 8);
		}

		*reg = new_val;

		remaining -= to_copy;
		current_off += to_copy;
		in += to_copy;
	}
	
	/* Disable Write Access? Maybe leave enabled or disable to protect? */
	/* Zephyr driver usually leaves it or re-disables. Re-disabling is safer. */
	PWR->CTLR &= ~PWR_CTLR_DBP;

	return 0;
}

static int bbram_wch_get_size(const struct device *dev, size_t *size)
{
	const struct bbram_wch_config *config = dev->config;

	*size = config->size;

	return 0;
}

static const struct bbram_driver_api bbram_wch_driver_api = {
	.read = bbram_wch_read,
	.write = bbram_wch_write,
	.get_size = bbram_wch_get_size,
};

static int bbram_wch_init(const struct device *dev)
{
	/* Enable PWR and BKP clocks */
	/* RCC->APB1PCENR |= RCC_APB1Periph_PWR | RCC_APB1Periph_BKP; */
	RCC->APB1PCENR |= (1 << 28) | (1 << 27); /* PWR(28) and BKP(27) bits */
	
	/* Or use proper macros */
	/* check soc.h for RCC_APB1PCENR_PWREN / BKPEN */
	
	return 0;
}

#define BBRAM_INIT(inst)                                                                           \
	static const struct bbram_wch_config bbram_cfg_##inst = {                                  \
		.base_addr = DT_INST_REG_ADDR(inst),                                               \
		.size = DT_INST_PROP(inst, size),                                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, bbram_wch_init, NULL, NULL, &bbram_cfg_##inst, POST_KERNEL,    \
			      CONFIG_BBRAM_INIT_PRIORITY, &bbram_wch_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BBRAM_INIT);

