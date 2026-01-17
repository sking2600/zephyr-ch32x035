/*
 * Copyright (c) 2024 Paul Wedeck <paulwedeck@gmail.com>
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WCH CH32 DMA Controller Driver
 *
 * This driver supports the DMA controller found on WCH CH32V/CH32L/CH32X series MCUs.
 * Each DMA controller instance (DMA1, DMA2, etc.) is handled as a separate device.
 * Channel indices are 0-based in Zephyr API but 1-based in hardware registers.
 */

#define DT_DRV_COMPAT wch_dma

#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/pm/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_ch32, CONFIG_DMA_LOG_LEVEL);

/*
 * ============================================================================
 * Register Definitions (self-contained, no external HAL dependency)
 * ============================================================================
 *
 * DMA Controller Register Layout (per WCH Reference Manuals):
 *   Offset 0x00: INTFR  - Interrupt Flag Register (read-only)
 *   Offset 0x04: INTFCR - Interrupt Flag Clear Register (write-only)
 *   Offset 0x08: Channel 1 registers start
 *
 * Each Channel has:
 *   Offset +0x00: CFGR  - Configuration Register (20 bytes stride per channel)
 *   Offset +0x04: CNTR  - Count Register
 *   Offset +0x08: PADDR - Peripheral Address Register
 *   Offset +0x0C: MADDR - Memory Address Register
 *   (0x10 reserved, but stride is 0x14 for some chips, 0x14 for others)
 *
 * Note: Channel stride is 0x14 (20 bytes) on most CH32 parts.
 */

/* DMA Controller base register offsets */
#define DMA_INTFR_OFFSET   0x00
#define DMA_INTFCR_OFFSET  0x04
#define DMA_CHAN_OFFSET    0x08

/* Per-channel register offsets (from channel base) */
#define DMA_CHAN_CFGR_OFFSET   0x00
#define DMA_CHAN_CNTR_OFFSET   0x04
#define DMA_CHAN_PADDR_OFFSET  0x08
#define DMA_CHAN_MADDR_OFFSET  0x0C
#define DMA_CHAN_STRIDE        0x14  /* 20 bytes between channels */

/* CFGR (Configuration Register) bit definitions */
#define DMA_CFGR_EN       BIT(0)   /* Channel enable */
#define DMA_CFGR_TCIE     BIT(1)   /* Transfer complete interrupt enable */
#define DMA_CFGR_HTIE     BIT(2)   /* Half transfer interrupt enable */
#define DMA_CFGR_TEIE     BIT(3)   /* Transfer error interrupt enable */
#define DMA_CFGR_DIR      BIT(4)   /* Direction: 0=P->M, 1=M->P */
#define DMA_CFGR_CIRC     BIT(5)   /* Circular mode */
#define DMA_CFGR_PINC     BIT(6)   /* Peripheral increment mode */
#define DMA_CFGR_MINC     BIT(7)   /* Memory increment mode */
#define DMA_CFGR_PSIZE_SHIFT  8    /* Peripheral data size [9:8] */
#define DMA_CFGR_PSIZE_MASK   (0x3 << DMA_CFGR_PSIZE_SHIFT)
#define DMA_CFGR_MSIZE_SHIFT  10   /* Memory data size [11:10] */
#define DMA_CFGR_MSIZE_MASK   (0x3 << DMA_CFGR_MSIZE_SHIFT)
#define DMA_CFGR_PL_SHIFT     12   /* Priority level [13:12] */
#define DMA_CFGR_PL_MASK      (0x3 << DMA_CFGR_PL_SHIFT)
#define DMA_CFGR_MEM2MEM  BIT(14)  /* Memory-to-memory mode */

/* INTFR/INTFCR bit definitions (4 bits per channel, starting at bit 0 for ch1) */
#define DMA_ISR_GIF   BIT(0)  /* Global interrupt flag */
#define DMA_ISR_TCIF  BIT(1)  /* Transfer complete flag */
#define DMA_ISR_HTIF  BIT(2)  /* Half transfer flag */
#define DMA_ISR_TEIF  BIT(3)  /* Transfer error flag */
#define DMA_ISR_MASK  0x0F    /* All flags for one channel */

/* Shift to get flags for channel n (0-indexed) */
#define DMA_ISR_SHIFT(ch)  ((ch) * 4)

/* Maximum channels per controller on any CH32 variant */
#define DMA_CH32_MAX_CHANNELS  8

/* Maximum transfer count (16-bit CNTR register) */
#define DMA_CH32_MAX_XFER_COUNT  0xFFFF

/*
 * ============================================================================
 * Driver Data Structures
 * ============================================================================
 */

struct dma_ch32_channel {
	dma_callback_t callback;
	void *user_data;
	uint32_t direction;  /* Cached direction for reload */
};

struct dma_ch32_data {
	struct dma_context ctx;
	struct dma_ch32_channel channels[DMA_CH32_MAX_CHANNELS];
};

struct dma_ch32_config {
	uintptr_t base;              /* DMA controller base address */
	uint8_t num_channels;        /* Number of channels on this controller */
	const struct device *clk_dev;
	uint32_t clk_id;
	void (*irq_configure)(const struct device *dev);
};

/*
 * ============================================================================
 * Register Access Helpers
 * ============================================================================
 */

static inline uint32_t dma_ch32_read_intfr(const struct dma_ch32_config *cfg)
{
	return sys_read32(cfg->base + DMA_INTFR_OFFSET);
}

static inline void dma_ch32_write_intfcr(const struct dma_ch32_config *cfg, uint32_t val)
{
	sys_write32(val, cfg->base + DMA_INTFCR_OFFSET);
}

static inline uintptr_t dma_ch32_chan_base(const struct dma_ch32_config *cfg, uint32_t ch)
{
	return cfg->base + DMA_CHAN_OFFSET + (ch * DMA_CHAN_STRIDE);
}

static inline uint32_t dma_ch32_read_cfgr(const struct dma_ch32_config *cfg, uint32_t ch)
{
	return sys_read32(dma_ch32_chan_base(cfg, ch) + DMA_CHAN_CFGR_OFFSET);
}

static inline void dma_ch32_write_cfgr(const struct dma_ch32_config *cfg, uint32_t ch,
				       uint32_t val)
{
	sys_write32(val, dma_ch32_chan_base(cfg, ch) + DMA_CHAN_CFGR_OFFSET);
}

static inline uint32_t dma_ch32_read_cntr(const struct dma_ch32_config *cfg, uint32_t ch)
{
	return sys_read32(dma_ch32_chan_base(cfg, ch) + DMA_CHAN_CNTR_OFFSET);
}

static inline void dma_ch32_write_cntr(const struct dma_ch32_config *cfg, uint32_t ch,
				       uint32_t val)
{
	sys_write32(val, dma_ch32_chan_base(cfg, ch) + DMA_CHAN_CNTR_OFFSET);
}

static inline void dma_ch32_write_paddr(const struct dma_ch32_config *cfg, uint32_t ch,
					uint32_t val)
{
	sys_write32(val, dma_ch32_chan_base(cfg, ch) + DMA_CHAN_PADDR_OFFSET);
}

static inline void dma_ch32_write_maddr(const struct dma_ch32_config *cfg, uint32_t ch,
					uint32_t val)
{
	sys_write32(val, dma_ch32_chan_base(cfg, ch) + DMA_CHAN_MADDR_OFFSET);
}

static inline uint32_t dma_ch32_read_paddr(const struct dma_ch32_config *cfg, uint32_t ch)
{
	return sys_read32(dma_ch32_chan_base(cfg, ch) + DMA_CHAN_PADDR_OFFSET);
}

static inline uint32_t dma_ch32_read_maddr(const struct dma_ch32_config *cfg, uint32_t ch)
{
	return sys_read32(dma_ch32_chan_base(cfg, ch) + DMA_CHAN_MADDR_OFFSET);
}

/*
 * ============================================================================
 * Helper Functions
 * ============================================================================
 */

/* Convert byte width to PSIZE/MSIZE encoding (0=8bit, 1=16bit, 2=32bit) */
static inline uint32_t dma_ch32_width_to_reg(uint32_t bytes)
{
	switch (bytes) {
	case 1:
		return 0;
	case 2:
		return 1;
	case 4:
		return 2;
	default:
		return 0; /* Default to byte access */
	}
}

/* Check if channel is busy (enabled and not complete) */
static bool dma_ch32_is_busy(const struct device *dev, uint32_t ch)
{
	const struct dma_ch32_config *cfg = dev->config;
	uint32_t cfgr = dma_ch32_read_cfgr(cfg, ch);
	uint32_t intfr = dma_ch32_read_intfr(cfg);
	uint32_t tc_flag = DMA_ISR_TCIF << DMA_ISR_SHIFT(ch);

	/* Busy if enabled AND transfer not complete */
	return (cfgr & DMA_CFGR_EN) && !(intfr & tc_flag);
}

/* Clear all interrupt flags for a channel */
static void dma_ch32_clear_flags(const struct dma_ch32_config *cfg, uint32_t ch)
{
	dma_ch32_write_intfcr(cfg, DMA_ISR_MASK << DMA_ISR_SHIFT(ch));
}

/*
 * ============================================================================
 * DMA API Implementation
 * ============================================================================
 */

static int dma_ch32_configure(const struct device *dev, uint32_t ch,
			      struct dma_config *config)
{
	const struct dma_ch32_config *cfg = dev->config;
	struct dma_ch32_data *data = dev->data;
	struct dma_block_config *block = config->head_block;
	uint32_t cfgr = 0;
	uint32_t paddr, maddr;
	uint32_t xfer_count;
	unsigned int key;

	/* Validate channel */
	if (ch >= cfg->num_channels) {
		LOG_ERR("Invalid channel %u (max %u)", ch, cfg->num_channels - 1);
		return -EINVAL;
	}

	/* Only single-block transfers supported */
	if (config->block_count != 1) {
		LOG_ERR("Only single-block transfers supported");
		return -ENOTSUP;
	}

	if (block == NULL) {
		LOG_ERR("No block config provided");
		return -EINVAL;
	}

	/* Validate block size */
	if (block->block_size == 0 || block->block_size > DMA_CH32_MAX_XFER_COUNT) {
		LOG_ERR("Invalid block size %u", block->block_size);
		return -EINVAL;
	}

	/* Scatter/gather not supported */
	if (block->source_gather_en || block->dest_scatter_en) {
		LOG_ERR("Scatter/gather not supported");
		return -ENOTSUP;
	}

	/* Reload not supported */
	if (block->source_reload_en || block->dest_reload_en) {
		LOG_ERR("Source/dest reload not supported");
		return -ENOTSUP;
	}

	/* Decrement mode not supported by hardware */
	if (block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT ||
	    block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		LOG_ERR("Address decrement not supported");
		return -ENOTSUP;
	}

	/* Priority must be 0-3 */
	if (config->channel_priority > 3) {
		LOG_ERR("Invalid priority %u (max 3)", config->channel_priority);
		return -EINVAL;
	}

	/* Configure direction and addresses */
	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
		cfgr |= DMA_CFGR_MEM2MEM;
		paddr = block->source_address;
		maddr = block->dest_address;
		/* M2M always reads from PADDR (source), writes to MADDR (dest) */
		if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			cfgr |= DMA_CFGR_PINC;
		}
		if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			cfgr |= DMA_CFGR_MINC;
		}
		/* Cyclic not supported for M2M */
		if (config->cyclic) {
			LOG_ERR("Cyclic not supported for M2M");
			return -ENOTSUP;
		}
		xfer_count = block->block_size / config->source_data_size;
		break;

	case MEMORY_TO_PERIPHERAL:
		cfgr |= DMA_CFGR_DIR;  /* M->P direction */
		maddr = block->source_address;
		paddr = block->dest_address;
		if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			cfgr |= DMA_CFGR_MINC;
		}
		if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			cfgr |= DMA_CFGR_PINC;
		}
		xfer_count = block->block_size / config->source_data_size;
		break;

	case PERIPHERAL_TO_MEMORY:
		/* DIR=0: P->M (default) */
		paddr = block->source_address;
		maddr = block->dest_address;
		if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			cfgr |= DMA_CFGR_PINC;
		}
		if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			cfgr |= DMA_CFGR_MINC;
		}
		xfer_count = block->block_size / config->dest_data_size;
		break;

	default:
		LOG_ERR("Unsupported direction %u", config->channel_direction);
		return -ENOTSUP;
	}

	/* Configure data sizes */
	cfgr |= dma_ch32_width_to_reg(config->source_data_size) << DMA_CFGR_PSIZE_SHIFT;
	cfgr |= dma_ch32_width_to_reg(config->dest_data_size) << DMA_CFGR_MSIZE_SHIFT;

	/* Priority */
	cfgr |= (config->channel_priority << DMA_CFGR_PL_SHIFT) & DMA_CFGR_PL_MASK;

	/* Circular mode */
	if (config->cyclic) {
		cfgr |= DMA_CFGR_CIRC;
	}

	/* Configure interrupts */
	if (config->dma_callback != NULL) {
		cfgr |= DMA_CFGR_TCIE;  /* Always enable TC interrupt if callback set */

		if (!config->error_callback_dis) {
			cfgr |= DMA_CFGR_TEIE;
		}

		if (config->complete_callback_en) {
			cfgr |= DMA_CFGR_HTIE;  /* Half-transfer for partial completion */
		}
	}

	/* Apply configuration with interrupts locked */
	key = irq_lock();

	if (dma_ch32_is_busy(dev, ch)) {
		irq_unlock(key);
		LOG_ERR("Channel %u is busy", ch);
		return -EBUSY;
	}

	/* Disable channel first */
	dma_ch32_write_cfgr(cfg, ch, 0);

	/* Clear any pending flags */
	dma_ch32_clear_flags(cfg, ch);

	/* Store callback info */
	data->channels[ch].callback = config->dma_callback;
	data->channels[ch].user_data = config->user_data;
	data->channels[ch].direction = config->channel_direction;

	/* Program registers (order matters: addresses before config) */
	dma_ch32_write_paddr(cfg, ch, paddr);
	dma_ch32_write_maddr(cfg, ch, maddr);
	dma_ch32_write_cntr(cfg, ch, xfer_count);
	dma_ch32_write_cfgr(cfg, ch, cfgr);

	irq_unlock(key);

	LOG_DBG("Ch%u configured: dir=%u, paddr=0x%08x, maddr=0x%08x, cnt=%u, cfgr=0x%04x",
		ch, config->channel_direction, paddr, maddr, xfer_count, cfgr);

	return 0;
}

static int dma_ch32_reload(const struct device *dev, uint32_t ch,
			   uint32_t src, uint32_t dst, size_t size)
{
	const struct dma_ch32_config *cfg = dev->config;
	struct dma_ch32_data *data = dev->data;
	unsigned int key;

	if (ch >= cfg->num_channels) {
		return -EINVAL;
	}

	key = irq_lock();

	if (dma_ch32_is_busy(dev, ch)) {
		irq_unlock(key);
		return -EBUSY;
	}

	/* Clear flags */
	dma_ch32_clear_flags(cfg, ch);

	/* Update addresses based on direction */
	if (data->channels[ch].direction == MEMORY_TO_PERIPHERAL) {
		dma_ch32_write_maddr(cfg, ch, src);
		dma_ch32_write_paddr(cfg, ch, dst);
	} else {
		dma_ch32_write_paddr(cfg, ch, src);
		dma_ch32_write_maddr(cfg, ch, dst);
	}
	dma_ch32_write_cntr(cfg, ch, size);

	irq_unlock(key);

	return 0;
}

static int dma_ch32_start(const struct device *dev, uint32_t ch)
{
	const struct dma_ch32_config *cfg = dev->config;
	unsigned int key;
	uint32_t cfgr;

	if (ch >= cfg->num_channels) {
		return -EINVAL;
	}

	key = irq_lock();

	/* Clear any pending flags before starting */
	dma_ch32_clear_flags(cfg, ch);

	/* Enable channel */
	cfgr = dma_ch32_read_cfgr(cfg, ch);
	cfgr |= DMA_CFGR_EN;
	dma_ch32_write_cfgr(cfg, ch, cfgr);

	irq_unlock(key);

	LOG_DBG("Ch%u started", ch);
	return 0;
}

static int dma_ch32_stop(const struct device *dev, uint32_t ch)
{
	const struct dma_ch32_config *cfg = dev->config;
	unsigned int key;
	uint32_t cfgr;

	if (ch >= cfg->num_channels) {
		return -EINVAL;
	}

	key = irq_lock();

	cfgr = dma_ch32_read_cfgr(cfg, ch);
	cfgr &= ~DMA_CFGR_EN;
	dma_ch32_write_cfgr(cfg, ch, cfgr);

	irq_unlock(key);

	LOG_DBG("Ch%u stopped", ch);
	return 0;
}

static int dma_ch32_suspend(const struct device *dev, uint32_t ch)
{
	const struct dma_ch32_config *cfg = dev->config;
	unsigned int key;
	uint32_t cfgr;

	if (ch >= cfg->num_channels) {
		return -EINVAL;
	}

	key = irq_lock();

	cfgr = dma_ch32_read_cfgr(cfg, ch);
	if (!(cfgr & DMA_CFGR_EN)) {
		irq_unlock(key);
		return -EINVAL;  /* Not running */
	}

	cfgr &= ~DMA_CFGR_EN;
	dma_ch32_write_cfgr(cfg, ch, cfgr);

	irq_unlock(key);
	return 0;
}

static int dma_ch32_resume(const struct device *dev, uint32_t ch)
{
	const struct dma_ch32_config *cfg = dev->config;
	unsigned int key;
	uint32_t cfgr;

	if (ch >= cfg->num_channels) {
		return -EINVAL;
	}

	key = irq_lock();

	cfgr = dma_ch32_read_cfgr(cfg, ch);
	if (cfgr & DMA_CFGR_EN) {
		irq_unlock(key);
		return -EINVAL;  /* Already running */
	}

	cfgr |= DMA_CFGR_EN;
	dma_ch32_write_cfgr(cfg, ch, cfgr);

	irq_unlock(key);
	return 0;
}

static int dma_ch32_get_status(const struct device *dev, uint32_t ch,
			       struct dma_status *stat)
{
	const struct dma_ch32_config *cfg = dev->config;
	uint32_t cfgr;
	unsigned int key;

	if (ch >= cfg->num_channels) {
		return -EINVAL;
	}

	key = irq_lock();

	cfgr = dma_ch32_read_cfgr(cfg, ch);

	stat->busy = dma_ch32_is_busy(dev, ch);
	stat->pending_length = dma_ch32_read_cntr(cfg, ch);

	if (cfgr & DMA_CFGR_MEM2MEM) {
		stat->dir = MEMORY_TO_MEMORY;
	} else if (cfgr & DMA_CFGR_DIR) {
		stat->dir = MEMORY_TO_PERIPHERAL;
	} else {
		stat->dir = PERIPHERAL_TO_MEMORY;
	}

	/* Report current addresses */
	if (stat->dir == MEMORY_TO_PERIPHERAL) {
		stat->read_position = dma_ch32_read_maddr(cfg, ch);
		stat->write_position = dma_ch32_read_paddr(cfg, ch);
	} else {
		stat->read_position = dma_ch32_read_paddr(cfg, ch);
		stat->write_position = dma_ch32_read_maddr(cfg, ch);
	}

	irq_unlock(key);
	return 0;
}

static int dma_ch32_get_attribute(const struct device *dev, uint32_t type,
				  uint32_t *value)
{
	ARG_UNUSED(dev);

	switch (type) {
	case DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT:
	case DMA_ATTR_BUFFER_SIZE_ALIGNMENT:
	case DMA_ATTR_COPY_ALIGNMENT:
		*value = 1;  /* Byte-aligned */
		return 0;
	case DMA_ATTR_MAX_BLOCK_COUNT:
		*value = 1;  /* Single block only */
		return 0;
	default:
		return -EINVAL;
	}
}

static bool dma_ch32_chan_filter(const struct device *dev, int ch, void *filter_param)
{
	const struct dma_ch32_config *cfg = dev->config;
	uint32_t requested_ch;

	if (filter_param == NULL) {
		return true;  /* Any channel OK */
	}

	requested_ch = *(uint32_t *)filter_param;

	/* Check if requested channel matches and is valid */
	return (ch == requested_ch) && (ch < cfg->num_channels);
}

/*
 * ============================================================================
 * Interrupt Handler
 * ============================================================================
 */

static void dma_ch32_isr(const struct device *dev, uint32_t ch)
{
	const struct dma_ch32_config *cfg = dev->config;
	struct dma_ch32_data *data = dev->data;
	uint32_t intfr;
	uint32_t flags;
	int status = 0;

	intfr = dma_ch32_read_intfr(cfg);
	flags = (intfr >> DMA_ISR_SHIFT(ch)) & DMA_ISR_MASK;

	/* Clear the flags we're handling */
	dma_ch32_write_intfcr(cfg, flags << DMA_ISR_SHIFT(ch));

	/* Determine callback status */
	if (flags & DMA_ISR_TEIF) {
		LOG_ERR("DMA ch%u transfer error", ch);
		status = -EIO;
		/* Disable channel on error */
		dma_ch32_write_cfgr(cfg, ch,
				    dma_ch32_read_cfgr(cfg, ch) & ~DMA_CFGR_EN);
	} else if (flags & DMA_ISR_TCIF) {
		status = DMA_STATUS_COMPLETE;
		/* Disable channel on completion (unless circular) */
		uint32_t cfgr = dma_ch32_read_cfgr(cfg, ch);
		if (!(cfgr & DMA_CFGR_CIRC)) {
			dma_ch32_write_cfgr(cfg, ch, cfgr & ~DMA_CFGR_EN);
		}
	} else if (flags & DMA_ISR_HTIF) {
		status = DMA_STATUS_BLOCK;
	} else {
		/* No recognized flag - spurious interrupt */
		return;
	}

	/* Invoke callback if registered */
	if (data->channels[ch].callback != NULL) {
		data->channels[ch].callback(dev, data->channels[ch].user_data,
					    ch, status);
	}
    

}

/*
 * ============================================================================
 * Device Initialization
 * ============================================================================
 */




static int dma_ch32_init(const struct device *dev)
{
	const struct dma_ch32_config *cfg = dev->config;
	int ret;

	LOG_INF("DMA: Initialized controller @ 0x%lx with %u channels",
		cfg->base, cfg->num_channels);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int dma_ch32_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct dma_ch32_config *cfg = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		return clock_control_off(cfg->clk_dev,
					 (clock_control_subsys_t)(uintptr_t)cfg->clk_id);
	case PM_DEVICE_ACTION_RESUME:
		return clock_control_on(cfg->clk_dev,
					(clock_control_subsys_t)(uintptr_t)cfg->clk_id);
	default:
		return -ENOTSUP;
	}
}
#endif /* CONFIG_PM_DEVICE */

/*
 * ============================================================================
 * Driver API
 * ============================================================================
 */

static DEVICE_API(dma, dma_ch32_api) = {
	.config = dma_ch32_configure,
	.reload = dma_ch32_reload,
	.start = dma_ch32_start,
	.stop = dma_ch32_stop,
	.suspend = dma_ch32_suspend,
	.resume = dma_ch32_resume,
	.get_status = dma_ch32_get_status,
	.get_attribute = dma_ch32_get_attribute,
	.chan_filter = dma_ch32_chan_filter,
};

/*
 * ============================================================================
 * Device Instantiation Macros
 * ============================================================================
 */

/* Generate ISR wrapper for each channel */
#define DMA_CH32_IRQ_HANDLER(inst, ch)                                         \
	static void dma_ch32_##inst##_isr_##ch(const struct device *dev)       \
	{                                                                      \
		dma_ch32_isr(dev, ch);                                         \
	}

/* Connect and enable one IRQ */
#define DMA_CH32_IRQ_CONNECT(inst, idx)                                        \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, idx, irq),                        \
		    DT_INST_IRQ_BY_IDX(inst, idx, priority),                   \
		    dma_ch32_##inst##_isr_##idx,                               \
		    DEVICE_DT_INST_GET(inst), 0);                              \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, idx, irq));

/* Generate IRQ handlers for all channels (up to 8) */
#define DMA_CH32_DEFINE_IRQS(inst)                                             \
	DMA_CH32_IRQ_HANDLER(inst, 0)                                          \
	DMA_CH32_IRQ_HANDLER(inst, 1)                                          \
	DMA_CH32_IRQ_HANDLER(inst, 2)                                          \
	DMA_CH32_IRQ_HANDLER(inst, 3)                                          \
	DMA_CH32_IRQ_HANDLER(inst, 4)                                          \
	DMA_CH32_IRQ_HANDLER(inst, 5)                                          \
	DMA_CH32_IRQ_HANDLER(inst, 6)                                          \
	DMA_CH32_IRQ_HANDLER(inst, 7)

/* IRQ configuration function - connects only the IRQs defined in DTS */
#define DMA_CH32_IRQ_CONFIG_FUNC(inst)                                         \
	static void dma_ch32_##inst##_irq_config(const struct device *dev)     \
	{                                                                      \
		ARG_UNUSED(dev);                                               \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 0),                       \
			   (DMA_CH32_IRQ_CONNECT(inst, 0)))                    \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 1),                       \
			   (DMA_CH32_IRQ_CONNECT(inst, 1)))                    \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 2),                       \
			   (DMA_CH32_IRQ_CONNECT(inst, 2)))                    \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 3),                       \
			   (DMA_CH32_IRQ_CONNECT(inst, 3)))                    \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 4),                       \
			   (DMA_CH32_IRQ_CONNECT(inst, 4)))                    \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 5),                       \
			   (DMA_CH32_IRQ_CONNECT(inst, 5)))                    \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 6),                       \
			   (DMA_CH32_IRQ_CONNECT(inst, 6)))                    \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 7),                       \
			   (DMA_CH32_IRQ_CONNECT(inst, 7)))                    \
	}

/* Main device definition macro */
#define DMA_CH32_INIT(inst)                                                    \
	DMA_CH32_DEFINE_IRQS(inst)                                             \
	DMA_CH32_IRQ_CONFIG_FUNC(inst)                                         \
                                                                               \
	static ATOMIC_DEFINE(dma_ch32_##inst##_atomic,                         \
			     DT_INST_PROP(inst, dma_channels));                \
                                                                               \
	static struct dma_ch32_data dma_ch32_##inst##_data = {                 \
		.ctx = {                                                       \
			.magic = DMA_MAGIC,                                    \
			.atomic = dma_ch32_##inst##_atomic,                    \
			.dma_channels = DT_INST_PROP(inst, dma_channels),      \
		},                                                             \
	};                                                                     \
                                                                               \
	static const struct dma_ch32_config dma_ch32_##inst##_config = {       \
		.base = DT_INST_REG_ADDR(inst),                                \
		.num_channels = DT_INST_PROP(inst, dma_channels),              \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),           \
		.clk_id = DT_INST_CLOCKS_CELL(inst, id),                       \
		.irq_configure = dma_ch32_##inst##_irq_config,                 \
	};                                                                     \
                                                                               \
	PM_DEVICE_DT_INST_DEFINE(inst, dma_ch32_pm_action);                    \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      dma_ch32_init,                                   \
			      PM_DEVICE_DT_INST_GET(inst),                     \
			      &dma_ch32_##inst##_data,                         \
			      &dma_ch32_##inst##_config,                       \
			      PRE_KERNEL_1,                                    \
			      CONFIG_DMA_INIT_PRIORITY,                        \
			      &dma_ch32_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_CH32_INIT)
