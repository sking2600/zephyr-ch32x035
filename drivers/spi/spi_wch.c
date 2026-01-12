/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT wch_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_wch);

#include "spi_context.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

#include <hal_ch32fun.h>

#include <zephyr/drivers/dma.h>

#define SPI_CTLR1_LSBFIRST BIT(7)
#define SPI_CTLR1_BR_POS   3

struct spi_wch_config {
	SPI_TypeDef *regs;
	const struct pinctrl_dev_config *pin_cfg;
	const struct device *clk_dev;
	uint8_t clock_id;
#ifdef CONFIG_SPI_WCH_DMA
	const struct device *dma_dev;
	uint8_t dma_tx_chan;
	uint8_t dma_rx_chan;
#endif
};

struct spi_wch_data {
	struct spi_context ctx;
#ifdef CONFIG_SPI_WCH_DMA
	struct k_sem dma_sem;
	int dma_status;
	uint8_t dummy_tx;
	uint8_t dummy_rx;
#endif
};

#ifdef CONFIG_SPI_WCH_DMA
static void spi_wch_dma_callback(const struct device *dev, void *user_data,
				 uint32_t channel, int status)
{
	const struct device *spi_dev = user_data;
	struct spi_wch_data *data = spi_dev->data;

	if (status < 0) {
		data->dma_status = status;
	}
	k_sem_give(&data->dma_sem);
}

static int spi_wch_dma_tx(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct spi_wch_config *cfg = dev->config;
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_callback = spi_wch_dma_callback;
	dma_cfg.user_data = (void *)dev;

	dma_blk.block_size = len;
	dma_blk.dest_address = (uint32_t)&cfg->regs->DATAR;
	if (buf != NULL) {
		dma_blk.source_address = (uint32_t)buf;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		struct spi_wch_data *data = dev->data;
		data->dummy_tx = 0;
		dma_blk.source_address = (uint32_t)&data->dummy_tx;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	return dma_config(cfg->dma_dev, cfg->dma_tx_chan, &dma_cfg);
}

static int spi_wch_dma_rx(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct spi_wch_config *cfg = dev->config;
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_callback = spi_wch_dma_callback;
	dma_cfg.user_data = (void *)dev;

	dma_blk.block_size = len;
	dma_blk.source_address = (uint32_t)&cfg->regs->DATAR;
	if (buf != NULL) {
		dma_blk.dest_address = (uint32_t)buf;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		struct spi_wch_data *data = dev->data;
		dma_blk.dest_address = (uint32_t)&data->dummy_rx;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	return dma_config(cfg->dma_dev, cfg->dma_rx_chan, &dma_cfg);
}
#endif

static uint8_t spi_wch_get_br(uint32_t target_clock_ratio)
{
	uint8_t prescaler;
	int prescaler_val = 2;

	for (prescaler = 0; prescaler < 7; prescaler++) {
		if (prescaler_val > target_clock_ratio) {
			break;
		}
		prescaler_val *= 2;
	}

	return prescaler;
}

static int spi_wch_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_wch_config *cfg = dev->config;
	struct spi_wch_data *data = dev->data;
	SPI_TypeDef *regs = cfg->regs;
	int err;
	uint32_t clock_rate;
	clock_control_subsys_t clk_sys;
	int8_t prescaler;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if ((config->operation & SPI_HALF_DUPLEX) != 0U) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_MODE_LOOP) != 0U) {
		LOG_ERR("Loop mode not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Frame size != 8 bits not supported");
		return -ENOTSUP;
	}

	regs->CTLR1 = 0;
	regs->CTLR2 = 0;
	regs->STATR = 0;

	if (spi_cs_is_gpio(config)) {
		/* When using soft NSS, SSI must be set high */
		regs->CTLR1 |= SPI_CTLR1_SSM | SPI_CTLR1_SSI;
	} else {
		regs->CTLR2 |= SPI_CTLR2_SSOE;
	}

	regs->CTLR1 |= SPI_CTLR1_MSTR;

	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		regs->CTLR1 |= SPI_CTLR1_LSBFIRST;
	}

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		regs->CTLR1 |= SPI_CTLR1_CPOL;
	}

	if ((config->operation & SPI_MODE_CPHA) != 0U) {
		regs->CTLR1 |= SPI_CTLR1_CPHA;
	}

	clk_sys = (clock_control_subsys_t)(uintptr_t)cfg->clock_id;
	err = clock_control_get_rate(cfg->clk_dev, clk_sys, &clock_rate);
	if (err != 0) {
		return err;
	}

	/* Approximate clock rate given ratios available */
	prescaler = spi_wch_get_br(clock_rate / config->frequency);
#if CONFIG_SPI_LOG_LEVEL >= LOG_LEVEL_INF
	uint32_t j = 2;

	for (int i = 0; i < prescaler; i++) {
		j = j * 2;
	}
	LOG_INF("Selected divider %d, value %d, results in %d frequency", j, prescaler,
		clock_rate / j);
#endif
	regs->CTLR1 |= prescaler << SPI_CTLR1_BR_POS;

	data->ctx.config = config;

	return 0;
}

static int spi_wch_transceive(const struct device *dev, const struct spi_config *config,
			      const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	const struct spi_wch_config *cfg = dev->config;
	struct spi_wch_data *data = dev->data;
	SPI_TypeDef *regs = cfg->regs;
	int err;
	uint8_t rx;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	err = spi_wch_configure(dev, config);
	if (err != 0) {
		goto done;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	/* Start SPI *AFTER* setting CS */
	regs->CTLR1 |= SPI_CTLR1_SPE;

#ifdef CONFIG_SPI_WCH_DMA
	if (cfg->dma_dev != NULL) {
		while (spi_context_total_tx_len(&data->ctx) > 0 ||
		       spi_context_total_rx_len(&data->ctx) > 0) {
			size_t len = spi_context_max_continuous_chunk(&data->ctx);

			data->dma_status = 0;
			k_sem_reset(&data->dma_sem);

			err = spi_wch_dma_tx(dev, data->ctx.tx_buf, len);
			if (err != 0) {
				goto done_dma;
			}
			err = spi_wch_dma_rx(dev, data->ctx.rx_buf, len);
			if (err != 0) {
				goto done_dma;
			}

			err = dma_start(cfg->dma_dev, cfg->dma_tx_chan);
			if (err != 0) {
				goto done_dma;
			}
			err = dma_start(cfg->dma_dev, cfg->dma_rx_chan);
			if (err != 0) {
				dma_stop(cfg->dma_dev, cfg->dma_tx_chan);
				goto done_dma;
			}

			regs->CTLR2 |= SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx;

			/* Wait for both TX and RX to complete */
			k_sem_take(&data->dma_sem, K_FOREVER);
			k_sem_take(&data->dma_sem, K_FOREVER);

			regs->CTLR2 &= ~(SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx);

			if (data->dma_status < 0) {
				err = data->dma_status;
				goto done_dma;
			}

			spi_context_update_tx(&data->ctx, 1, len);
			spi_context_update_rx(&data->ctx, 1, len);
		}
		goto done;

done_dma:
		regs->CTLR2 &= ~(SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx);
		goto done;
	}
#endif

	while (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
		if (spi_context_tx_buf_on(&data->ctx)) {
			while ((regs->STATR & SPI_STATR_TXE) == 0U) {
			}
			regs->DATAR = *(uint8_t *)(data->ctx.tx_buf);
		} else {
			while ((regs->STATR & SPI_STATR_TXE) == 0U) {
			}
			regs->DATAR = 0;
		}
		spi_context_update_tx(&data->ctx, 1, 1);
		while ((regs->STATR & SPI_STATR_RXNE) == 0U) {
		}
		rx = regs->DATAR;
		if (spi_context_rx_buf_on(&data->ctx)) {
			*data->ctx.rx_buf = rx;
		}
		spi_context_update_rx(&data->ctx, 1, 1);
	}

done:
	regs->CTLR1 &= ~(SPI_CTLR1_SPE);
	spi_context_cs_control(&data->ctx, false);
	spi_context_release(&data->ctx, err);
	return err;
}

static int spi_wch_transceive_sync(const struct device *dev, const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs)
{
	return spi_wch_transceive(dev, config, tx_bufs, rx_bufs);
}

static int spi_wch_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_wch_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_wch_init(const struct device *dev)
{
	int err;
	const struct spi_wch_config *cfg = dev->config;
	struct spi_wch_data *data = dev->data;
	clock_control_subsys_t clk_sys;

	clk_sys = (clock_control_subsys_t)(uintptr_t)cfg->clock_id;

	err = clock_control_on(cfg->clk_dev, clk_sys);
	if (err < 0) {
		return err;
	}

	err = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

#ifdef CONFIG_SPI_WCH_DMA
	k_sem_init(&data->dma_sem, 0, 2);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_wch_driver_api) = {
	.transceive = spi_wch_transceive_sync,
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_wch_release,
};

#define SPI_WCH_DMA_CHAN_INIT(inst)                                                                \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),                                             \
		(.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, tx)),                    \
		 .dma_tx_chan = DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel),                      \
		 .dma_rx_chan = DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel),),                    \
		())

#define SPI_WCH_DEVICE_INIT(n)                                                                     \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct spi_wch_config spi_wch_config_##n = {                                  \
		.regs = (SPI_TypeDef *)DT_INST_REG_ADDR(n),                                        \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                  \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
		.clock_id = DT_INST_CLOCKS_CELL(n, id),                                            \
		SPI_WCH_DMA_CHAN_INIT(n)};                                                         \
	static struct spi_wch_data spi_wch_dev_data_##n = {                                        \
		SPI_CONTEXT_INIT_LOCK(spi_wch_dev_data_##n, ctx),                                  \
		SPI_CONTEXT_INIT_SYNC(spi_wch_dev_data_##n, ctx),                                  \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	SPI_DEVICE_DT_INST_DEFINE(n, spi_wch_init, NULL, &spi_wch_dev_data_##n,                    \
				  &spi_wch_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,      \
				  &spi_wch_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_WCH_DEVICE_INIT)
