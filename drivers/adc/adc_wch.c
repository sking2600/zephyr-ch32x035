/*
 * Copyright (c) 2025 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_adc

#include <hal_ch32fun.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/pm/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/dma.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_wch);

/* Standard WCH ADC bits */
#ifndef ADC_ADON
#define ADC_ADON                BIT(0)
#endif

#ifndef ADC_TSVREFE
#define ADC_TSVREFE             BIT(23)
#endif

#ifndef ADC_RSWSTART
#define ADC_RSWSTART            BIT(22)
#endif

#ifndef ADC_CTLR1_SCAN
#define ADC_CTLR1_SCAN          BIT(8)
#endif

#ifndef ADC_CTLR2_DMA
#define ADC_CTLR2_DMA           BIT(8)
#endif

#ifndef ADC_RSTCAL
#define ADC_RSTCAL              BIT(3)
#endif

#ifndef ADC_CAL
#define ADC_CAL                 BIT(2)
#endif

#ifndef ADC_L_0
#define ADC_L_0                 BIT(20)
#endif

#define WCH_ADC_PGA_1X 0
#define WCH_ADC_PGA_4X BIT(27)
#define WCH_ADC_PGA_16X BIT(28)
#define WCH_ADC_PGA_64X (BIT(27) | BIT(28))
#define WCH_ADC_PGA_MASK (BIT(27) | BIT(28))

struct adc_wch_config {
	ADC_TypeDef *regs;
	const struct pinctrl_dev_config *pin_cfg;
	const struct device *clock_dev;
	uint32_t clock_id;
#ifdef CONFIG_ADC_WCH_DMA
	const struct device *dma_dev;
	uint32_t dma_chan;
#endif
    uint16_t vref_mv;
};

struct adc_wch_data {
	uint32_t gain[18];
};

static int adc_wch_channel_setup(const struct device *dev,
				      const struct adc_channel_cfg *channel_cfg)
{
	struct adc_wch_data *data = dev->data;
	uint32_t pga = 0;

	switch (channel_cfg->gain) {
	case ADC_GAIN_1:
		pga = WCH_ADC_PGA_1X;
		break;
	case ADC_GAIN_4:
		pga = WCH_ADC_PGA_4X;
		break;
	case ADC_GAIN_16:
		pga = WCH_ADC_PGA_16X;
		break;
	case ADC_GAIN_64:
		pga = WCH_ADC_PGA_64X;
		break;
	default:
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		return -EINVAL;
	}
	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		return -EINVAL;
	}
	if (channel_cfg->differential) {
		return -EINVAL;
	}
	if (channel_cfg->channel_id >= 18) {
		return -EINVAL;
	}

	data->gain[channel_cfg->channel_id] = pga;

	return 0;
}

#ifdef CONFIG_ADC_WCH_DMA
struct adc_wch_dma_ctx {
	struct k_sem sem;
	int status;
};

static void adc_wch_dma_callback(const struct device *dev, void *user_data,
				 uint32_t channel, int status)
{
	struct adc_wch_dma_ctx *ctx = user_data;

	ctx->status = status;
	k_sem_give(&ctx->sem);
}
#endif

static int adc_wch_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_wch_config *config = dev->config;
	struct adc_wch_data *data = dev->data;
	ADC_TypeDef *regs = config->regs;
	uint32_t channels = sequence->channels;
	int rsqr = 2;
	int sequence_id = 0;
	int total_channels = 0;
	int first_channel = -1;
	int i;
    int first_channel_id = -1;
    uint32_t common_pga = 0;
	uint16_t *samples = sequence->buffer;

	if (sequence->options != NULL) {
		return -ENOTSUP;
	}

	if (sequence->oversampling != 0) {
		return -ENOTSUP;
	}
	if (sequence->channels >= (1 << 18)) {
		return -EINVAL;
	}

	/* RSQR registers setup */
	regs->RSQR1 = 0;
	regs->RSQR2 = 0;
	regs->RSQR3 = 0;

	for (i = 0; channels != 0; i++, channels >>= 1) {
		if ((channels & 1) != 0) {
			if (first_channel < 0) {
				first_channel = i;
			}
			(&regs->RSQR1)[rsqr] |= i << sequence_id;
			total_channels++;
			sequence_id += 5;
			if (sequence_id >= 30) {
				sequence_id = 0;
				rsqr--;
			}
		}
	}

	if (total_channels == 0) {
		return 0;
	}

    /* Validate that all channels in the sequence have the same gain */
    /* Hardware has a single global PGA setting in CTLR2 */
    for (i = 0; i < 18; i++) {
        if ((sequence->channels & BIT(i)) != 0) {
             if (first_channel_id == -1) {
                 first_channel_id = i;
                 common_pga = data->gain[i];
             } else {
                 if (data->gain[i] != common_pga) {
                     LOG_ERR("All channels in sequence must have same gain");
                     return -ENOTSUP;
                 }
             }
        }
    }
    
    /* Apply Gain */
    regs->CTLR2 = (regs->CTLR2 & ~WCH_ADC_PGA_MASK) | common_pga;

	if (sequence->buffer_size < total_channels * sizeof(*samples)) {
		return -ENOMEM;
	}

#ifdef CONFIG_ADC_WCH_DMA
	if (config->dma_dev != NULL) {
		struct adc_wch_dma_ctx dma_ctx;
		struct dma_config dma_cfg = {0};
		struct dma_block_config dma_blk = {0};
		int err;

		k_sem_init(&dma_ctx.sem, 0, 1);

		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.source_data_size = 2;
		dma_cfg.dest_data_size = 2;
		dma_cfg.source_burst_length = 1;
		dma_cfg.dest_burst_length = 1;
		dma_cfg.user_data = &dma_ctx;
		dma_cfg.dma_callback = adc_wch_dma_callback;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;

		dma_blk.source_address = (uint32_t)&regs->RDATAR;
		dma_blk.dest_address = (uint32_t)sequence->buffer;
		dma_blk.block_size = total_channels * sizeof(uint16_t);
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		err = dma_config(config->dma_dev, config->dma_chan, &dma_cfg);
		if (err != 0) {
			return err;
		}

		regs->CTLR2 |= ADC_CTLR2_DMA;
		
		err = dma_start(config->dma_dev, config->dma_chan);
		if (err != 0) {
			return err;
		}

		regs->STATR = 0;
		regs->CTLR2 |= ADC_RSWSTART;

		if (k_sem_take(&dma_ctx.sem, K_MSEC(100)) != 0) {
			regs->CTLR2 &= ~ADC_CTLR2_DMA;
			dma_stop(config->dma_dev, config->dma_chan);
			return -EIO;
		}

		regs->CTLR2 &= ~ADC_CTLR2_DMA;
		dma_stop(config->dma_dev, config->dma_chan);

		return dma_ctx.status;
	}
#endif

	/* Fallback to software-triggered read if DMA is disabled */
	for (i = 0; i < total_channels; i++) {
		regs->CTLR2 |= ADC_RSWSTART;
		while (!(regs->STATR & (1 << 1))); /* WAIT EOC */
		*samples++ = (uint16_t)regs->RDATAR;
	}

	return 0;
}

static int adc_wch_init(const struct device *dev)
{
	struct adc_wch_config *config = (struct adc_wch_config *)dev->config;
	ADC_TypeDef *regs = config->regs;
	int err;
    int i;

	clock_control_on(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	/* Ensure ADON is set */
	regs->CTLR2 |= ADC_ADON;

	/* Calibration */
	regs->CTLR2 |= ADC_RSTCAL;
	for (i = 0; i < 10000; i++) {
        if (!(regs->CTLR2 & ADC_RSTCAL)) break;
        k_busy_wait(1);
    }
	
    regs->CTLR2 |= ADC_CAL;
	for (i = 0; i < 10000; i++) {
        if (!(regs->CTLR2 & ADC_CAL)) break;
        k_busy_wait(1);
    }

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int adc_wch_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct adc_wch_config *config = dev->config;
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		config->regs->CTLR2 &= ~ADC_ADON;
		err = clock_control_off(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);
		if (err < 0) {
			return err;
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		err = clock_control_on(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);
		if (err < 0) {
			return err;
		}
		config->regs->CTLR2 |= ADC_ADON;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

#ifdef CONFIG_ADC_WCH_DMA
#define ADC_WCH_DMA_NODE(n)						\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),			\
		(.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(n)),	\
		 .dma_chan = DT_INST_DMAS_CELL_BY_IDX(n, 0, channel),),		\
		)
#else
#define ADC_WCH_DMA_NODE(n)
#endif

#define ADC_WCH_DEVICE(n)                                                                     \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                    \
	static const struct adc_driver_api adc_wch_api_##n = {                                 \
		.channel_setup = adc_wch_channel_setup,                                       \
		.read = adc_wch_read,                                                         \
		.ref_internal = DT_INST_PROP_OR(n, vref_mv, 3300),                            \
	};                                                                                         \
                                                                                                    \
	static struct adc_wch_data adc_wch_data_##n;                                         \
                                                                                                    \
	static const struct adc_wch_config adc_wch_config_##n = {                        \
		.regs = (ADC_TypeDef *)DT_INST_REG_ADDR(n),                                        \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_id = DT_INST_CLOCKS_CELL(n, id),                                            \
		.vref_mv = DT_INST_PROP_OR(n, vref_mv, 3300),                                      \
		ADC_WCH_DMA_NODE(n)								   \
	};                                                                                         \
                                                                                                    \
	PM_DEVICE_DT_INST_DEFINE(n, adc_wch_pm_action);                                            \
	DEVICE_DT_INST_DEFINE(n, adc_wch_init, PM_DEVICE_DT_INST_GET(n), &adc_wch_data_##n, &adc_wch_config_##n, \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &adc_wch_api_##n);

DT_INST_FOREACH_STATUS_OKAY(ADC_WCH_DEVICE)
