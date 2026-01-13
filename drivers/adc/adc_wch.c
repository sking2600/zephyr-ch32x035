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

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_wch);

#define WCH_ADC_PGA_1X 0
#define WCH_ADC_PGA_4X ADC_PGA_0
#define WCH_ADC_PGA_16X ADC_PGA_1
#define WCH_ADC_PGA_64X ADC_PGA

struct adc_wch_config {
	ADC_TypeDef *regs;
	const struct pinctrl_dev_config *pin_cfg;
	const struct device *clock_dev;
	uint32_t clock_id;
#ifdef CONFIG_ADC_WCH_DMA
	const struct device *dma_dev;
	uint32_t dma_chan;
#endif
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
	uint16_t *samples = sequence->buffer;

	if (sequence->options != NULL) {
		return -ENOTSUP;
	}

	if (sequence->oversampling != 0) {
		return -ENOTSUP;
	}
	if (sequence->channels >= (1 << 16)) {
		return -EINVAL;
	}

	if (sequence->calibrate) {
		uint32_t timeout = ADC_WCH_TIMEOUT_US / ADC_WCH_TIMEOUT_STEP_US;

		regs->CTLR2 |= ADC_RSTCAL;
		while ((regs->CTLR2 & ADC_RSTCAL) != 0 && timeout > 0) {
			k_busy_wait(ADC_WCH_TIMEOUT_STEP_US);
			timeout--;
		}
		if (timeout == 0) {
			LOG_ERR("Calibration reset timeout");
			return -EIO;
		}

		timeout = ADC_WCH_TIMEOUT_US / ADC_WCH_TIMEOUT_STEP_US;
		regs->CTLR2 |= ADC_CAL;
		while ((regs->CTLR2 & ADC_CAL) != 0 && timeout > 0) {
			k_busy_wait(ADC_WCH_TIMEOUT_STEP_US);
			timeout--;
		}
		if (timeout == 0) {
			LOG_ERR("Calibration timeout");
			return -EIO;
		}
	}

	/*
	 * Build the sample sequence. The channel IDs are packed 5 bits at a time starting in RSQR3
	 * and working down in memory to RSQR1.
	 */
	regs->RSQR1 = 0;
	regs->RSQR2 = 0;
	regs->RSQR3 = 0;

	for (i = 0; channels != 0; i++, channels >>= 1) {
		if ((channels & 1) != 0) {
			if (first_channel < 0) {
				first_channel = i;
			}
			total_channels++;
			(&regs->RSQR1)[rsqr] |= i << sequence_id;
			/* Each channel ID is 5 bits wide */
			sequence_id += 5;
			/* Each sequence register can hold 6 x 5 bit channel IDs */
			if (sequence_id >= 30) {
				/* Move on to the next RSQRn register, i.e. RSQR(n-1) */
				sequence_id = 0;
				rsqr--;
			}
		}
	}
	if (total_channels == 0) {
		return 0;
	}
	if (sequence->buffer_size < total_channels * sizeof(*samples)) {
		return -ENOMEM;
	}

	/* Set the number of channels to read. Note that '0' means 'one channel'. */
	regs->RSQR1 |= (total_channels - 1) * ADC_L_0;

	/* Apply PGA gain from the first channel in the sequence. 
	 * Note: Built-in PGA is global for the sequence.
	 */
#if defined(ADC_PGA) || defined(ADC_CTLR1_PGA)
	if (first_channel >= 0) {
		uint32_t ctlr1 = regs->CTLR1;
#if defined(ADC_PGA)
		ctlr1 &= ~ADC_PGA;
#else
		ctlr1 &= ~ADC_CTLR1_PGA;
#endif
		ctlr1 |= data->gain[first_channel];
		regs->CTLR1 = ctlr1;
	}
#endif

#ifdef CONFIG_ADC_WCH_DMA
	if (config->dma_dev != NULL) {
		struct adc_wch_dma_ctx dma_ctx;
		struct dma_config dma_cfg = {0};
		struct dma_block_config dma_blk = {0};

		k_sem_init(&dma_ctx.sem, 0, 1);

		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.source_data_size = 2;
		dma_cfg.dest_data_size = 2;
		dma_cfg.callback_arg = &dma_ctx;
		dma_cfg.dma_callback = adc_wch_dma_callback;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;

		dma_blk.block_size = total_channels * 2;
		dma_blk.source_address = (uint32_t)&regs->RDATAR;
		dma_blk.dest_address = (uint32_t)sequence->buffer;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		int err = dma_config(config->dma_dev, config->dma_chan, &dma_cfg);
		if (err != 0) {
			return err;
		}

		regs->CTLR2 |= ADC_DMA;
		
		err = dma_start(config->dma_dev, config->dma_chan);
		if (err != 0) {
			return err;
		}

		regs->CTLR2 |= ADC_RSWSTART;

		k_sem_take(&dma_ctx.sem, K_FOREVER);

		regs->CTLR2 &= ~ADC_DMA;

		return dma_ctx.status;
	}
#endif

	regs->CTLR2 |= ADC_RSWSTART;
	for (i = 0; i < total_channels; i++) {
		uint32_t timeout = ADC_WCH_TIMEOUT_US / ADC_WCH_TIMEOUT_STEP_US;

		while ((regs->STATR & ADC_EOC) == 0 && timeout > 0) {
			k_busy_wait(ADC_WCH_TIMEOUT_STEP_US);
			timeout--;
		}
		if (timeout == 0) {
			LOG_ERR("Conversion timeout");
			return -EIO;
		}
		*samples++ = regs->RDATAR;
	}

	return 0;
}

static int adc_wch_init(const struct device *dev)
{
	struct adc_wch_config *config = (struct adc_wch_config *)dev->config;
	ADC_TypeDef *regs = config->regs;
	int err;

	LOG_INF("WCH ADC driver init");

	clock_control_on(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	/*
	 * The default sampling time is 3 cycles and shows coupling between channels. Use 15 cycles
	 * instead. Arbitrary.
	 */
	regs->SAMPTR2 = ADC_SMP0_1 | ADC_SMP1_1 | ADC_SMP2_1 | ADC_SMP3_1 | ADC_SMP4_1 |
			ADC_SMP5_1 | ADC_SMP6_1 | ADC_SMP7_1 | ADC_SMP8_1 | ADC_SMP9_1;

	regs->CTLR2 = ADC_ADON | ADC_EXTSEL | ADC_TSVREFE;

#if defined(ADC_PGA)
	/* Default to Gain 1x if supported */
	regs->CTLR1 &= ~ADC_PGA;
#elif defined(ADC_CTLR1_PGA)
	regs->CTLR1 &= ~ADC_CTLR1_PGA;
#endif

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
		 .dma_chan = DT_INST_DMAS_CELL(n, channel),),		\
		())
#else
#define ADC_WCH_DMA_NODE(n)
#endif

#define ADC_WCH_DEVICE(n)                                                                     \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static DEVICE_API(adc, adc_wch_api_##n) = {                                           \
		.channel_setup = adc_wch_channel_setup,                                       \
		.read = adc_wch_read,                                                         \
		.ref_internal = DT_INST_PROP(n, vref_mv),                                          \
	};                                                                                         \
                                                                                                   \
	static struct adc_wch_data adc_wch_data_##n;                                         \
                                                                                                   \
	static const struct adc_wch_config adc_wch_config_##n = {                        \
		.regs = (ADC_TypeDef *)DT_INST_REG_ADDR(n),                                        \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_id = DT_INST_CLOCKS_CELL(n, id),                                            \
		ADC_WCH_DMA_NODE(n)								   \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(n, adc_wch_pm_action);                                            \
	DEVICE_DT_INST_DEFINE(n, adc_wch_init, PM_DEVICE_DT_INST_GET(n), &adc_wch_data_##n, &adc_wch_config_##n, \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &adc_wch_api_##n);

DT_INST_FOREACH_STATUS_OKAY(ADC_WCH_DEVICE)
