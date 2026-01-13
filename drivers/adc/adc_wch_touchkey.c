/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_touchkey

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <hal_ch32fun.h>

LOG_MODULE_REGISTER(adc_wch_touchkey, CONFIG_ADC_LOG_LEVEL);

#define ADC_CTX_LOCK_DELAY_MS 10

struct adc_wch_touchkey_config {
    ADC_TypeDef *regs;
    const struct pinctrl_dev_config *pcfg;
    const struct device *clock_dev;
    uint32_t clock_subsys;
};

struct adc_wch_touchkey_data {
    struct adc_context ctx;
    const struct device *dev;
    uint16_t *buffer;
    uint16_t *buffer_ptr;
};

/* TKey specific bits in CTLR1 (Based on example) */
#define TKEY_CTLR1_TKEY_EN (1 << 24)
#define TKEY_CTLR1_BUF_EN  (1 << 26)

static int adc_wch_touchkey_channel_setup(const struct device *dev,
                                          const struct adc_channel_cfg *channel_cfg)
{
    if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
        return -EINVAL;
    }
    
    /* Touchkey typically uses specific internal channels, but Zephyr maps this via channel_id */
    if (channel_cfg->channel_id > 15) { /* Max ADC channels usually 16 */
         return -EINVAL;
    }

    /* Verify if differential mode is requested (not supported) */
    if (channel_cfg->differential) {
        return -ENOTSUP;
    }

    return 0;
}

static int adc_wch_touchkey_start_read(const struct device *dev,
                                       const struct adc_sequence *sequence)
{
    const struct adc_wch_touchkey_config *cfg = dev->config;
    struct adc_wch_touchkey_data *data = dev->data;
    ADC_TypeDef *adc = cfg->regs;
    int ret;
    uint32_t channel_mask = sequence->channels;
    
    if (sequence->resolution != 12) {
        /* Example implies 12-bit (uint16_t result) standard ADC usage */
        return -ENOTSUP;
    }

    /* Iterate over selected channels */
    int channel_id = 0;
    while (channel_mask) {
        if (channel_mask & 1) {
            /* Configure Channel and Sequence */
            /* ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_CyclesMode0 ) */
            /* In register:
               SAMPTR1/2 determines sample time.
               RSQR3 L[3:0] = 1 (length 1)
               RSQR3 SQ1[4:0] = channel_id
            */
            
            /* TKey Config (Per Example):
               TKey1->IDATAR1 = 0x20; // Charging Time
               TKey1->RDATAR = 0x8;   // Discharging Time
            */
            adc->IDATAR1 = 0x20;
            adc->RDATAR = 0x8;
            
            /* Channel Selection */
            /* Assuming channel_id maps to ADC channel directly */
            adc->RSQR3 = (channel_id & 0x1F); /* SQ1 */
            adc->RSQR1 = 0; /* Length = 1 conversion */
            
            /* Sample time? Example used ADC_SampleTime_CyclesMode0. 
               We should probably use existing SAMPTR settings or default. 
               Leaving as is (reset value) or configuring if needed.
            */

            /* Start Conversion */
            /* CTLR2 SWSTART */
            adc->CTLR2 |= ADC_SWSTART;
            
            /* Wait EOC */
            /* Timeout loop */
            int timeout = 10000;
            while (!(adc->STATR & ADC_EOC) && timeout-- > 0) {
                k_busy_wait(1);
            }
            
            if (timeout <= 0) {
                LOG_ERR("Touchkey timeout");
                return -ETIMEDOUT;
            }
            
            /* Read Result */
            uint16_t result = (uint16_t)(adc->RDATAR);
            
            *data->buffer_ptr++ = result;
        }
        channel_mask >>= 1;
        channel_id++;
    }
    
    adc_context_on_sampling_done(&data->ctx, dev);
    return 0;
}


static int adc_wch_touchkey_read(const struct device *dev,
                                 const struct adc_sequence *sequence)
{
    struct adc_wch_touchkey_data *data = dev->data;
    int ret;

    adc_context_lock(&data->ctx, false, NULL);
    
    ret = adc_context_wait_for_completion(&data->ctx);
    
    data->buffer = sequence->buffer;
    data->buffer_ptr = data->buffer;

    adc_context_start_read(&data->ctx, sequence);
    
    ret = adc_context_wait_for_completion(&data->ctx);
    adc_context_release(&data->ctx, ret);
    
    return ret;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_wch_touchkey_read_async(const struct device *dev,
                                       const struct adc_sequence *sequence,
                                       struct k_poll_signal *async)
{
    struct adc_wch_touchkey_data *data = dev->data;
    int ret;

    adc_context_lock(&data->ctx, true, async);
    data->buffer = sequence->buffer;
    data->buffer_ptr = data->buffer;

    adc_context_start_read(&data->ctx, sequence);
    return 0;
}
#endif


static const struct adc_driver_api adc_wch_touchkey_driver_api = {
    .channel_setup = adc_wch_touchkey_channel_setup,
    .read = adc_wch_touchkey_read,
#ifdef CONFIG_ADC_ASYNC
    .read_async = adc_wch_touchkey_read_async,
#endif
    .ref_internal = 3300, /* VCC */
};

static int adc_wch_touchkey_init(const struct device *dev)
{
    const struct adc_wch_touchkey_config *cfg = dev->config;
    struct adc_wch_touchkey_data *data = dev->data;
    ADC_TypeDef *adc = cfg->regs;
    int ret;

    /* Config Pins */
    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return ret;
    }

    /* Enable Clock */
    if (!device_is_ready(cfg->clock_dev)) {
        return -ENODEV;
    }
    ret = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)(uintptr_t)cfg->clock_subsys);
    if (ret < 0) {
        return ret;
    }
    
    /* Config ADC for TKey */
    /* Example:
       ADC_Mode_Independent, ScanConvMode DISABLE, ContinuousConvMode DISABLE
       TKey1->CTLR1 |= (1<<26)|(1<<24);
    */
    
    /* Reset CTLR1/2 to defaults? */
    /* Assuming dedicated use. */
    
    adc->CTLR1 |= TKEY_CTLR1_TKEY_EN | TKEY_CTLR1_BUF_EN;
    
    /* Enable ADC */
    adc->CTLR2 |= ADC_ADON;
    k_busy_wait(10); /* Power up delay */
    
    /* Calibration? Usually needed for ADC, maybe for TKey too */
    adc->CTLR2 |= ADC_RSTCAL;
    int timeout = 1000;
    while((adc->CTLR2 & ADC_RSTCAL) && timeout-- > 0) {
        k_busy_wait(10);
    }
    
    if (timeout <= 0) {
        LOG_ERR("Calibration reset timeout");
        return -EIO;
    }

    adc->CTLR2 |= ADC_CAL;
    timeout = 1000;
    while((adc->CTLR2 & ADC_CAL) && timeout-- > 0) {
        k_busy_wait(10);
    }

    if (timeout <= 0) {
        LOG_ERR("Calibration timeout");
        return -EIO;
    }

    adc_context_unlock_unconditionally(&data->ctx);
    
    return 0;
}


#define WCH_TOUCHKEY_INIT(n)                                                    \
    PINCTRL_DT_INST_DEFINE(n);                                                  \
    static const struct adc_wch_touchkey_config adc_wch_touchkey_cfg_##n = {    \
        .regs = (ADC_TypeDef *)DT_INST_REG_ADDR(n),                             \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                              \
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                     \
        .clock_subsys = DT_INST_CLOCKS_CELL(n, id),                             \
    };                                                                          \
    static struct adc_wch_touchkey_data adc_wch_touchkey_data_##n = {           \
        .dev = DEVICE_DT_INST_GET(n),                                           \
        .ctx = {                                                                \
            .sequence = {                                                       \
                .options = NULL,                                                \
            },                                                                  \
            .lock = Z_SEM_INITIALIZER(adc_wch_touchkey_data_##n.ctx.lock, 1, 1),\
            .sync = Z_SEM_INITIALIZER(adc_wch_touchkey_data_##n.ctx.sync, 0, 1),\
        },                                                                      \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, &adc_wch_touchkey_init, NULL,                      \
                          &adc_wch_touchkey_data_##n,                           \
                          &adc_wch_touchkey_cfg_##n,                            \
                          POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,                \
                          &adc_wch_touchkey_driver_api);

DT_INST_FOREACH_STATUS_OKAY(WCH_TOUCHKEY_INIT)
