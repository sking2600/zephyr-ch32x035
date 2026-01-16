/*
 * Copyright (c) 2024 WCH
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>

LOG_MODULE_REGISTER(opa_test, LOG_LEVEL_INF);

/* 
 * OPA1 Verification:
 * Input: PA0 (CHP4) - Connected to a signal source (or just floating/noise to observe)
 * Gain: 8x
 * Output: PA3 (Connected to ADC Channel 3)
 *
 * We will also read PA0 (ADC Channel 0) directly for comparison.
 */

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

#define ADC_RESOLUTION 12
#define ADC_OVERSAMPLING 0

/* Channels: 0 (PA0) and 3 (PA3) */
#define OP_AMP_GAIN 8

/* 
 * NOTE: The OPA driver init will happen automatically at boot because it's a child of MFD 
 * and enabled in DT. We don't need to call it API.
 */

static int setup_adc(void)
{
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -1;
    }
    return 0;
}

static int read_channel(uint8_t channel, int16_t *buf)
{
    struct adc_channel_cfg channel_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = channel,
        .differential = 0,
    };
    
    adc_channel_setup(adc_dev, &channel_cfg);

    struct adc_sequence sequence = {
        .channels = BIT(channel),
        .buffer = buf,
        .buffer_size = sizeof(int16_t),
        .resolution = ADC_RESOLUTION,
        .oversampling = ADC_OVERSAMPLING,
    };

    return adc_read(adc_dev, &sequence);
}

int main(void)
{
    int16_t sample_raw_pa0;
    int16_t sample_raw_pa3;
    int ret;
    int count = 0;

    /* Use printk checks for immediate output */
    printk("BOOTING OPA TEST APPLICATION...\n");

    LOG_INF("WCH OPA PGA Test Application");
    LOG_INF("Configured OPA1 for %dx Gain on PA0 -> PA3", OP_AMP_GAIN);

    if (setup_adc() < 0) {
        LOG_ERR("ADC Setup Failed");
        return 0;
    }

    /* Wait for OPA stabilization if needed? (Driver init done) */
    k_sleep(K_MSEC(100));

    while (1) {
        printk("Loop %d\n", count++);
        
        /* Read PA0 Direct */
        ret = read_channel(0, &sample_raw_pa0);
        if (ret < 0) {
            LOG_ERR("ADC Read PA0 failed %d", ret);
        }

        /* Read PA3 (Amplified) */
        ret = read_channel(3, &sample_raw_pa3);
        if (ret < 0) {
            LOG_ERR("ADC Read PA3 failed %d", ret);
        }

        /* Convert to mV? For simplicity just raw values first. */
        /* VRef ~3.3V, 12-bit = 0..4095 */
        /* 1 tick = 3300/4096 = ~0.8mV */
        
        int mv_pa0 = (sample_raw_pa0 * 3300) / 4096;
        int mv_pa3 = (sample_raw_pa3 * 3300) / 4096;

        LOG_INF("PA0 (In): %d (~%dmV) | PA3 (Out): %d (~%dmV) | Ratio: %d.%02d",
                sample_raw_pa0, mv_pa0, sample_raw_pa3, mv_pa3,
                sample_raw_pa0 ? (sample_raw_pa3 / sample_raw_pa0) : 0, 
                sample_raw_pa0 ? ((sample_raw_pa3 % sample_raw_pa0)*100/sample_raw_pa0) : 0);
                
        k_sleep(K_MSEC(1000));
    }
}
