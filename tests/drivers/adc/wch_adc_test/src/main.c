/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>

#define ADC_NODE DT_NODELABEL(adc1)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT

static uint16_t sample_buffer[3];

static const struct adc_channel_cfg channel_pa3_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = 3, /* PA3 is OPA1 Output */
	.differential = 0,
};

static const struct adc_channel_cfg channel_temp_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = 16,
	.differential = 0,
};

static const struct adc_channel_cfg channel_vref_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = 17,
	.differential = 0,
};

static const struct adc_sequence sequence_all = {
	.channels = BIT(3) | BIT(16) | BIT(17),
	.buffer = sample_buffer,
	.buffer_size = sizeof(sample_buffer),
	.resolution = ADC_RESOLUTION,
	.calibrate = true,
};

int main(void)
{
	int err;
    extern void sdi_console_init(void);
    extern void sdi_console_printf(const char *format, ...);
    
    sdi_console_init();
    sdi_console_printf("\n*** WCH ADC + PGA (8x) DIAGNOSTIC ***\n");

	if (!device_is_ready(adc_dev)) {
		sdi_console_printf("ADC device not ready\n");
		return 0;
	}

	(void)adc_channel_setup(adc_dev, &channel_pa3_cfg);
	(void)adc_channel_setup(adc_dev, &channel_temp_cfg);
	(void)adc_channel_setup(adc_dev, &channel_vref_cfg);

	sdi_console_printf("ADC+PGA configured. Running main loop...\n");

	while (1) {
        /* Use sentinel value 0xAAAA (not raw ADC output) to detect unwritten slots */
        sample_buffer[0] = 0xAAAA;
        sample_buffer[1] = 0xAAAA;
        sample_buffer[2] = 0xAAAA;
        
		err = adc_read(adc_dev, &sequence_all);
		if (err < 0) {
			sdi_console_printf("ADC Read Failed: %d | Buffer: %04X %04X %04X\n", 
                err, sample_buffer[0], sample_buffer[1], sample_buffer[2]);
		} else {
            /* PA0 is positive input to OPA1 (8x gain). Output on PA3 (Channel 3) */
            sdi_console_printf("PA0 (8x): %4u | TEMP: %4u | VREF: %4u\n", 
                sample_buffer[0], sample_buffer[1], sample_buffer[2]);
        }
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
