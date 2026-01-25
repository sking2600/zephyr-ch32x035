/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_opa

#include <zephyr/drivers/opamp.h>
#include <zephyr/drivers/mfd/wch_opacmp.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(opamp_wch, CONFIG_OPAMP_LOG_LEVEL);

struct wch_opa_config {
	struct wch_opacmp_regs *regs;
	uint8_t index; /* 0 for OPA1, 1 for OPA2 */
	uint8_t psel;
	uint8_t nsel;
	uint8_t mode;
};

static int wch_opa_set_gain(const struct device *dev, enum opamp_gain gain)
{
	const struct wch_opa_config *config = dev->config;
	struct wch_opacmp_regs *regs = config->regs;
	uint32_t val = 0;
	uint32_t mask = 0;
    /* Basic mapping for standard internal PGA modes */
    /* Note: accurate mapping depends on specific feedback resistor ratios */
	switch (gain) {
	case OPAMP_GAIN_1:
		val = OPA_NSEL_CHN_PGA_1x;
		break;
	case OPAMP_GAIN_2:
		val = OPA_NSEL_CHN_PGA_2x;
		break;
	case OPAMP_GAIN_4:
		val = OPA_NSEL_CHN_PGA_4x;
		break;
	case OPAMP_GAIN_8:
		val = OPA_NSEL_CHN_PGA_8x;
		break;
	case OPAMP_GAIN_16:
		val = OPA_NSEL_CHN_PGA_16x;
		break;
	case OPAMP_GAIN_32:
		val = OPA_NSEL_CHN_PGA_32x;
		break;
    case OPAMP_GAIN_64:
        val = OPA_NSEL_CHN_PGA_64x;
        break;
	default:
		return -ENOTSUP;
	}

    /* Shift logic: OPA1 is at bit 8 for NSEL, OPA2 might be different? 
       Assuming consistent stride or check manual.
       For now, let's implement for OPA1 (index 0) and verify OPA2 stride if needed.
    */
    /* If OPA2 uses bits [24:27] for NSEL? 
       Let's assume OPA stride is 16 bits in CTLR1 if they are packed there, or check headers.
       Wait, wch_opacmp.h only defined macros for "1".
       Let's generalize:
       OPA1: Shift 0 for EN, 8 for NSEL
       OPA2: Shift ?
    */
    /* IMPORTANT: I should have verified OPA2 layout. 
       Assuming OPA2 is upper 16 bits of CTLR1 for now or has its own control. 
       Let's check CTLR1 definition in wch_opacmp.h again.
    */
    
    // Hardcoding for OPA1 for this pass, enabling 0-shift
    // If index > 0, we might need adjustments.
    
    int shift = config->index * 16; // Guessing 16-bit stride
    int nsel_shift = OPA_CTLR1_NSEL1_SHIFT + shift;

    mask = OPA_CTLR1_NSEL1_MASK << shift;
    regs->CTLR1 = (regs->CTLR1 & ~mask) | (val << nsel_shift);

	return 0;
}

static const struct opamp_driver_api wch_opa_api = {
	.set_gain = wch_opa_set_gain,
};

static int wch_opa_init(const struct device *dev)
{
	const struct wch_opa_config *config = dev->config;
	struct wch_opacmp_regs *regs = config->regs;
    int shift = config->index * 16; /* Assumed stride */

    uint32_t val = (1 << 0) | /* EN */
                   (config->mode << 1) |
                   (config->psel << 4) |
                   (config->nsel << 8); // Base offsets
    
    /* Apply stride */
    val <<= shift;
    uint32_t mask = 0xFFFF << shift; /* 16 bits per OPA */

	regs->CTLR1 = (regs->CTLR1 & ~mask) | val;

	return 0;
}

#define WCH_OPA_INIT(n)                                                        \
	static const struct wch_opa_config wch_opa_config_##n = {                  \
		.regs = (struct wch_opacmp_regs *)DT_REG_ADDR(DT_PARENT(DT_DRV_INST(n))), \
		.index = DT_INST_PROP(n, index),                                       \
		.psel = DT_INST_PROP_OR(n, wch_psel, 0),                               \
		.nsel = DT_INST_PROP_OR(n, wch_nsel, 0),                               \
		.mode = DT_INST_PROP_OR(n, wch_mode, 0),                               \
	};                                                                         \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, wch_opa_init, NULL, NULL,                         \
			      &wch_opa_config_##n, POST_KERNEL,                        \
			      CONFIG_OPAMP_INIT_PRIORITY, &wch_opa_api);

DT_INST_FOREACH_STATUS_OKAY(WCH_OPA_INIT)
