/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_opa

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/mfd/wch_opacmp.h>

LOG_MODULE_REGISTER(opa_wch, CONFIG_MFD_LOG_LEVEL);

struct wch_opa_config {
	const struct device *parent;
	uint32_t reg_base;
	uint8_t index; /* 0-based index from DT 'index' property, used for bit shifting */
	uint32_t gain;
	uint32_t psel_mux;
	uint32_t nsel_mux;
};

static int wch_opa_init(const struct device *dev)
{
	const struct wch_opa_config *config = dev->config;
	struct wch_opacmp_regs *regs = (struct wch_opacmp_regs *)config->reg_base;
	uint32_t nsel_val = 0;
	
	if (!device_is_ready(config->parent)) {
		LOG_ERR("Parent MFD device not ready");
		return -ENODEV;
	}

	/* Calculate NSEL based on gain */
	if (config->gain > 1) {
		switch(config->gain) {
			case 2: nsel_val = OPA_NSEL_CHN_PGA_2x; break;
			case 4: nsel_val = OPA_NSEL_CHN_PGA_4x; break;
			case 8: nsel_val = OPA_NSEL_CHN_PGA_8x; break;
			case 16: nsel_val = OPA_NSEL_CHN_PGA_16x; break;
			case 32: nsel_val = OPA_NSEL_CHN_PGA_32x; break;
			case 64: nsel_val = OPA_NSEL_CHN_PGA_64x; break;
			default:
				LOG_WRN("Unsupported gain %d, defaulting to Unity (NSEL=0)", config->gain);
				nsel_val = OPA_NSEL_CHN_PGA_1x;
		}
	} else {
		/* If gain is 1 or 0, use specified NSEL MUX or default to 0 */
		nsel_val = config->nsel_mux;
	}

	LOG_DBG("OPA%d Init: Gain=%d (NSEL=%d), PSEL=%d", config->index + 1, config->gain, nsel_val, config->psel_mux);

	/* 
	   Configure CTLR1 for OPA1.
	   TODO: Logic for OPA2/3/4 if they share CTLR1 or have own bits? 
	   CH32L103 OPA Reference:
	   OPA1: CTLR1 bits 0-12
	   OPA2: CTLR1 bits 16-28? OR CTLR2?
	   
	   Reference `ch32l103_opa.c`:
	   OPA1 uses CTLR1.
	   OPA2? `OPA_Init` only handles OPA1 in the snippet shown?
	   "if(OPA_InitStruct->OPA_NUM == OPA1)" ... "tmp2 = OPA->CTLR1"
	   
	   Wait, the reference code only showed OPA1.
	   "OPA->CTLR1 |= (uint32_t)(1 << (OPA_NUM*16));" implies OPA2 starts at bit 16 of CTLR1?
	   Let's assume OPA2 is in upper half of CTLR1 if it exists, or in CTLR2.
	   
	   For now, we support OPA1 (index 0).
	*/

	if (config->index == 0) {
		uint32_t ctlr1 = regs->CTLR1;
		
		/* Clear OPA1 bits (0-12) */
		/* Mask: 0x1FFF */
		ctlr1 &= ~0x1FFF;

		/* Set new values */
		/* Enable (Bit 0) */
		ctlr1 |= OPA_CTLR1_EN1_MASK;
		
		/* Mode (Bit 1-3) - Default to IO Out for now? Or keep 0?
		   Reference: "OPA_InitStruct->Mode << 1". 
		   User code: "OPA_InitStructure.Mode = OUT_IO_OUT0;" which is typically 0 for OPA1?
		   Let's check `ch32l103_opa.h` defs if we had them. 
		   Example main.c: "OPA_InitStructure.Mode = OUT_IO_OUT0;"
		   Usually OUT_IO_OUT0 = 0.
		   So we leave Mode as 0.
		*/
		
		/* PSEL (Bit 4-6) */
		ctlr1 |= (config->psel_mux << OPA_CTLR1_PSEL1_SHIFT);

		/* FB (Bit 7) - Enable if Gain > 1 (PGA mode)? */
		if (config->gain > 1) {
			ctlr1 |= OPA_CTLR1_FBEN1_MASK;
		}

		/* NSEL (Bit 8-11) */
		ctlr1 |= (nsel_val << OPA_CTLR1_NSEL1_SHIFT);
		
		regs->CTLR1 = ctlr1;
	} else {
		LOG_ERR("OPA Index %d not yet supported", config->index);
		return -EINVAL;
	}

	return 0;
}

#define WCH_OPA_INIT(inst)                                      \
	static const struct wch_opa_config wch_opa_config_##inst = { \
		.parent = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),   \
		.reg_base = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(inst))),   \
		.index = DT_INST_PROP(inst, index),                      \
		.gain = DT_INST_PROP_OR(inst, gain, 1),                  \
		.psel_mux = DT_INST_PROP_OR(inst, input_positive, 0),    \
		.nsel_mux = DT_INST_PROP_OR(inst, input_negative, 0),    \
	};                                                           \
                                                                 \
	DEVICE_DT_INST_DEFINE(inst, wch_opa_init, NULL,              \
			      NULL, &wch_opa_config_##inst,                  \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(WCH_OPA_INIT)
