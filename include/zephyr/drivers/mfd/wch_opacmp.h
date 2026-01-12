/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MFD_WCH_OPACMP_H_
#define ZEPHYR_INCLUDE_DRIVERS_MFD_WCH_OPACMP_H_

#include <zephyr/device.h>
#include <hal_ch32fun.h>

/**
 * @brief Register structure for WCH OPA/CMP block
 */
struct wch_opacmp_regs {
	OPA_TypeDef *regs;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_MFD_WCH_OPACMP_H_ */
