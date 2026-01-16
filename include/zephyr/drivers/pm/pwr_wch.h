/*
 * Copyright (c) 2026 Scott King
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PM_PWR_WCH_H_
#define ZEPHYR_INCLUDE_DRIVERS_PM_PWR_WCH_H_

#include <zephyr/device.h>

/**
 * @brief Enable or disable access to the backup domain.
 *
 * @param dev PWR device instance.
 * @param enable True to enable, false to disable.
 * @return 0 on success, negative errno on failure.
 */
int wch_pwr_set_backup_access(const struct device *dev, bool enable);

/**
 * @brief Enter a low power mode.
 *
 * @param dev PWR device instance.
 * @param mode Mode to enter (0: Sleep, 1: Stop, 2: Standby).
 * @return 0 on success, negative errno on failure.
 */
int wch_pwr_enter_low_power(const struct device *dev, uint32_t mode);

#endif /* ZEPHYR_INCLUDE_DRIVERS_PM_PWR_WCH_H_ */
