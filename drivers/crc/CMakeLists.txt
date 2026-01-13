# Copyright (c) 2025 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/crc.h)

zephyr_library()

# zephyr-keep-sorted-start
zephyr_library_sources_ifdef(CONFIG_CRC_DRIVER_RENESAS_RA crc_renesas_ra.c)
zephyr_library_sources_set_if_requested(CONFIG_CRC_SF32LB crc_sf32lb.c)
zephyr_library_sources_set_if_requested(CONFIG_CRC_WCH    crc_wch.c)
# zephyr-keep-sorted-stop
