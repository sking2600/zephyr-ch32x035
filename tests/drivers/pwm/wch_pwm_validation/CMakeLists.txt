# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wch_pwm_validation)

target_sources(app PRIVATE src/main.c ${ZEPHYR_BASE}/drivers/console/sdi_console.c)
