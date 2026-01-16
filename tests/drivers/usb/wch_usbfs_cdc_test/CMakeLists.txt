cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wch_usbfs_cdc_test)

target_sources(app PRIVATE src/main.c)
zephyr_compile_options(-Wno-unused-variable)
