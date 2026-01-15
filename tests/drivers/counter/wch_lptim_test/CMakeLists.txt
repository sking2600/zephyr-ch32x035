# CMakeLists.txt for LPTIM Validation

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wch_lptim_validation)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE ../../../../drivers/console/sdi_console.c)
target_include_directories(app PRIVATE ../../../../include/zephyr/drivers/console)
