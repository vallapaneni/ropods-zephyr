# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(icm20948_sample)

# Add driver include path for custom attributes
target_include_directories(app PRIVATE ../../drivers/sensor/icm20948)

# Build the self-test demo instead of interrupt demo
target_sources(app PRIVATE src/self_test_demo.c)
