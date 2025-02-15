#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
if (NOT SYSBUILD)
	message(WARNING
		" This is a multi-image application that should be built using sysbuild.\n"
		" Add --sysbuild argument to west build command to prepare all the images.")
endif()

project(Opale)

# ====================================================================================
# MAIN
# ====================================================================================
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/main.c)

# ====================================================================================
# PERIPHERALS
# ====================================================================================
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/peripherals/adc.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/peripherals/gpio.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/peripherals/i2c.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/peripherals/pwm.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/peripherals/spi.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/peripherals/uart.c)

# ====================================================================================
# DEVICES
# ====================================================================================
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/devices/BNO055.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/devices/II2SDLPC.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/devices/LIV3R.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/devices/M95256.c)
target_sources(app PRIVATE ${CMAKE_SOURCE_DIR}/core/app/src/devices/MS5611.c)

# ====================================================================================
# TASK
# ====================================================================================
# ...