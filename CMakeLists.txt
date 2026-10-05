# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(c6_sdr CXX)

target_sources(app PRIVATE
	src/main.cpp
	src/sdr.cpp
	src/audio.cpp
)
# ESP32-P4 spectrum / raw-IQ front-end driven through the c6_sdr hybrid
# slave firmware on the ESP32-C6 over the ESP-Hosted SDIO link.
#
#   west build -b esp32p4_function_ev_v1/esp32p4/hpcore --sysbuild
