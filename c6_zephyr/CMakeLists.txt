# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(c6_zephyr)

# nanopb — reuse the P4 driver's .proto verbatim for wire compatibility.
list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)
zephyr_nanopb_sources(app src/esp_hosted_rpc.proto)

target_sources(app PRIVATE
	src/main.c
	src/link.c
	src/rpc.c
	src/ota_boot.c
)
target_include_directories(app PRIVATE src ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/src)
