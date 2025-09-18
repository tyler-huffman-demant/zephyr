# SPDX-License-Identifier: Apache-2.0
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_C_FLAGS_DEBUG "-Og -g" CACHE STRING "" FORCE)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr COMPONENTS unittest REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(bluetooth_ull_llcp_phy_update)

add_subdirectory(${ZEPHYR_BASE}/tests/bluetooth/controller/common common)
add_subdirectory(${ZEPHYR_BASE}/tests/bluetooth/controller/uut uut)

#add_compile_options(-Og -g)

target_link_libraries(testbinary PRIVATE uut common)

target_sources(testbinary
  PRIVATE
    src/main.c
)
