#
# Copyright (c) 2014 Zubax, zubax.com
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

cmake_minimum_required(VERSION 2.8)

project(opengrab_epm_v3_test)

find_library(UAVCAN_LIB uavcan REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -pedantic -std=c++11 -g3")  # GCC or Clang

add_executable(subscriber subscriber.cpp)
target_link_libraries(subscriber ${UAVCAN_LIB} rt)
