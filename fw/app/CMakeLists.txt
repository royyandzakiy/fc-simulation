cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fc_fw)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/fc_core.cmake)
target_sources(app PRIVATE src/main.cpp)
