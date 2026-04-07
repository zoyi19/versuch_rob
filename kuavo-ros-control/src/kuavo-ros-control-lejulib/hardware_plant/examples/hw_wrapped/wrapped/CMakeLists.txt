# Copyright 2025 Lejurobot. All rights reserved.
#
# hw_wrapped 静态库 CMake 配置
# 该库封装了 hardware_plant，提供简洁易用的硬件接口

cmake_minimum_required(VERSION 3.0.2)
project(hw_wrapped)

# ==============================================================================
# 依赖项
# ==============================================================================
find_package(hardware_plant REQUIRED)

# ==============================================================================
# 环境变量配置
# ==============================================================================
# KUAVO_ASSETS_PATH: 机器人资源文件路径（通过 source hardware_lib_setup.sh 设置）
set(KUAVO_ASSETS_PATH $ENV{KUAVO_ASSETS_PATH})
add_compile_definitions(KUAVO_ASSETS_PATH="${KUAVO_ASSETS_PATH}")
message(STATUS "KUAVO_ASSETS_PATH: ${KUAVO_ASSETS_PATH}")

# HARDWARE_PLANT_PATH: hardware_plant 库路径
set(HARDWARE_PLANT_PATH $ENV{HARDWARE_PLANT_PATH})

# ==============================================================================
# hw_wrapped 静态库
# ==============================================================================
add_library(hw_wrapped STATIC
    src/hw_wrapped.cpp      # 输出流运算符实现
    src/hw_wrapped_impl.cc  # HardwareInterface 核心实现
)

# 头文件目录
# - PUBLIC: include/ 目录对外暴露
# - PRIVATE: src/ 目录仅内部使用（包含 hw_wrapped_log.h）
target_include_directories(hw_wrapped
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 链接 hardware_plant 库
target_link_libraries(hw_wrapped
    PUBLIC
        hardware_plant::hardware_plant
)

# 编译定义
target_compile_definitions(hw_wrapped
    PRIVATE
        PROJECT_SOURCE_DIR="${HARDWARE_PLANT_PATH}"
)
