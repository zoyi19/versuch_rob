cmake_minimum_required(VERSION 3.0.2)
project(quick_start_example)

find_package(hardware_plant REQUIRED)

set(KUAVO_ASSETS_PATH $ENV{KUAVO_ASSETS_PATH})
add_compile_definitions(KUAVO_ASSETS_PATH="${KUAVO_ASSETS_PATH}")
message(STATUS "KUAVO_ASSETS_PATH: ${KUAVO_ASSETS_PATH}")

set(HARDWARE_PLANT_PATH $ENV{HARDWARE_PLANT_PATH})

# ==============================================================================
# Quick Start Example
# ==============================================================================
add_executable(quick_start_example
    quick_start_example.cpp
)

target_link_libraries(quick_start_example
    PUBLIC
        hardware_plant::hardware_plant
)

target_compile_definitions(quick_start_example
    PRIVATE
        PROJECT_SOURCE_DIR="${HARDWARE_PLANT_PATH}"
)
