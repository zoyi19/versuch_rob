cmake_minimum_required(VERSION 3.0.2)
project(hw_wrapped_example)

# 添加 hw_wrapped 库
add_subdirectory(wrapped)

# ==============================================================================
# hw_wrapped Example
# ==============================================================================
add_executable(wrapped_example
    wrapped_example.cpp
)

target_link_libraries(wrapped_example
    PRIVATE
        hw_wrapped
)

# ==============================================================================
# IMU Example
# ==============================================================================
add_executable(imu_example
    imu_example.cpp
)

target_link_libraries(imu_example
    PRIVATE
        hw_wrapped
)

# ==============================================================================
# Joint Sin Example
# ==============================================================================
add_executable(joint_sin_example
    joint_sin_example.cpp
)

target_link_libraries(joint_sin_example
    PRIVATE
        hw_wrapped
)

# ==============================================================================
# Hardware Interface Example
# ==============================================================================
add_executable(hardware_interface_example
    hardware_interface_example.cpp
)

target_link_libraries(hardware_interface_example
    PRIVATE
        hw_wrapped
)
