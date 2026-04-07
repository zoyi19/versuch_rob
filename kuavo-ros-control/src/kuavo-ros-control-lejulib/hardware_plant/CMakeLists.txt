# ==============================================================================
# Project Configuration
# ==============================================================================
cmake_minimum_required(VERSION 3.10)
project(hardware_plant VERSION 1.0.0)

# ==============================================================================
# Build Settings
# ==============================================================================
# Enforce modern C++ standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Generate compile_commands.json for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_library(UDEV_LIBRARY udev)


if (NOT UDEV_LIBRARY)
    message(FATAL_ERROR
        "libudev not found.\n"
        "Please install it with:\n"
        "  sudo apt install libudev-dev"
    )
endif()

# ==============================================================================
# Dependencies
# ==============================================================================
# Find required packages
if (CATKIN_DEVEL_PREFIX)
    # 查找ROS包
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        kuavo_msgs
    )
    
    find_package(kuavo_common REQUIRED)
    find_package(kuavo_solver REQUIRED)
    execute_process(
        COMMAND rospack find kuavo_assets
        OUTPUT_VARIABLE KUAVO_ASSETS_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    add_definitions(-DKUAVO_ASSETS_PATH="${KUAVO_ASSETS_PATH}")
else()
    set(kuavo_common_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/kuavo_common/include)
    set(kuavo_common_LIBRARIES kuavo_common)
    # 支持两种路径：同级目录（build_hardware_lib.txt）或父级目录（正常catkin构建）
    if(EXISTS ${CMAKE_SOURCE_DIR}/kuavo_solver/include)
        set(kuavo_solver_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/kuavo_solver/include)
    else()
        set(kuavo_solver_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/../kuavo_solver/include)
    endif()
    set(kuavo_solver_LIBRARIES kuavo_solver)
endif()

# Output dependency information for debugging
# message(STATUS "kuavo_common library path: ${kuavo_common_LIBRARIES}")
# message(STATUS "kuavo_common include path: ${kuavo_common_INCLUDE_DIRS}")
# message(STATUS "kuavo_solver library path: ${kuavo_solver_LIBRARIES}")
# message(STATUS "kuavo_solver include path: ${kuavo_solver_INCLUDE_DIRS}")

# ==============================================================================
# Subdirectories
# ==============================================================================
# Add all required library subdirectories
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/Math)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/EC_Master)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/xsens_ros_mti_driver)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/ruiwo_controller)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/leju_claw_driver)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/hipnuc_imu)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/dexhand_sdk)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/ruiwo_controller_cxx)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/canbus_sdk)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/motorevo_controller)
# ==============================================================================
# Library Target
# ==============================================================================
# Define source files for the library
set(LIBRARY_SOURCES
    src/actuators_interface.cpp 
    src/Kalman.cpp 
    src/kalman_estimate.cc 
    src/utils.cpp
    src/touch_hand_controller.cpp
    src/lejuclaw_controller.cc
    src/hardware_plant.cc
    src/motor_status_manager.cpp
    src/revo2_hand_controller.cpp
)

# Create shared library
add_library(${PROJECT_NAME} STATIC ${LIBRARY_SOURCES})
add_library(${PROJECT_NAME}_cxx STATIC ${LIBRARY_SOURCES})

# Add include directories
target_include_directories(hardware_plant PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${kuavo_common_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${kuavo_solver_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include/hardware_plant>
  $<INSTALL_INTERFACE:include/kuavo_common>
  $<INSTALL_INTERFACE:include/kuavo_solver>
)

target_include_directories(hardware_plant_cxx PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${kuavo_common_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${kuavo_solver_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include/hardware_plant>
  $<INSTALL_INTERFACE:include/kuavo_common>
  $<INSTALL_INTERFACE:include/kuavo_solver>
)

# Initialize hardware include directories
set(HARDWARE_INCLUDE_DIRS
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${kuavo_common_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include/hardware_plant>
    $<INSTALL_INTERFACE:include/kuavo_common>
)

# Add ruiwo_controller_cxx to CXX include directories
set(HARDWARE_CXX_INCLUDE_DIRS
    ${HARDWARE_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lib/ruiwo_controller_cxx>
    $<INSTALL_INTERFACE:include/ruiwo_controller_cxx>
)

# Append ruiwo_controller to HARDWARE_INCLUDE_DIRS
list(APPEND HARDWARE_INCLUDE_DIRS 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lib/ruiwo_controller>
    $<INSTALL_INTERFACE:include/ruiwo_controller>
)

target_include_directories(${PROJECT_NAME} PUBLIC ${HARDWARE_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_cxx PUBLIC ${HARDWARE_CXX_INCLUDE_DIRS})

# Link dependencies
set(HARDWARE_LIBRARIES
    xsens_mti_driver
    ec_master_main
    ec_app
    mathtools_hw
    lejuclaw
    hipnuc_imu_receiver
    dexhand_sdk
    canbus_sdk
    motorevo_controller
    stdc++fs)

set(HARDWARE_CXX_LIBRARIES ${HARDWARE_LIBRARIES} ruiwo_actuatorCXXLib)
list(APPEND HARDWARE_LIBRARIES ruiwo_actuatorLib)

# Change PUBLIC to PRIVATE for internal dependencies
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        xsens_mti_driver
        ec_master_main
        ec_app
        mathtools_hw
        lejuclaw
        hipnuc_imu_receiver
        dexhand_sdk
        canbus_sdk
        motorevo_controller
        stdc++fs
        ruiwo_actuatorLib
        ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_cxx
    PUBLIC
        xsens_mti_driver
        ec_master_main
        ec_app
        mathtools_hw
        lejuclaw
        hipnuc_imu_receiver
        dexhand_sdk
        canbus_sdk
        motorevo_controller
        stdc++fs
        ruiwo_actuatorCXXLib
        ${catkin_LIBRARIES}
)

if(CATKIN_DEVEL_PREFIX)
    target_link_libraries(${PROJECT_NAME}
        PUBLIC 
            ${kuavo_common_LIBRARIES}
            ${kuavo_solver_LIBRARIES}
    )
    target_link_libraries(${PROJECT_NAME}_cxx
        PUBLIC 
            ${kuavo_common_LIBRARIES}
            ${kuavo_solver_LIBRARIES}
    )
else()
    target_link_libraries(${PROJECT_NAME}
        PUBLIC 
            kuavo_common
            kuavo_solver
    )
    target_link_libraries(${PROJECT_NAME}_cxx
        PUBLIC 
            kuavo_common
            kuavo_solver
    )
endif()

# ==============================================================================
# Test Executables
# ==============================================================================
# Leju claw controller test
add_executable(lejuclaw_controller_test 
    ${PROJECT_SOURCE_DIR}/src/tests/lejuclaw_controller_test.cc
)
target_link_libraries(lejuclaw_controller_test 
    PUBLIC 
        ${PROJECT_NAME}
)
target_include_directories(lejuclaw_controller_test 
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/src
)

add_executable(lejuclaw_ctrl_test src/tests/lejuclaw_controller_test.cc)
target_link_libraries(lejuclaw_ctrl_test PUBLIC hardware_plant_cxx)
target_include_directories(lejuclaw_ctrl_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
)
install(TARGETS lejuclaw_ctrl_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Hardware plant test
add_executable(hardware_plant_test 
    ${PROJECT_SOURCE_DIR}/src/tests/hardware_plant_test.cpp
)
target_link_libraries(hardware_plant_test 
    PUBLIC 
        ${PROJECT_NAME}
)
target_include_directories(hardware_plant_test 
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/src
)
target_compile_definitions(hardware_plant_test
    PRIVATE
        PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)


# Include IMU test script subdirectory (with ROS support)
if(CATKIN_DEVEL_PREFIX)
    add_subdirectory(${PROJECT_SOURCE_DIR}/src/tests)
    message(STATUS "Adding IMU test script subdirectory with ROS support")

    # 配置静态链接：尽可能将所有依赖编译进可执行文件
    set_target_properties(hardware_plant_wheel_arm_test PROPERTIES
        LINK_FLAGS "-static-libgcc -static-libstdc++ -Wl,-rpath,\$ORIGIN:\$ORIGIN/../lib"
    )

    # 设置 RPATH，使可执行文件能够找到库
    set_target_properties(hardware_plant_wheel_arm_test PROPERTIES
        INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib"
        BUILD_WITH_INSTALL_RPATH TRUE
    )

    option(STATIC_LINK_WHEEL_ARM_TEST "Try to fully statically link wheel_arm_test (requires static versions of all libraries)" OFF)
    if(STATIC_LINK_WHEEL_ARM_TEST)
        set_target_properties(hardware_plant_wheel_arm_test PROPERTIES
            LINK_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static"
        )
        message(STATUS "Attempting full static linking for hardware_plant_wheel_arm_test")
        message(WARNING "Full static linking may fail if static versions of system libraries are not available")
    endif()
else()
    message(STATUS "Skipping IMU test script - not in catkin environment")
endif()

# Ankle solver test
add_executable(ankle_solver_test 
    ${PROJECT_SOURCE_DIR}/src/tests/ankle_solver_test.cpp
)
target_link_libraries(ankle_solver_test 
    PUBLIC 
        ${PROJECT_NAME}
)
target_include_directories(ankle_solver_test 
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/src
)

# ==============================================================================
# Installation
# ==============================================================================
# Install library and headers
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_cxx
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(FILES hardware_lib_setup.sh
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/tests/
    DESTINATION tests
)
# Install all the subdirectory libraries
install(TARGETS
    xsens_mti_driver
    ec_master_main
    ec_app
    mathtools_hw
    # hand_sdkLib
    lejuclaw
    hipnuc_imu_receiver
    dexhand_sdk
    canbus_sdk
    motorevo_controller
    ruiwo_actuatorLib
    ruiwo_actuatorCXXLib
    xsens-lcm-types-lib
    # kuavo_common
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install public headers
install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/src/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Generate and install CMake config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

set(hardware_plant_INCLUDE_DIRS ${HARDWARE_INCLUDE_DIRS})
set(hardware_plant_LIBRARIES ${PROJECT_NAME})
set(hardware_plant_CXX_INCLUDE_DIRS ${HARDWARE_CXX_INCLUDE_DIRS})
set(hardware_plant_CXX_LIBRARIES ${PROJECT_NAME}_cxx)

# Generate the config file
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)
# Install the config files
install(FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install the export set
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)
