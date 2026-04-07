cmake_minimum_required(VERSION 3.15)
project(canbus_sdk VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release (CMake default)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Always include debug information in all build types
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g")

include(GNUInstallDirs)

set(BMAPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../BMAPI")
set(LEJU_CAN_API_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../LEJU_CAN_API")
SET(DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan")
SET(LIB_LIST yaml-cpp stdc++fs usb-1.0 bmapi64 stdc++ pthread)

# Find dependencies
add_library(canbus_sdk SHARED
    src/canbus_base.cpp
    src/bm_canbus_wrapper.cpp
    src/leju_canbus_wrapper.cpp
    src/bus_control_block.cpp
    src/canbus_ctrl.cpp
    src/canbus_sdk.cpp
    src/config_parser.cpp
    ${LEJU_CAN_API_DIR}/CANFD_leju.cpp
)

# Include directories
target_include_directories(canbus_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${YAML_CPP_INCLUDE_DIRS}
        ${USB1_INCLUDE_DIRS}
    PRIVATE
        ${BMAPI_DIR}/include
        ${LEJU_CAN_API_DIR}
)

# Link libraries
target_link_libraries(canbus_sdk
    PUBLIC
    ${LIB_LIST}
)
target_link_directories(canbus_sdk PUBLIC
  $<BUILD_INTERFACE:${DEPS_DIR}>
)

# Set RPATH for the library to find third party libraries
set_target_properties(canbus_sdk PROPERTIES
    INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib
    BUILD_WITH_INSTALL_RPATH FALSE
)

# Export the target
install(TARGETS canbus_sdk
    EXPORT canbus_sdkTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Testing
option(ENABLE_TESTS "Build tests" ON)
if(ENABLE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Examples
option(ENABLE_EXAMPLES "Build examples" ON)
if(ENABLE_EXAMPLES)
    add_subdirectory(examples)
endif()

# Configure package
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/canbus_sdkConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(EXPORT canbus_sdkTargets
    FILE canbus_sdkTargets.cmake
    NAMESPACE canbus_sdk::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canbus_sdk
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/canbus_sdkConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canbus_sdk
)

install(DIRECTORY include/canbus_sdk
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT Development
)