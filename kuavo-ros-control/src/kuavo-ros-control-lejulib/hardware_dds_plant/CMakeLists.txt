cmake_minimum_required(VERSION 3.16)
project(hardware_dds_plant)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build and install third-party libraries if needed
set(THIRD_PARTY_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party_libs/build_and_install.sh")
set(THIRD_PARTY_MARKER "${CMAKE_CURRENT_BINARY_DIR}/third_party_built.marker")

# Check if third-party libraries need to be built
if(NOT EXISTS "${THIRD_PARTY_MARKER}")
    message(STATUS "Building third-party libraries...")
    execute_process(
        COMMAND ${THIRD_PARTY_SCRIPT}
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party_libs"
        RESULT_VARIABLE SCRIPT_RESULT
        OUTPUT_VARIABLE SCRIPT_OUTPUT
        ERROR_VARIABLE SCRIPT_ERROR
    )
    
    if(SCRIPT_RESULT EQUAL 0)
        message(WARNING "CycloneDDS libraries built successfully")
        # Create marker file to avoid rebuilding
        file(WRITE "${THIRD_PARTY_MARKER}" "Third-party libraries built on: ${CMAKE_CURRENT_LIST_DIR}")
    else()
        message(WARNING "Failed to build third-party libraries: ${SCRIPT_ERROR}")
    endif()
else()
    message(STATUS "Third-party libraries already built (marker file exists)")
endif()

# Find required packages
find_package(CycloneDDS-CXX REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate DDS types from IDL files
set(IDL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/idl/JointCmd.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/idl/SensorsData.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/idl/ImuData.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/idl/JointData.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/idl/EndEffectorData.idl
)

set(IDL_TARGETS
    JointCmd
    SensorsData
    ImuData
    JointData
    EndEffectorData
)

# Generate C++ headers from IDL files
foreach(idl ${IDL_FILES})
    get_filename_component(basename ${idl} NAME_WE)
    set(generated_header ${CMAKE_CURRENT_BINARY_DIR}/${basename}.hpp)
    set(generated_source ${CMAKE_CURRENT_BINARY_DIR}/${basename}.cpp)
    
    list(APPEND GENERATED_SOURCES ${generated_source})
    list(APPEND GENERATED_HEADERS ${generated_header})
    idlcxx_generate(TARGET ${basename} FILES ${idl} WARNINGS no-implicit-extensibility)
endforeach()

# Create static library for HardwareDdsPlant
add_library(hardware_dds_plant SHARED
    src/hardware_dds_plant.cc
    ${GENERATED_SOURCES}
)

# Set C++ standard for the library
set_property(TARGET hardware_dds_plant PROPERTY CXX_STANDARD 17)

# Library dependencies
target_link_libraries(hardware_dds_plant PUBLIC
    CycloneDDS-CXX::ddscxx
    Eigen3::Eigen
    pthread
    ${CMAKE_CURRENT_SOURCE_DIR}/unitree_sdk2/x86_64/libunitree_sdk2.a
    hardware_plant
)

# Include directories for the library
target_include_directories(hardware_dds_plant PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:/usr/local/include/ddscxx>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../hardware_plant/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/unitree_sdk2/include>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/dds_types>
)# Add custom target for generated files
add_custom_target(generate_dds_types 
    DEPENDS ${GENERATED_HEADERS} ${GENERATED_SOURCES}
)

# Make sure generated files are created before building the library
add_dependencies(hardware_dds_plant generate_dds_types ${IDL_TARGETS})

# Installation
install(TARGETS hardware_dds_plant
    EXPORT hardware_dds_plantTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install generated DDS headers
install(FILES ${GENERATED_HEADERS}
    DESTINATION include/dds_types
)


# Export configuration
install(EXPORT hardware_dds_plantTargets
    FILE hardware_dds_plantTargets.cmake
    NAMESPACE hardware_dds_plant::
    DESTINATION lib/cmake/hardware_dds_plant
)

# Create package configuration files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/hardware_dds_plantConfigVersion.cmake"
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/hardware_dds_plantConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/hardware_dds_plantConfig.cmake"
    INSTALL_DESTINATION lib/cmake/hardware_dds_plant
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/hardware_dds_plantConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/hardware_dds_plantConfigVersion.cmake"
    DESTINATION lib/cmake/hardware_dds_plant
)