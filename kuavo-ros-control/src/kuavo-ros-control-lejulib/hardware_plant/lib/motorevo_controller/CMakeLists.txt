cmake_minimum_required (VERSION 3.8)
project(motorevo_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(GTest REQUIRED)

# 设置 canbus_sdk 路径
set(CANBUS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk")

include_directories(
  ${GTEST_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/src/
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk/include
)

### 显式指定源文件
set(SRC_FILE
  ${PROJECT_SOURCE_DIR}/src/motor_ctrl.cpp
  ${PROJECT_SOURCE_DIR}/src/motor_utils.cpp
  ${PROJECT_SOURCE_DIR}/src/motorevo_actuator.cpp
)

# 添加lib目录
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "release"))
  link_directories("${CANBUS_SDK_DIR}/build")
  # 添加依赖库
  SET(LIB_LIST stdc++ pthread canbus_sdk)
else()
  add_definitions(-fPIC -g)
  link_directories("${CANBUS_SDK_DIR}/build")
  # 添加依赖库
  SET(LIB_LIST stdc++ pthread canbus_sdk)
endif()

# 生成动态库
add_library(${PROJECT_NAME} SHARED ${SRC_FILE})
target_link_libraries(${PROJECT_NAME} ${LIB_LIST})

# 设置包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/motorevo_controller>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../../include>
)

# 添加测试子目录
add_subdirectory(tests)

# 添加示例子目录
add_subdirectory(examples)

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include/motorevo_controller
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)