cmake_minimum_required(VERSION 3.12)
project(lejuclaw)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Check and Install dependencies
message("Check and Install dependencies.....")
execute_process(
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/install_deps_packages.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
message("Check and Install dependencies done.")

set(BMAPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../BMAPI")
message("BMAPI_DIR: ${BMAPI_DIR}")

# 添加third_party目录
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "release"))
  # link_directories("${BMAPI_DIR}/bin/unix64/release")
  link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan/)
  # 添加依赖库
  SET(LIB_LIST yaml-cpp stdc++fs usb-1.0 bmapi64 stdc++ pthread)
else()
  link_directories("${BMAPI_DIR}/bin/unix64/debug")
  # 添加依赖库
  SET(LIB_LIST yaml-cpp stdc++fs usb-1.0 bmapi64d pthread)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${PROJECT_SOURCE_DIR}/src/
  ${PROJECT_SOURCE_DIR}/../motorevo_controller/include
  ${PROJECT_SOURCE_DIR}/../canbus_sdk/include
  ${BMAPI_DIR}/include
)

file(GLOB_RECURSE SRC_FILE
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.cc
  ${PROJECT_SOURCE_DIR}/src/*.c
)

# message("SRC_FILE: ${SRC_FILE}")
# message("LIB_DIR: ${LIB_DIR}")

# 生成库文件
add_library(${PROJECT_NAME} STATIC ${SRC_FILE})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${BMAPI_DIR}/include>
    $<INSTALL_INTERFACE:include/leju_claw_driver>
)
target_link_libraries(${PROJECT_NAME} ${LIB_LIST} motorevo_controller canbus_sdk)
# target_link_directories(${PROJECT_NAME} PUBLIC ${BMAPI_DIR}/bin/unix64/release)
target_link_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan>
    $<INSTALL_INTERFACE:lib>
)

# 添加可执行文件
add_executable(lejuclaw_test ${PROJECT_SOURCE_DIR}/test/lejuclaw_test.cpp)
target_link_libraries(lejuclaw_test ${LIB_NAME} ${PROJECT_NAME}  ${LIB_LIST})

# 单位置测试（非VR模式，调用ros服务，等待到达位置）
add_executable(lejuclaw_can_test_single_position ${PROJECT_SOURCE_DIR}/test/lejuclaw_can_test_single_position.cpp)
target_include_directories(lejuclaw_can_test_single_position PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(lejuclaw_can_test_single_position ${PROJECT_NAME} ${LIB_LIST} motorevo_controller canbus_sdk)

# VR控制测试（VR模式，模拟VR发布位置，插值运动）
add_executable(lejuclaw_can_test_vr_control ${PROJECT_SOURCE_DIR}/test/lejuclaw_can_test_vr_control.cpp)
target_include_directories(lejuclaw_can_test_vr_control PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(lejuclaw_can_test_vr_control ${PROJECT_NAME} ${LIB_LIST} motorevo_controller canbus_sdk)

# 添加服务器程序
add_executable(lejuclaw_server ${PROJECT_SOURCE_DIR}/test/lejuclaw_server/lejuclaw_server.cpp)
target_link_libraries(lejuclaw_server ${LIB_NAME} ${PROJECT_NAME}  ${LIB_LIST})

install(DIRECTORY include/
  DESTINATION include/leju_claw_driver
)

install(DIRECTORY ${BMAPI_DIR}/include/
  DESTINATION include/leju_claw_driver
)

install(FILES ${BMAPI_DIR}/bin/unix64/release/libbmapi64.so
  DESTINATION lib
)
