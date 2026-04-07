cmake_minimum_required(VERSION 3.12)
project(ruiwo_cxx_lib)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_BUILD_TYPE Debug)

set(BMAPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../BMAPI")
# 添加lib目录 
SET(DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan")
SET(LIB_LIST yaml-cpp stdc++fs usb-1.0 bmapi64 stdc++ pthread)

link_directories("${DEPS_DIR}")
include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${PROJECT_SOURCE_DIR}/src/
  ${BMAPI_DIR}/include
)

file(GLOB_RECURSE SRC_FILE
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.cc
  ${PROJECT_SOURCE_DIR}/src/*.c
)

# 生成库文件
set (ruiwo_lib_name ruiwo_actuatorCXXLib)
add_library(${ruiwo_lib_name} SHARED ${SRC_FILE} ${PROJECT_SOURCE_DIR}/../leju_claw_driver/src/ruiwoSDK.c)
target_link_libraries(${ruiwo_lib_name} PUBLIC ${LIB_LIST})
target_include_directories(${ruiwo_lib_name} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../../include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../leju_claw_driver/include>
  $<INSTALL_INTERFACE:include/ruiwo_controller_cxx>
  $<INSTALL_INTERFACE:include/ruiwo_controller_cxx/include>
  $<INSTALL_INTERFACE:include/leju_claw_driver>
)
target_link_directories(${ruiwo_lib_name} PUBLIC
  $<BUILD_INTERFACE:${DEPS_DIR}>
)

# 添加可执行文件
add_executable(ruiwo_cxx_test ${PROJECT_SOURCE_DIR}/test/ruiwo_test.cpp)
target_link_libraries(ruiwo_cxx_test ${ruiwo_lib_name})

add_executable(setpos_test ${PROJECT_SOURCE_DIR}/test/setpos_test.cc)
target_link_libraries(setpos_test ${ruiwo_lib_name})

install(TARGETS ${ruiwo_lib_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include 
  DESTINATION include/ruiwo_controller_cxx
)

install(FILES ${BMAPI_DIR}/bin/unix64/release/libbmapi64.so
  DESTINATION lib
)