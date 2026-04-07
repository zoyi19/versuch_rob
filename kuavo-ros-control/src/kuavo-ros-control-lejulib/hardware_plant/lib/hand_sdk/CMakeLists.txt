cmake_minimum_required(VERSION 3.12)
project(hand_sdk)

# 添加protobuf-c库
find_package(Protobuf REQUIRED)
find_library(PROTOBUF_C_LIB protobuf-c)

set(PROTOBUF_C_LIB "/usr/lib/x86_64-linux-gnu/libprotobuf-c.so") 
include_directories(./ ${Protobuf_INCLUDE_DIRS} /usr/include/protobuf-c)

# 包含头文件目录
# include_directories(./ ${Protobuf_INCLUDE_DIRS})

# 添加源文件
set(SOURCES
    hand_sdk.c
    main_mtr_shared.pb-c.c
    main_to_mtr.pb-c.c
    mtr_to_main.pb-c.c
    softCRC.c
    stark_common.pb-c.c
    control_hand.c
)

# 生成库文件
add_library(hand_sdkLib SHARED ${SOURCES})
target_include_directories(hand_sdkLib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:/usr/include/protobuf-c>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(hand_sdkLib PUBLIC  ${PROTOBUF_C_LIB} ${Protobuf_LIBRARIES})

add_executable(hand_test hand_test.c)

# 链接protobuf-c库
message("fafdsf" ${Protobuf_LIBRARIES})
target_link_libraries(hand_test PUBLIC hand_sdkLib)

add_executable(control_hand control_hand.c)

target_link_libraries(control_hand PUBLIC hand_sdkLib)


install(TARGETS hand_sdkLib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY kuavo_brainco_hand_ota proto
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)
