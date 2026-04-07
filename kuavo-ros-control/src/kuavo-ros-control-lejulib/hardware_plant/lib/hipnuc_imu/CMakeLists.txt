cmake_minimum_required(VERSION 3.0.2)
project(hipnuc_imu)

include_directories( ${EIGEN_INCLUDE_DIRS} )

find_package( Eigen3 REQUIRED )
include_directories( /usr/include/eigen3/ )

add_library(hipnuc_imu_receiver src/hipnuc_imu_receiver.cpp src/hipnuc_dec.c)
add_executable(hipimu_test src/hipimu_test.cpp)
target_link_libraries(hipimu_test hipnuc_imu_receiver pthread lcm)
target_include_directories(hipnuc_imu_receiver PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/hipnuc_imu/src>
  $<INSTALL_INTERFACE:/usr/include/eigen3>
)


install(TARGETS hipnuc_imu_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES src/hipnuc.h src/hipnuc_imu_receiver.h
  DESTINATION include/${PROJECT_NAME}/src
)

install(DIRECTORY scripts
  DESTINATION share/hardware_plant/lib/hipnuc_imu
)


