cmake_minimum_required(VERSION 3.0.2)
project(humanoid_interface_drake)


add_compile_options(-std=c++11)
## Find catkin macros and libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH /opt/drake/lib/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  kuavo_assets
  kuavo_common
  # humanoid_interface
)
find_package(Eigen3 REQUIRED)
find_package(drake CONFIG REQUIRED)
find_package(lcm CONFIG REQUIRED)

catkin_package(
 INCLUDE_DIRS 
  include
  ${drake_INCLUDE_DIRS}
 LIBRARIES 
  ${PROJECT_NAME}
  ankle_solver_lib
 CATKIN_DEPENDS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  kuavo_common

)

# Resolve for the package path at compile time.


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${drake_INCLUDE_DIRS}
)

add_library(ankle_solver_lib
  src/ankle_solver/arm_ankle_solver.cc
  src/ankle_solver/arm_joint_controller.cc
  src/ankle_solver/arm_tendon_controller.cc
)

target_link_libraries(ankle_solver_lib
  PUBLIC
  ${drake_LIBRARIES}
  drake::drake
)

set(target_objs
  ${PROJECT_SOURCE_DIR}/src/planner/plantIK.cc.o
)

add_library(${PROJECT_NAME}
  src/humanoid_interface_drake.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_BINARY_DIR}/include
  ${drake_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ankle_solver_lib
  ${drake_LIBRARIES}
  drake::drake
  lcm
  ${catkin_LIBRARIES}
  ${target_objs}
  stdc++fs
  )

#############
## Install ##
#############

install(TARGETS ankle_solver_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#############

add_subdirectory(test)


# Add a dependency on the custom targetrosl
# add_dependencies(${PROJECT_NAME} run_script)
