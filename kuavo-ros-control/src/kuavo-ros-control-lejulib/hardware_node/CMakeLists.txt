cmake_minimum_required(VERSION 3.0.2)
project(hardware_node)


set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH /opt/drake/lib/)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
if(NOT DEFINED ROBOT_VERSION)
    # 从环境变量中读取版本号
    if (NOT DEFINED ENV{ROBOT_VERSION})
        message(STATUS "ROBOT_VERSION environment variable not set.")
        message(STATUS "Defaulting to version 3.2")
        set(ROBOT_VERSION 32)
    else()
        set(ROBOT_VERSION $ENV{ROBOT_VERSION})
        message(STATUS "ROBOT_VERSION from system is ${ROBOT_VERSION}.")
    endif()
else()
  message(STATUS "ROBOT_VERSION is " ${ROBOT_VERSION})
endif()

add_compile_definitions(ROBOT_VERSION_INT=${ROBOT_VERSION})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospack
  std_msgs
  message_generation
  std_srvs
  roslib
  kuavo_msgs
  kuavo_assets
  kuavo_common
  kuavo_solver
  humanoid_interface
  humanoid_interface_drake
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
include_directories( /usr/include/eigen3/ )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  robotHeadMotionData.msg
  horizontalMotionByStep.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS 
 include
 LIBRARIES hardware_node
 CATKIN_DEPENDS roscpp kuavo_common kuavo_assets kuavo_solver std_msgs message_runtime rospy
 DEPENDS system_lib
)

###########
## Build ##
###########
# DDS support configuration
option(USE_DDS "Enable DDS support for hardware communication" OFF)

# Add USE_LEJU_DDS option
option(USE_LEJU_DDS "Enable Leju DDS communication" OFF)

set(HARDWARE_PLANT_DIR ../hardware_plant)
add_subdirectory(${HARDWARE_PLANT_DIR} "hardware_plant")

if(USE_DDS)
    add_compile_definitions(USE_DDS=1)
    message(STATUS "DDS support enabled")
endif()

if(USE_LEJU_DDS)
    add_compile_definitions(USE_LEJU_DDS=1)
    message(STATUS "Leju DDS communication enabled")
else()
    message(STATUS "Leju DDS communication disabled (compile with -DUSE_LEJU_DDS=ON to enable)")
endif()

# Add hardware_dds_plant subdirectory if any DDS support is enabled
if(USE_DDS OR USE_LEJU_DDS)
    set(HARDWARE_DDS_PLANT_DIR ../hardware_dds_plant)
    add_subdirectory(${HARDWARE_DDS_PLANT_DIR} "hardware_dds_plant")
endif()
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${HARDWARE_PLANT_DIR}/src
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)
## Declare a C++ library
add_library(ros_hardware_plant
  src/hardware_node.cc
  include/hardware_node/touch_dexhand_ros1.cpp
  src/revo2_dexhand_ros.cpp
)
add_library(ros_hardware_plant_cxx
  src/hardware_node.cc
  include/hardware_node/touch_dexhand_ros1.cpp
  src/revo2_dexhand_ros.cpp
)

# Link DDS libraries conditionally
if(USE_DDS OR USE_LEJU_DDS)
  target_link_libraries(ros_hardware_plant PUBLIC hardware_plant hardware_dds_plant ${catkin_LIBRARIES})
  target_link_libraries(ros_hardware_plant_cxx PUBLIC hardware_plant_cxx hardware_dds_plant ${catkin_LIBRARIES})
else()
  target_link_libraries(ros_hardware_plant PUBLIC hardware_plant ${catkin_LIBRARIES})
  target_link_libraries(ros_hardware_plant_cxx PUBLIC hardware_plant_cxx ${catkin_LIBRARIES})
  target_include_directories(ros_hardware_plant PUBLIC ${HARDWARE_PLANT_DIR}/src)
  target_include_directories(ros_hardware_plant_cxx PUBLIC ${HARDWARE_PLANT_DIR}/src)
endif()
target_include_directories(ros_hardware_plant PUBLIC  ${catkin_INCLUDE_DIRS}  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
target_include_directories(ros_hardware_plant_cxx PUBLIC  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
target_link_libraries(ros_hardware_plant PUBLIC yaml-cpp)
target_link_libraries(ros_hardware_plant_cxx PUBLIC yaml-cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hardware_communication_node.cpp)
add_executable(hardware_test src/tests/hardwareTest.cc)
target_link_libraries(hardware_test
  ros_hardware_plant
  ${catkin_LIBRARIES}
) 
target_include_directories(hardware_test PUBLIC 
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardware_cxx_test src/tests/hardwareTest.cc)
target_link_libraries(hardware_cxx_test
  ros_hardware_plant_cxx
  ${catkin_LIBRARIES}
) 
target_include_directories(hardware_cxx_test PUBLIC 
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardware_test_roban src/tests/hardwareTestRoban.cc)
target_link_libraries(hardware_test_roban
  ros_hardware_plant
  ${catkin_LIBRARIES}
) 
target_include_directories(hardware_test_roban PUBLIC 
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardware_node_jointcmd_test src/tests/hardware_node_jointcmd_test.cc)
target_link_libraries(hardware_node_jointcmd_test
  ros_hardware_plant_cxx
  ${catkin_LIBRARIES}
) 
target_include_directories(hardware_node_jointcmd_test PUBLIC 
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardwareSelfCheck src/tests/hardwareSelfCheck.cpp)
target_link_libraries(hardwareSelfCheck 
  hardware_plant_cxx
  ${catkin_LIBRARIES}
)
target_include_directories(hardwareSelfCheck PUBLIC 
  ${catkin_INCLUDE_DIRS}
)
install(TARGETS hardwareSelfCheck
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)




add_executable(ruiwo_motor_ros_py_test src/tests/ruiwo_controller_ros_test.cc)
add_executable(ruiwo_motor_ros_cxx_test src/tests/ruiwo_controller_ros_test.cc)

target_link_libraries(ruiwo_motor_ros_py_test ros_hardware_plant)
target_link_libraries(ruiwo_motor_ros_cxx_test ros_hardware_plant_cxx)
install(TARGETS ruiwo_motor_ros_cxx_test ruiwo_motor_ros_py_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# dexhand ros 
add_executable(dexhand_controller_ros_test src/tests/dexhand/dexhand_controller_test.cc)
target_link_libraries(dexhand_controller_ros_test ros_hardware_plant_cxx)
install(TARGETS dexhand_controller_ros_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_library(nodelet_hardware
  src/hardware_nodelet.cc
)
add_library(nodelet_hardware_cxx
  src/hardware_nodelet.cc
)
set_target_properties(nodelet_hardware PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
set_target_properties(nodelet_hardware_cxx PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)

add_dependencies(nodelet_hardware ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nodelet_hardware_cxx ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(nodelet_hardware
  ${catkin_LIBRARIES}
  ros_hardware_plant
)
target_link_libraries(nodelet_hardware_cxx
  ${catkin_LIBRARIES}
  ros_hardware_plant_cxx
)
target_include_directories(nodelet_hardware PUBLIC ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
target_include_directories(nodelet_hardware_cxx PUBLIC ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})

install(TARGETS nodelet_hardware nodelet_hardware_cxx
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ros_hardware_plant ros_hardware_plant_cxx
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES include/hardware_node/actuator_dynamics_config.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install (DIRECTORY plugins 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Motor Follow Test Sim Node
add_executable(motorFollowTestSim 
  src/tests/motorTest/motor_follow_test_sim.cpp
  src/tests/motorTest/motor_follow_test_core.cpp
)
target_link_libraries(motorFollowTestSim
  hardware_plant_cxx
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)
target_include_directories(motorFollowTestSim PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../hardware_plant/src
  ${PROJECT_SOURCE_DIR}/src/tests/motorTest
)

install(TARGETS motorFollowTestSim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Motor Follow Test Node
add_executable(motorFollowTest 
  src/tests/motorTest/motor_follow_test.cpp
  src/tests/motorTest/motor_follow_test_core.cpp
)
target_link_libraries(motorFollowTest
  hardware_plant_cxx
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
) 
target_include_directories(motorFollowTest PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../hardware_plant/src
  ${PROJECT_SOURCE_DIR}/src/tests/motorTest
)

install(TARGETS motorFollowTest
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Ankle Solver V17 Test Node
add_executable(ankleSolverV17Test 
  src/tests/ankleTest/ankle_solver_v17_test.cpp
)
target_link_libraries(ankleSolverV17Test
  hardware_plant_cxx
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
) 
target_include_directories(ankleSolverV17Test PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../hardware_plant/src
  ${PROJECT_SOURCE_DIR}/src/tests/ankleTest
)
# 输出可执行文件名为 ankle_solver_v17_test，便于 rosrun/roslaunch 调用
set_target_properties(ankleSolverV17Test PROPERTIES OUTPUT_NAME ankle_solver_v17_test PREFIX "")

install(TARGETS ankleSolverV17Test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

# add_executable(hand_ctrl_test src/tests/hand_controller_test.cc)
# target_link_libraries(hand_ctrl_test hardware_lib)

add_executable(dexhand_ctrl_test src/tests/dexhand_controller_test.cc)
target_link_libraries(dexhand_ctrl_test
  ros_hardware_plant
  ${catkin_LIBRARIES}
)
target_include_directories(dexhand_ctrl_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
)
install(TARGETS dexhand_ctrl_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_executable(roban_arm_with_revo2_test src/tests/roban_arm_with_revo2_test.cc)
target_link_libraries(roban_arm_with_revo2_test
  ros_hardware_plant
  ${catkin_LIBRARIES}
)
target_include_directories(roban_arm_with_revo2_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
)
install(TARGETS roban_arm_with_revo2_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)



## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hardware_communication.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
