cmake_minimum_required(VERSION 3.0.2)
project(bezier_exploration)
add_definitions(-std=c++14 -Wall -Wextra)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros_msgs
  message_generation
  tf2
  tf2_geometry_msgs
  tf2_ros
  octomap_ros
)

add_message_files(
  DIRECTORY
    msg
  FILES
    TreeNode.msg
    BezierTraj.msg
    gpMsgs.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    GetGain.srv
    InitService.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    bezier_exploration
  CATKIN_DEPENDS 
    roscpp rospy std_msgs message_runtime tf2
    tf2_geometry_msgs
    tf2_ros
    octomap_ros
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(bezier_exploration src/bezier_exploration.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bezier_exploration
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

## Declare a C++ executable
add_executable(bezier_exploration_node src/bezier_exploration_node.cpp)
add_executable(gain_evaluator_node src/gain_evaluator.cpp)
add_executable(bezier_exploration_test src/bezier_exploration_test.cpp)
add_executable(bezier_exploration_test_real src/bezier_exploration_test_real.cpp)
add_executable(map_evaluator src/map_evaluator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bezier_exploration_node
  ${catkin_LIBRARIES}
  bezier_exploration
)

target_link_libraries(gain_evaluator_node
  ${catkin_LIBRARIES}
  bezier_exploration
)

target_link_libraries(bezier_exploration_test
  ${catkin_LIBRARIES}
  bezier_exploration
)

target_link_libraries(bezier_exploration_test_real
  ${catkin_LIBRARIES}
  bezier_exploration
)

target_link_libraries(map_evaluator
  ${catkin_LIBRARIES}
  bezier_exploration
)

## Declare a Python executable
catkin_install_python(
  PROGRAMS src/gain_regressor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)