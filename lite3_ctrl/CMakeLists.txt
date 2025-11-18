cmake_minimum_required(VERSION 3.0.2)
project(lite3_ctrl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lite3_velocity_controller
  src/lite3_velocity_controller.cpp
)

target_link_libraries(lite3_velocity_controller
  ${catkin_LIBRARIES}
)
