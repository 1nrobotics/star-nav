cmake_minimum_required(VERSION 3.0.2)
project(exploration_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  plan_env
  traj_utils
  bspline_opt
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES exploration_manager
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    pcl_ros
    plan_env
    traj_utils
    bspline_opt
    quadrotor_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../plan_manage/include
  ${PROJECT_SOURCE_DIR}/../path_searching/include
)

## Exploration Manager Library
add_library(exploration_manager
  src/frontier_finder.cpp
  src/exploration_fsm.cpp
)

target_link_libraries(exploration_manager
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Exploration Node
add_executable(exploration_node
  src/exploration_node.cpp
)

target_link_libraries(exploration_node
  exploration_manager
  ${catkin_LIBRARIES}
)
