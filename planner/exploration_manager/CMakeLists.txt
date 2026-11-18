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

## Exploration Library: waypoint (lawnmower) and frontier exploration
add_library(exploration_manager
  src/waypoint_exploration.cpp
  src/frontier_map.cpp
  src/frontier_exploration.cpp
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_frontier_map test/test_frontier_map.cpp)
  if(TARGET test_frontier_map)
    target_link_libraries(test_frontier_map exploration_manager ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS exploration_node exploration_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
