cmake_minimum_required(VERSION 3.0.2)
project(mujoco_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  rosgraph_msgs
  geometry_msgs
  quadrotor_msgs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

catkin_package()

add_executable(mujoco_bridge_node src/mujoco_bridge_node.cpp)
target_include_directories(mujoco_bridge_node PRIVATE ${catkin_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS})
target_link_libraries(mujoco_bridge_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
target_compile_options(mujoco_bridge_node PRIVATE -Wall -Wextra)
add_dependencies(mujoco_bridge_node ${catkin_EXPORTED_TARGETS})

install(TARGETS mujoco_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
