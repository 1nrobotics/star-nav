cmake_minimum_required(VERSION 3.0.2)
project(vk180_ros)

## Compile as C++17
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## VK180 eCAL SDK
set(CapnProto_DIR "/opt/vilota/lib/cmake/CapnProto")
set(vk_sdk_DIR "/opt/vilota/lib/cmake/vk_sdk")
find_package(vk_sdk CONFIG REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include  # No headers to export
  # LIBRARIES vk180_ros   # No shared library built
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs
  # DEPENDS               # External dependencies would go here (like vk_sdk, but it's found separately)
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Original eCAL test executables
add_executable(receive AllTopicReceiver.cpp)
add_executable(odom odom.cpp)
add_executable(disparity disparity.cpp)

target_link_libraries(receive 
  PRIVATE 
    CapnProto::capnp 
    vilota::vk_sdk
)
target_link_libraries(odom 
  PRIVATE 
    CapnProto::capnp 
    vilota::vk_sdk
)
target_link_libraries(disparity 
  PRIVATE 
    CapnProto::capnp 
    vilota::vk_sdk
)

## ROS bridge node
add_executable(ecal_ros_bridge src/ecal_ros_bridge_node.cpp)
target_link_libraries(ecal_ros_bridge 
  PRIVATE
    ${catkin_LIBRARIES}
    CapnProto::capnp 
    vilota::vk_sdk
)
