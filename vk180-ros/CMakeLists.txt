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
  cv_bridge
  pcl_ros
  pcl_conversions
)

## OpenCV
find_package(OpenCV REQUIRED)

## Eigen
find_package(Eigen3 REQUIRED)

## PCL
find_package(PCL REQUIRED)

option(ENABLE_VK180_SDK "Build VK180/Vilota hardware integration binaries" ON)

if(ENABLE_VK180_SDK)
  set(CapnProto_DIR "/opt/vilota/lib/cmake/CapnProto")
  set(vk_sdk_DIR "/opt/vilota/lib/cmake/vk_sdk")
  find_package(vk_sdk CONFIG REQUIRED)
else()
  message(STATUS "VK180 SDK support disabled: building launch/config-only package")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include  # No headers to export
  # LIBRARIES vk180_ros   # No shared library built
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge pcl_ros pcl_conversions
  DEPENDS OpenCV PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

if(ENABLE_VK180_SDK)
  add_executable(receive src/utils/AllTopicReceiver.cpp)
  add_executable(odom src/utils/odom.cpp)
  add_executable(disparity src/utils/disparity.cpp)
  add_executable(sensor_extrinsics src/utils/sensor_extrinsics.cpp)
  add_executable(depth_conversion_test src/utils/depth_conversion_test.cpp)

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
  target_link_libraries(sensor_extrinsics
    PRIVATE
      CapnProto::capnp
      vilota::vk_sdk
  )
  target_link_libraries(depth_conversion_test
    PRIVATE
      CapnProto::capnp
      vilota::vk_sdk
  )

  add_executable(ecal_ros_bridge src/nodes/ecal_ros_bridge_node.cpp)
  target_link_libraries(ecal_ros_bridge
    PRIVATE
      ${catkin_LIBRARIES}
      CapnProto::capnp
      vilota::vk_sdk
  )

  add_executable(vk180_planner_bridge src/nodes/vk180_planner_bridge.cpp)
  target_link_libraries(vk180_planner_bridge
    PRIVATE
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${PCL_LIBRARIES}
      CapnProto::capnp
      vilota::vk_sdk
  )
endif()

#############
## Install ##
#############

if(ENABLE_VK180_SDK)
  install(TARGETS receive odom disparity sensor_extrinsics depth_conversion_test ecal_ros_bridge vk180_planner_bridge
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
