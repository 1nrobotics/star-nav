cmake_minimum_required(VERSION 3.0.2)
project(terrain_analysis)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  pcl_ros
  pcl_conversions
)
find_package(PCL REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(terrainAnalysis src/terrainAnalysis.cpp)
target_link_libraries(terrainAnalysis ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS terrainAnalysis
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
