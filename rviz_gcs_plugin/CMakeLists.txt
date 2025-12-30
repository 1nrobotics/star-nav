cmake_minimum_required(VERSION 2.8.3)
project(rviz_gcs_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  geometry_msgs
)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

catkin_package(
  CATKIN_DEPENDS roscpp rviz std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(${PROJECT_NAME}
  src/gcs_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)