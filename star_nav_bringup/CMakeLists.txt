cmake_minimum_required(VERSION 3.0.2)
project(star_nav_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/terrain_filter.py scripts/cmu_waypoint_router.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/sim_smoke.test)
endif()
