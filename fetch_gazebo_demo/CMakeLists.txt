cmake_minimum_required(VERSION 2.8.3)
project(fetch_gazebo_demo)

find_package(catkin)
catkin_package()

install(
  PROGRAMS scripts/demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
