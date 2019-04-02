cmake_minimum_required(VERSION 3.7.2)
project(fetch_gazebo_demo)

find_package(catkin)
catkin_package()

install(
  PROGRAMS
    scripts/demo.py
    scripts/pick_place_demo.py
    scripts/tests_arm_movements.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch maps config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
