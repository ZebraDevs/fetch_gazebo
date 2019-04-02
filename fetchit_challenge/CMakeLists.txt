cmake_minimum_required(VERSION 3.7.2)
project(fetchit_challenge)

find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############

install(
  PROGRAMS
    scripts/spawn_assembly_delayed.sh
    scripts/spawn_assembly_delayed_simple.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch models urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

