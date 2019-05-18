cmake_minimum_required(VERSION 3.5.1)
project(fetchit_challenge)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES SchunkMachine.action)
add_action_files(DIRECTORY action FILES SickCamera.action)
generate_messages(DEPENDENCIES actionlib_msgs)
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

