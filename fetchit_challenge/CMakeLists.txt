cmake_minimum_required(VERSION 2.8.3)
project(fetchit_challenge)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  fetch_gazebo
  gazebo_ros
  robot_state_publisher
  rospy
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
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

