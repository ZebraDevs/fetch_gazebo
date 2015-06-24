cmake_minimum_required(VERSION 2.8.3)
project(fetch_gazebo)

find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    angles
    control_toolbox
    gazebo_ros
    robot_controllers
    robot_controllers_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    gazebo_ros
    robot_controllers
    robot_controllers_interface
  DEPENDS
    gazebo
  LIBRARIES
    fetch_gazebo_plugin
  INCLUDE_DIRS
    include
)

add_library(fetch_gazebo_plugin src/plugin.cpp)
target_link_libraries(fetch_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS fetch_gazebo_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  
)

install(
  PROGRAMS scripts/prepare_simulated_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config include launch robots worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
