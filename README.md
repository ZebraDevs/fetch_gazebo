# Fetch Gazebo

This repository contains the Gazebo simulation for Fetch Robotics Fetch and
Freight Research Edition Robots.

Please note the _branch_: the default branch in GitHub is **gazebo9**, and
this branch is **gazebo11**.
1. **gazebo11** should be used with **ROS Noetic** and **Ubuntu 20.04**
2. **gazebo9** should be used with **ROS Melodic** and **Ubuntu 18.04**
3. **gazebo7** should be used with **ROS Kinetic** and **Ubuntu 16.04** (not supported on hardware)
4. **gazebo2** should be used with **ROS Indigo** and **Ubuntu 14.04** (EOL)

## Tutorial & Documentation

Please refer to our documentation page: http://docs.fetchrobotics.com/gazebo.html

## Launch it on ROSDS

The Fetch Gazebo packages can be run in the cloud, through an external service ROSDS, provided by TheConstruct.
http://docs.fetchrobotics.com/gazebo.html#launch-it-on-rosds

## ROS Buildfarm Release
 
Fetch Gazebo Package | Kinetic Source | Kinetic Debian | Melodic Source | Melodic Debian
-------------------- | -------------- | -------------- | -------------- | --------------
fetch_gazebo | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__fetch_gazebo__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__fetch_gazebo__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__fetch_gazebo__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__fetch_gazebo__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_gazebo__ubuntu_bionic__source)](http://build.ros.org/view/Mbin_uB64/job/Msrc_uB__fetch_gazebo__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_gazebo__ubuntu_bionic_amd64__binary)](http://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__fetch_gazebo__ubuntu_bionic_amd64__binary/) |
fetch_gazebo_demo | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__fetch_gazebo_demo__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__fetch_gazebo_demo__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__fetch_gazebo_demo__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__fetch_gazebo_demo__ubuntu_xenial_amd64__binary/) | | | | |
fetchit_challenge | | | | | | |

## ROS Buildfarm Devel

Fetch Gazebo Package | Melodic Devel
-------------------- | -------------
fetch_gazebo | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__fetch_gazebo__ubuntu_bionic_amd64)](http://build.ros.org/view/Mdev/job/Mdev__fetch_gazebo__ubuntu_bionic_amd64/)
fetch_gazebo_demo | | | |
fetchit_challenge | | | |

