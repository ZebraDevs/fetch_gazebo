^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetchit_challenge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.2 (2019-08-06)
------------------
* Merge pull request `#93 <https://github.com/fetchrobotics/fetch_gazebo/issues/93>`_ from fetchrobotics/sick
  Adds sick camera interface
* Corrected Inertias to solve standing bolts (`#92 <https://github.com/fetchrobotics/fetch_gazebo/issues/92>`_)
* [Schunk Machine] Updates appearance and behaviour (`#89 <https://github.com/fetchrobotics/fetch_gazebo/issues/89>`_)
  - Added new Shunk with updated appearance
  - Actuated chuck and action for open and close and start operation and end operation
  - Decoupled shunk controllers from action
  NOTE: this breaks previous behaviour because there is a new action to start and end operation.
* Merge pull request `#88 <https://github.com/fetchrobotics/fetch_gazebo/issues/88>`_ from ben-fido/gazebo9
  Fix TF bug, spawn Schunk Machine in separate namespace
* Missing utility cart (`#85 <https://github.com/fetchrobotics/fetch_gazebo/issues/85>`_)
  - Updated New Coloured cart support due to faulty merge
  - Added 3 carts in the table
* Added blue and grey parts (`#79 <https://github.com/fetchrobotics/fetch_gazebo/issues/79>`_)
* Update tables walls and tables (`#78 <https://github.com/fetchrobotics/fetch_gazebo/issues/78>`_)
  - Added Arena for Fetch Remote Lab and Montreal 2019 with updated walls and dimensions
  - Added new Tables and positions in the updated arrangement
  - Added further changes to have a more intense light scenario
* Additional kits gazebo world (`#76 <https://github.com/fetchrobotics/fetch_gazebo/issues/76>`_)
  - Added colour caddy
  - Fixed error of naming old model
* Schunk Machine interface. (`#72 <https://github.com/fetchrobotics/fetch_gazebo/issues/72>`_)
* Contributors: Alex Moriarty, Niharika Arora, Miguel Angel Rodríguez, Sarah Elliott, Benjamin Jarvhi

0.9.1 (2019-04-04)
------------------
* Cleanup dependencies (`#68 <https://github.com/fetchrobotics/fetch_gazebo/issues/68>`_)
* cmake_minimum_required(VERSION 3.7.2) & C++14 (`#67 <https://github.com/fetchrobotics/fetch_gazebo/issues/67>`_)
    - Bump the minimum CMake version, to fix a CMake warning.
    - Remove explicit CMAKE_CXX_FLAGS c++0x because the default is now C++14
* [FetchIt!] Tapered large gear (`#63 <https://github.com/fetchrobotics/fetch_gazebo/issues/63>`_)
* Contributors: Alex Moriarty, Sarah Elliott, Miguel Angel Rodríguez

0.9.0 (2019-03-28)
------------------
* [FetchIt!] Don't find_package an exec_depend
* [package.xml] sync versions to 0.8.0
* [fetchit] catkin depends and installation (`#58 <https://github.com/fetchrobotics/fetch_gazebo/issues/58>`_)
    The models, scripts and launch files were not installed. Fixes `#57 <https://github.com/fetchrobotics/fetch_gazebo/issues/57>`_
* [FetchIt!] fix the stl collision scales (`#60 <https://github.com/fetchrobotics/fetch_gazebo/issues/60>`_)
    Added rescaled stls with corrected origins that generated issue that seems they were no collisions but in fact they were decentered and huge.
    This fixes `#56 <https://github.com/fetchrobotics/fetch_gazebo/issues/56>`_
* [FetchIt!] Model updates for new largeGear (`#51 <https://github.com/fetchrobotics/fetch_gazebo/issues/51>`_)
    * [FetchIt!] STL updates for new largeGear
    * This fixes `#50 <https://github.com/fetchrobotics/fetch_gazebo/issues/50>`_ in part, the stl files
    * These changes correspond to `fetchrobotics/fetchit#2 <https://github.com/fetchrobotics/fetchit/issues/2>`_
    * Added updated dae versions with rescaling to correct dimensions and same colours as previous versions
* [package.xml] adds: license(BSD), author, maintainers (`#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_)
    * [package.xml] update maintainers
    * [fetchit_challenge] adds license(BSD), author, maintainers
    * [fetchit_challenge] add <url> tags to package.xml
* Merge pull request `#38 <https://github.com/fetchrobotics/fetch_gazebo/issues/38>`_ from RDaneelOlivav/gazebo9
    Invisible table and gear Fix, creation of simpler simulation environment, creation of different lighting conditions launches
https://github.com/fetchrobotics/power_msgs/pull/10* Merge pull request `#32 <https://github.com/fetchrobotics/fetch_gazebo/issues/32>`_ from RDaneelOlivav/gazebo9
    FetchIt Challenge Package addition and minor melodic Moveit Fix
* Added fetchit Challenge package files
* Contributors: Alexander Moriarty, Miguel Angel Rodríguez

0.8.0 (2016-12-27)
------------------
* fetchit_challenge was newly added after 0.8.0

