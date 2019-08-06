^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_gazebo_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.2 (2019-08-06)
------------------

0.9.1 (2019-04-04)
------------------
* cmake_minimum_required(VERSION 3.7.2) & C++14 (`#67 <https://github.com/fetchrobotics/fetch_gazebo/issues/67>`_)
    - Bump the minimum CMake version, to fix a CMake warning.
    - Remove explicit CMAKE_CXX_FLAGS c++0x because the default is now C++14
* Contributors: Alex Moriarty

0.9.0 (2019-03-28)
------------------
* Improved pick place frictions (`#59 <https://github.com/fetchrobotics/fetch_gazebo/issues/59>`_)
    * Added higher frictions to avoid the base movements when moving the arm, and also added a simple pick and place environment for fast testing
    * Added a script that can be used to test basic movement with fetch arm and gripper and also test that the friction is enough that movement of the arm wont affect it
    * [gazebo_demo] Install script and depends
* [package.xml] adds: license(BSD), author, maintainers (`#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_)
    * [package.xml] update maintainers
    * [fetchit_challenge] adds license(BSD), author, maintainers
    * [fetchit_challenge] add <url> tags to package.xml
* Merge pull request `#32 <https://github.com/fetchrobotics/fetch_gazebo/issues/32>`_ from RDaneelOlivav/gazebo9
    FetchIt Challenge Package addition and minor melodic Moveit Fix
* Added correction suggested in pick and place demo script and new tables
* Corrected Bug for new Melodic MoveIt Version
* Contributors: Alex Moriarty, Miguel Angel Rodríguez

0.8.0 (2016-12-27)
------------------
* Add rviz config for pickplace demo
* Contributors: Kentaro Wada

0.7.1 (2016-02-27)
------------------
* improve parameterization of demos
* Contributors: Di Sun, Michael Ferguson

0.7.0 (2015-11-04)
------------------

0.6.2 (2015-09-13)
------------------

0.6.1 (2015-06-28)
------------------

0.6.0 (2015-06-24)
------------------

0.5.1 (2015-06-23)
------------------

0.5.0 (2015-06-13)
------------------
* force gripper fully open for demo
* Contributors: Michael Ferguson

0.4.4 (2015-06-12)
------------------

0.4.3 (2015-06-10)
------------------

0.4.2 (2015-06-06)
------------------

0.4.1 (2015-06-05)
------------------
* update launch files for package name changes
* Contributors: Michael Ferguson

0.4.0 (2015-06-05)
------------------
* split demos into separate package
* Contributors: Michael Ferguson

0.3.2 (2015-06-04)
------------------
* add depend on angles
* Contributors: Michael Ferguson

0.3.1 (2015-06-03)
------------------
* add simulation model for freight
* add navigation configuration for simulated env
* update for timer-based base controller
* Contributors: Michael Ferguson

0.3.0 (2015-06-03)
------------------
* init from preview repo
* Contributors: Michael Ferguson
