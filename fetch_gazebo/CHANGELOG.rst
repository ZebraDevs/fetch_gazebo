^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.2 (2019-08-06)
------------------

0.9.1 (2019-04-04)
------------------
* Cleanup dependencies (`#68 <https://github.com/fetchrobotics/fetch_gazebo/issues/68>`_)
* cmake_minimum_required(VERSION 3.7.2) & C++14 (`#67 <https://github.com/fetchrobotics/fetch_gazebo/issues/67>`_)
    - Bump the minimum CMake version, to fix a CMake warning.
    - Remove explicit CMAKE_CXX_FLAGS c++0x because the default is now C++14
* Contributors: Alex Moriarty

0.9.0 (2019-03-28)
------------------
* Improved wheel frictions (`#59 <https://github.com/fetchrobotics/fetch_gazebo/issues/59>`_)
* fetch_gazebo: add casters to fetch.gazebo.xacro (`#49 <https://github.com/fetchrobotics/fetch_gazebo/issues/49>`_)
    This adds 4 casters to the gazebo model, and updates the physics params.
    This should fix `#31 <https://github.com/fetchrobotics/fetch_gazebo/issues/31>`_ and fix `#35 <https://github.com/fetchrobotics/fetch_gazebo/issues/35>`_
    This is related to `#37 <https://github.com/fetchrobotics/fetch_gazebo/issues/37>`_
* [package.xml] adds: license(BSD), author, maintainers (`#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_)
    * [package.xml] update maintainers
    * [fetchit_challenge] adds license(BSD), author, maintainers
    * [fetchit_challenge] add <url> tags to package.xml
* [CMake/Catkin] fetch_gazebo install headers (`#43 <https://github.com/fetchrobotics/fetch_gazebo/issues/43>`_)
    catkin_make_isolated --install would fail because we were not installing the header files
    This fixes `#42 <https://github.com/fetchrobotics/fetch_gazebo/issues/42>`_
* [CMake][package] Clean CMake warnings and REP-140 (`#29 <https://github.com/fetchrobotics/fetch_gazebo/issues/29>`_)
    This cleans up some, but misses one warning- might be coming from gazebo itself.
    Also moved to package.xml format 2
* Merge pull request `#25 <https://github.com/fetchrobotics/fetch_gazebo/issues/25>`_ from stfuchs/fix/melodic-rosdep
    change rosdep from gazebo7 to gazebo9
* Merge pull request `#24 <https://github.com/fetchrobotics/fetch_gazebo/issues/24>`_ from mikeferguson/gazebo9
    fixes for gazebo9 (melodic)
* updates for gazebo7
* Contributors: Alex Moriarty, Michael Ferguson, Miguel Angel Rodríguez, Russell Toris, Steffen Fuchs

0.8.0 (2016-12-27)
------------------
* update rosdeps for gazebo5
* add arg z/yaw for spawning robot
* Contributors: Michael Ferguson, Yuki Furuta

0.7.1 (2016-02-27)
------------------
* robot launch files publish base_scan_raw
* working version of the merged demos
* Contributors: Di Sun, Andrew Vaziri

0.7.0 (2015-11-04)
------------------
* add run depend on gazebo_plugins
* Contributors: Michael Ferguson

0.6.2 (2015-09-13)
------------------
* Fixed number of samples for the laser scanner
  Real laser scan has 662 samples
* Update test_zone.sdf, had two `</world>` tag
* Contributors: Alex Henning, Kei Okada

0.6.1 (2015-06-28)
------------------
* add headless argument to launch files
* Contributors: Michael Ferguson

0.6.0 (2015-06-24)
------------------
* add check to prevent running prepare script on a real robot
* Contributors: Michael Ferguson

0.5.1 (2015-06-23)
------------------
* forget install models
* Contributors: Kei Okada

0.5.0 (2015-06-13)
------------------
* apply filter to velocity
* implement servo loop
  * position wraps velocity controller
  * velocity limits are now loaded for continuous joints
  * retuned all but base motors
* Contributors: Michael Ferguson

0.4.4 (2015-06-12)
------------------
* use centering controller for gripper
* Contributors: Michael Ferguson

0.4.3 (2015-06-10)
------------------
* base_link collision mesh is updated, fix laser min range
* Change retrieval of effort limits/continuous state from parameters to URDF
* Contributors: Michael Ferguson, Michael Hwang

0.4.2 (2015-06-06)
------------------
* fix install
* Contributors: Michael Ferguson

0.4.1 (2015-06-05)
------------------

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
