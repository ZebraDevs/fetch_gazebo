^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.2 (2019-08-06)
------------------

0.9.1 (2019-04-04)
------------------
* catkin_make only supports cmake 2.8.3 for meta-pkg (`#70 <https://github.com/fetchrobotics/fetch_gazebo/issues/70>`_)
    This fixes `#69 <https://github.com/fetchrobotics/fetch_gazebo/issues/69>`_
* cmake_minimum_required(VERSION 3.7.2) & C++14 (`#67 <https://github.com/fetchrobotics/fetch_gazebo/issues/67>`_)
    - Bump the minimum CMake version, to fix a CMake warning.
    - Remove explicit CMAKE_CXX_FLAGS c++0x because the default is now C++14
* Contributors: Alex Moriarty

0.9.0 (2019-03-28)
------------------
* [fetch_simulation] fix filename & CMakeLists.txt
    cherry-pick 730051a64df815ed61b6d3fb012fb5a225ae2e2e
* [fetch_simulation] new meta-package (`#52 <https://github.com/fetchrobotics/fetch_gazebo/issues/52>`_)
    A meta-package makes installing things easier and they're used with
    wiki.ros.org and index.ros.org to automatically group and link things.
* Contributors: Alexander Moriarty

0.8.0 (2016-12-27)
------------------
* Meta-package fetch_simulation was newly added after 0.8.0

