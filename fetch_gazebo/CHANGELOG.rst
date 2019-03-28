^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2019-03-28)
------------------

0.7.2 (2019-03-26)
------------------
* This release fast-forwarded the gazebo2 branch, and reverted breaking changes from:
  gazebo5, gazebo7 and gazebo9
* [package.xml] adds: license(BSD), author, maintainers (#45) (#55)
  * [package.xml] update maintainers
  * [fetchit_challenge] adds license(BSD), author, maintainers
  * [fetchit_challenge] add <url> tags to package.xml
* [CMake/Catkin] fetch_gazebo install headers (#44)
  catkin_make_isolated --install would fail because we were not installing the header files
  This cherry-picks PR #43
  This fixes #42
* reverts and reverted changes:
  * Revert "update rosdeps for gazebo5"
    This reverts commit 6ee207e53cc1e8fe3904720c0bfd7cb032997ca5.
  * Revert "update changelogs" <0.8.0>
    This reverts commit ab8607adc9b0120c09869fb5c84d320c9c020d3e.
  * Revert "0.8.0"
    This reverts commit 3aa624857adf4142c0ca3708b028a3b05d8737ad.
  * Revert "updates for gazebo7"
    This reverts commit 545769af5fd77aef83e0c182f74e577c983a4ea7.
  * Revert [gazebo9] Merge pull request #24
    "Merge pull request #24 from mikeferguson/gazebo9"
    This reverts commit eecb032ded6f675ded81e477087218a2079944fa, reversing
    changes made to 545769af5fd77aef83e0c182f74e577c983a4ea7.
  * Revert "Merge pull request #25 ... rosdep"
    Merge pull request #25 from stfuchs/fix/melodic-rosdep
    This reverts commit fce68bf44ce2d214edd11f705e61590efbafa1fe, reversing
    changes made to eecb032ded6f675ded81e477087218a2079944fa.
  * Revert PR #29 "[CMake][package] ... REP-140"
    [CMake][package] Clean CMake warnings and REP-140 (#29)"
    This reverts commit 36bf32339acb8763360e63b76b45927c1032ae61.
  * [CMake][package] Clean CMake warnings and REP-140 (#29)
    This cleans up some, but misses one warning- might be coming from gazebo
    itself.
    Also moved to package.xml format 2
  * Merge pull request #25 from stfuchs/fix/melodic-rosdep
    change rosdep from gazebo7 to gazebo9
  * change rosdep from gazebo7 to gazebo9
  * Merge pull request #24 from mikeferguson/gazebo9
    fixes for gazebo9 (melodic)
  * fixes for gazebo9 (melodic)
  * updates for gazebo7
* add arg z/yaw for spawning robot
* Contributors: Alexander Moriarty, Michael Ferguson, Russell Toris, Steffen Fuchs, Yuki Furuta

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
