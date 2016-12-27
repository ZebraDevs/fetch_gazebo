^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
