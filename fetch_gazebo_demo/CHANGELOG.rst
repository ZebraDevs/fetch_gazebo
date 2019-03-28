^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_gazebo_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.2 (2019-03-28)
------------------

0.8.1 (2019-03-26)
------------------
* This release fast forwarded the gazebo7 branch and reverted the breaking changes from gazebo9
* Merge pull request `#41 <https://github.com/fetchrobotics/fetch_gazebo/issues/41>`_ from moriarty/gazebo7-new
  Fast Forward Gazebo7, and revert breaking changes for Gazebo 9
  * Related to `#39 <https://github.com/fetchrobotics/fetch_gazebo/issues/39>`_ which was a similar fast forward and revert for Gazebo 2.
  * Also cherry-picks `#43 <https://github.com/fetchrobotics/fetch_gazebo/issues/43>`_ and `#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_
* [package.xml] adds: license(BSD), author, maintainers (`#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_)
  * [package.xml] update maintainers
  * [fetchit_challenge] adds license(BSD), author, maintainers
  * [fetchit_challenge] add <url> tags to package.xml
  This cherry-picks ccb5ad3f40cedc2944a9b3d6fab8d75239a9997b (`#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_)
* Reverts and reverted changes:
  * Revert "Corrected Bug for new Melodic MoveIt Version"
    This reverts commit 674f28175ace0c41f6a248abf82cab2b57829d95.
    See PR `#26 <https://github.com/fetchrobotics/fetch_gazebo/issues/26>`_ and PR `#32 <https://github.com/fetchrobotics/fetch_gazebo/issues/32>`_
  * Merge pull request `#32 <https://github.com/fetchrobotics/fetch_gazebo/issues/32>`_ from RDaneelOlivav/gazebo9
    FetchIt Challenge Package addition and minor melodic Moveit Fix
  * Corrected Bug for new Melodic MoveIt Version
* Added correction suggested in pick and place demo script and new tables
* Contributors: Alexander Moriarty, Miguel Angel Rodríguez

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
