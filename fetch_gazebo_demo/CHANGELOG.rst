^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_gazebo_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Reverts and reverted changes:
  * Revert "update changelogs" <0.8.0>
    This reverts commit ab8607adc9b0120c09869fb5c84d320c9c020d3e.
  * Revert "0.8.0"
    This reverts commit 3aa624857adf4142c0ca3708b028a3b05d8737ad.
  * Revert "Corrected Bug for new Melodic MoveIt Version"
    This reverts commit 674f28175ace0c41f6a248abf82cab2b57829d95.
    See PR #26 and PR #32
  * Corrected Bug for new Melodic MoveIt Version
* Merge pull request #32 from RDaneelOlivav/gazebo9
  FetchIt Challenge Package addition and minor melodic Moveit Fix
  (partially reverted)
* Added correction suggested in pick and place demo script and new tables
* Add rviz config for pickplace demo
* Contributors: Alexander Moriarty, Kentaro Wada, Miguel Angel Rodríguez

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
