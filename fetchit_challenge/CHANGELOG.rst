^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetchit_challenge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2019-03-28)
------------------
* Merge pull request `#64 <https://github.com/fetchrobotics/fetch_gazebo/issues/64>`_ from moriarty/indigo-backports
* [FetchIt!] Don't find_package an exec_depend
* [fetchit] catkin depends and installation (`#58 <https://github.com/fetchrobotics/fetch_gazebo/issues/58>`_)
  The models, scripts and launch files were not installed. Fixes `#57 <https://github.com/fetchrobotics/fetch_gazebo/issues/57>`_
* [FetchIt!] fix the stl collision scales (`#60 <https://github.com/fetchrobotics/fetch_gazebo/issues/60>`_)
  Added rescaled stls with corrected origins that generated issue that seems they were no collisions but in fact they were decentered and huge.
  This fixes `#56 <https://github.com/fetchrobotics/fetch_gazebo/issues/56>`_
* [FetchIt!] Model updates for new largeGear (`#51 <https://github.com/fetchrobotics/fetch_gazebo/issues/51>`_)
  * This fixes `#50 <https://github.com/fetchrobotics/fetch_gazebo/issues/50>`_ in part, the stl files
  * These changes correspond to `fetchrobotics/fetchit#2 <https://github.com/fetchrobotics/fetchit/issues/2>`_
  * Added updated dae versions with rescaling to correct dimensions and same colours as previous versions
* Contributors: Alexander Moriarty, Carl Saldanha, RDaneelOlivav

0.7.2 (2019-03-26)
------------------
* [package.xml] sync versions to 0.7.1 for indigo
  - fetch_simulation was backported from 0.8.0
  - fetchit_challenge didn't have a version set
* [package.xml] adds: license(BSD), author, maintainers (#45) (#55)
  * [package.xml] update maintainers
  * [fetchit_challenge] adds license(BSD), author, maintainers
  * [fetchit_challenge] add <url> tags to package.xml
* Merge pull request #39 from moriarty/gazebo2
  Fast Forward Gazebo2, and revert Gazebo 5,7 & 9
  Not all commits in this branch will compile with Gazebo2 and Indigo.
* Merge pull request #38 from RDaneelOlivav/gazebo9
  Invisible table and gear Fix, creation of simpler simulation environment, creation of different lighting conditions launches
* Merge pull request #32 from RDaneelOlivav/gazebo9
  FetchIt Challenge Package addition and minor melodic Moveit Fix
* Added fetchit Challenge package files
* Contributors: Alexander Moriarty, Miguel Angel Rodríguez

0.7.1 (2016-02-27)
------------------
* fetchit_challenge was newly added after 0.7.1
