^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetchit_challenge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.2 (2019-03-28)
------------------
* Merge pull request `#65 <https://github.com/fetchrobotics/fetch_gazebo/issues/65>`_ from moriarty/kinetic-backports
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
* Contributors: Alexander Moriarty, Carl Saldanha, Miguel Angel Rodríguez

0.8.1 (2019-03-26)
------------------
* [package.xml] sync versions to 0.8.0 for kinetic
* Merge pull request `#41 <https://github.com/fetchrobotics/fetch_gazebo/issues/41>`_ from moriarty/gazebo7-new
    * Fast Forward Gazebo7, and revert breaking changes for Gazebo 9
    * Related to `#39 <https://github.com/fetchrobotics/fetch_gazebo/issues/39>`_ which was a similar fast forward and revert for Gazebo 2.
    * Also cherry-picks `#43 <https://github.com/fetchrobotics/fetch_gazebo/issues/43>`_ and `#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_
* [package.xml] adds: license(BSD), author, maintainers (`#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_)
    * [package.xml] update maintainers
    * [fetchit_challenge] adds license(BSD), author, maintainers
    * [fetchit_challenge] add <url> tags to package.xml
  This cherry-picks ccb5ad3f40cedc2944a9b3d6fab8d75239a9997b (`#45 <https://github.com/fetchrobotics/fetch_gazebo/issues/45>`_)
* Merge pull request `#38 <https://github.com/fetchrobotics/fetch_gazebo/issues/38>`_ from RDaneelOlivav/gazebo9
  Invisible table and gear Fix, creation of simpler simulation environment, creation of different lighting conditions launches
* Merge pull request `#32 <https://github.com/fetchrobotics/fetch_gazebo/issues/32>`_ from RDaneelOlivav/gazebo9
  FetchIt Challenge Package addition and minor melodic Moveit Fix
* Added fetchit Challenge package files
* Contributors: Alexander Moriarty, Miguel Angel Rodríguez

0.8.0 (2016-12-27)
------------------
* fetchit_challenge was newly added after 0.8.0
