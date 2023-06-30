^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2_tiago
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2023.1.0 (2023-06-29)
------------------
* Added support for Navigation2 in Iron.
* Clean simulation reset in launch file.
* Added new world, resources and launch file to start the TIAGo with real robot configuration.
* Update driver node to new WebotsController node.

2023.0.4 (2023-05-23)
------------------
* Start ros control and navigation nodes when Webots is ready.

2023.0.3 (2023-04-12)
------------------
* Fixed the calibration of the TIAGo.
* Improved the launch of the nodes when using navigation.
* Added Cartographer for SLAM.

2023.0.2 (2023-02-07)
------------------
* Updated supervisor launch.

2022.1.3 (2022-11-02)
------------------
* Added macOS support.
* Added reset handler to support simulation reset from Webots.

2022.1.2 (2022-10-21)
------------------
* Added WSL support.

1.2.0 (2021-12-21)
------------------
* Adapt the worlds to the new R2022a FLU convention.

1.1.2 (2021-11-03)
------------------
* Initial version (package replaced).
