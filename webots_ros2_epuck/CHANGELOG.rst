^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2_epuck
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2023.1.0 (2023-06-29)
------------------
* Added support for Navigation2 in Iron.
* Clean simulation reset in launch file.
* Update driver node to new WebotsController node.

2023.0.4 (2023-05-23)
------------------
* Fixed ability to launch RViz without other tools.
* Start ros control and navigation nodes when Webots is ready.

2023.0.3 (2023-04-12)
------------------
* Refactored launch files.
* Updated and fixed navigation for Humble compatibility.

2023.0.2 (2023-02-07)
------------------
* Updated supervisor launch.

2023.0.1 (2023-01-05)
------------------
* Fixed broken controller connection in Rats life example.

2022.1.3 (2022-11-02)
------------------
* Added macOS support.
* Added reset handler to support simulation reset from Webots.

2022.1.2 (2022-10-21)
------------------
* Added WSL support.

1.2.3 (2022-05-30)
------------------
* Fixed support for Humble and Rolling.

1.2.0 (2021-12-21)
------------------
* Adapt the worlds to the new R2022a FLU convention.

1.1.2 (2021-11-03)
------------------
* Utilize the 'webots_ros2_driver' and 'ros2_control' instead of 'webots_ros2_core'.

1.0.0 (2020-09-01)
------------------
* Use the webots_ros2_core::WebotsDifferentialDriveNode class

0.0.3 (2020-06-15)
------------------
* Initial version
