^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2022.1.4 (2022-11-18)
------------------
* Fix the camera focal length in the CameraInfo topic.
* Update the calculation of CameraRecognitionObject messages to the RDF convention of R2022b.

2022.1.3 (2022-11-02)
------------------
* Added macOS support.

2022.1.2 (2022-10-21)
------------------
* Fix issue where relatively defined PROTO were not found.
* Added WSL support.

2022.1.1 (2022-10-11)
------------------
* Simplified the detection of Webots installation folder.

2022.1.0 (2022-09-23)
------------------
* Added an URDF importer feature to spawn robots from URDF files.

1.2.3 (2022-05-30)
------------------
* Add option to set 'robot_description' parameter for 'robot_state_publisher' node.
* Fix recognition camera.
* Add a 'PointCloud2' publisher for the 'RangeFinder' device.

1.2.2 (2022-01-19)
------------------
* Fix the Supervisor API access from plugins.

1.2.1 (2022-01-10)
------------------
* Fix link error for 'webots_ros2_control' on macOS.
* Fix lidar device according to FLU convention.

1.2.0 (2021-12-21)
------------------
* Adapt the worlds to the new R2022a FLU convention.
* Remove a double webots_ros2_driver header installation.
* Add the publication of the 'gps/speed_vector' topic to the GPS ROS 2 device.

1.1.2 (2021-11-03)
------------------
* Adapted the 'webots_ros2_driver' package to be also a python alternative to the 'webots_ros2_core' package.

1.1.0 (2021-07-19)
------------------
* Initial version
