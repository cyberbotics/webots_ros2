^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2023.1.3 (2024-08-07)
------------------
* Make webots_ros2_driver scripts executable.
* Fixed the produced URDF to also contain joint limits necessary for ros2_control.

2023.1.1 (2023-07-11)
------------------
* Added component remapping parameter to rename PROTO components.
* Added deprecation message when declaring driver node in launch file.
* Added animation_{start,stop}_recording services to Ros2Supervisor node.
* Added /Ros2Supervisor namespace to Ros2Supervisor node.
* Fixed Python plugin termination on SIGINT call or simulation ends.
* Fixed RangeFinder activation to also check for point cloud subscriptions.

2023.1.0 (2023-06-29)
------------------
* Added Ros2Pen static plugin.
* Added parameters to rename Camera and RangeFinder topics.
* Added new WebotsController node to launch robot controller plugins.
* Added compilation of the generic robot window library.
* Default to canonical topic name and fix camera_info stamp in Ros2Camera, Ros2RangeFinder.
* Added VacuumGripper device support.

2023.0.4 (2023-05-23)
------------------
* Fixed vertical field of view in static RangeFinder plugin.
* Added support for painted point clouds.
* Added custom handler to start nodes when Webots is ready.

2023.0.3 (2023-04-12)
------------------
* Adding port, stream type parameters to webots_laucher
* Copying .wbproj when launching a webots world via webots_launcher
* Added Emitter and Receiver support
* Changed undefined Lidar frequency to the default from the .proto file
* Added Compass support

2023.0.2 (2023-02-07)
------------------
* Fixed the spawn of URDF robots in WSL and macOS when using full path.
* Fixed relative assets in macOS.
* Added Ros2Supervisor creation.

2023.0.1 (2023-01-05)
------------------
* Fixed relative assets in WSL.

2023.0.0 (2022-11-30)
------------------
* Add support for the new Python API of Webots R2023a
* Convert C++ controller API functions to C
* Replace libController submodule by commited source files

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
