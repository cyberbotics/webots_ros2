^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.3 (2022-02-22)
------------------
* This package shows now deprecation warnings and will be removed with the release of Webots `R2023a`.
* Users of `webots_ros2_core` should migrate to `webots_ros2_driver`.

1.1.2 (2021-11-03)
------------------
* This package is now deprecated.

1.0.4 (2021-01-08)
------------------
* Improved performance of camera.
* Replaced `tkinter` by a simple command line tools.
* Fixed usage on Windows.
* Introduced notion of minimum and target Webots versions.

1.0.3 (2020-12-01)
------------------
* Improved the performance of point cloud publishing by a few times.

1.0.2 (2020-10-12)
------------------
* Fixed support for 3D Lidars
* Fixed Webots executable discovery

1.0.0 (2020-09-01)
------------------
* Added a universal 'webots_differential_drive_node' node.

0.0.4 (2020-07-03)
------------------
* Fixed dependencies issue.

0.0.3 (2020-06-15)
------------------
* Added support for multi robots.
* Added a new TfPublisher class to publish transforms of all the Solid nodes of the robot (if the robot `supervisor` field is true).
* Added the possibility to run nodes in synchronized mode (using the 'synchronization' parameter).
* Added better support for differential drive robots ('WebotsDifferentialDriveNode' class).
* Added CameraDevice, LEDDevice and LaserDevice that create correspoding ROS2 topics

0.0.2 (2019-09-23)
------------------
* Initial version
