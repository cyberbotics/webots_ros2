^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2023.1.3 (2024-08-07)
------------------
* Added support for ROS 2 Jazzy.
* Make webots_ros2_driver scripts executable.
* Fixed passing the `robot_description` parameter to ros2_control.
* Fixed the produced URDF to also contain joint limits necessary for ros2_control.
* Added support for the new `ros2_control` API affecting resource_manager and controller_manager.

2023.1.2 (2024-04-08)
------------------
* Fixed errors showing up when launching moveit for ur5e.
* Fixed nav2 turtlebot test failing very often.
* Fixed build and tests for rolling.
* Fixed deprecated ros_controls command: cmd_vel_unstamped.
* Remove usage of deprecated resource manager method: activate_all_components().
* Set is_urdf_loaded__ of the resource manager to true.

2023.1.1 (2023-07-11)
------------------
* Added deprecation message when declaring driver node in launch file.
* Fixed RangeFinder activation to also check for point cloud subscriptions.
* Added component remapping parameter to WebotsController to rename PROTO components.
* Added animation_{start,stop}_recording services to Ros2Supervisor node.
* Added /Ros2Supervisor namespace to Ros2Supervisor node.
* Fixed Python plugin termination on SIGINT call or simulation ends.

2023.1.0 (2023-06-29)
------------------
* Added Ros2Pen static plugin.
* Added support for Navigation2 in Iron.
* Clean simulation reset in launch files.
* Fixed Universal Robot trajectory interpolation.
* Added new TIAGo project to webots_ros2_tiago to run real robot configuration.
* Added new WebotsController node in the driver interface to launch robot controller plugins.
* Fixed unfound robot window library in Tesla example.
* Default to canonical topic name and fix camera_info stamp in Ros2Camera, Ros2RangeFinder.
* Added VacuumGripper gripper support in webots_ros2_driver.
* Added BoolStamped message in webots_ros2_msgs.
* Added GetBool service in webots_ros2_msgs.
* Fixed webots_ros2_control component activation.

2023.0.4 (2023-05-23)
------------------
* Drop support for Foxy.
* Fixed vertical field of view in static RangeFinder plugin.
* Added support for painted point clouds.
* Fixed ability to launch RViz without other tools in e-puck example.
* Fixed command line arguments in importer tools.
* Added custom handler in driver interface to start nodes when Webots is ready.

2023.0.3 (2023-04-12)
------------------
* Fixed the calibration of the TIAGo.
* Improved the navigation of the TIAGo example.
* Added Cartographer for SLAM in the TIAGo example.
* Adding port, stream type parameters to webots_laucher
* Copying .wbproj when launching a Webots world via webots_launcher
* Added Emitter and Receiver support in webots_ros2_driver
* Changed undefined Lidar frequency to the default from the .proto file
* Added Compass support in webots_ros2_driver
* Added startup of the Turlebot navigation and mapping tools from the launch file.
* Fixed the calibration of the e-puck.
* Fixed and improved the navigation of the e-puck example.

2023.0.2 (2023-02-07)
------------------
* Drop support for Galactic.
* Fixed the spawn of URDF robots in WSL and macOS when using full path.
* Fixed relative assets in macOS.
* Ros2Supervisor is now optional.

2023.0.1 (2023-01-05)
------------------
* Fixed relative assets in WSL.
* Fixed broken controller connection in Rats life example.

2023.0.0 (2022-11-30)
------------------
* Added support for the new Python API of Webots R2023a
* Convert C++ controller API functions to C
* Replace libController submodule by commited source files
* Removed 'webots_ros2_core' package (deprecated).
* Allow custom motor-encoder pair.

2022.1.4 (2022-11-18)
------------------
* Fixed the camera focal length in the CameraInfo topic.
* Upgraded to urdf2webots 2.0.3
* Update the calculation of CameraRecognitionObject messages to the RDF convention of R2022b.

2022.1.3 (2022-11-02)
------------------
* Added macOS support.
* Added reset handler to all examples to support simulation reset from Webots.

2022.1.2 (2022-10-21)
------------------
* Added WSL support.

2022.1.0 (2022-09-23)
------------------
* Adapted controllers to communicate with Webots R2022b.
* Added feature to import URDF on the fly.
* Added PointCloud2 support for RangeFinder.

1.2.3 (2022-06-01)
------------------
* Fixed support for Humble and Rolling.

1.1.2 (2021-11-03)
------------------
* Adapted the 'webots_ros2_driver' package to be also a python alternative to the 'webots_ros2_core' package.
* Replaced the use of the deprecated 'webots_ros2_core' package by the 'webots_ros2_driver' package.
* Removed the 'webots_ros2_example', 'webots_ros2_tutorials' and 'webots_ros2_abb' packages.
* Replaced the 'webots_ros2_tiago' package.

1.1.0 (2021-07-19)
------------------
* Included the 'webots_ros2_driver' package as a C++ alternative to the 'webots_ros2_core' package.
* Integrated ros2_control.
* Included a Mavic drone simulation example

1.0.5 (2021-01-08)
------------------
* Improved performance of the camera.
* Replaced tkinter with simple command-line tools.
* Fixed usage on Windows.
* Introduced notion of minimum and target Webots versions.

1.0.1 (2020-09-18)
------------------
* Removed the 'webots_ros2_desktop' package.
* Added missing 'webots_ros2_demos', 'webots_ros2_epuck' and 'webots_ros2_msgs' packages as dependencies.

1.0.0 (2020-09-01)
------------------
* Improved support for macOS

0.0.3 (2020-06-15)
------------------
* Updated to Webots R2020a

0.0.2 (2019-09-23)
------------------
* Moved sources to cyberbotics/webots_ros2 (https://github.com/cyberbotics/webots_ros2)
* Added a 'webots_ros2_msgs', 'webots_ros2_core', 'webots_ros2_desktop', 'webots_ros2_examples' 'webots_ros2_abb' and 'webots_ros2_universal_robot' packages
* Added support for ABB robots.

0.0.1 (2019-08-09)
------------------
* Initial version
