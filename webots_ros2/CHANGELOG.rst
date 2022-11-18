^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2022.1.4 (2022-11-18)
------------------
* Fix the camera focal length in the CameraInfo topic.
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
* Add PointCloud2 support for RangeFinder.

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
