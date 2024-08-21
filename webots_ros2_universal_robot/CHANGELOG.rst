^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2_universal_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2023.1.3 (2024-08-07)
------------------
* Fixed passing the `robot_description` parameter to ros2_control.

2023.1.2 (2023-07-28)
------------------
* Fixed errors showing up when launching moveit for ur5e.

2023.1.0 (2023-06-29)
------------------
* Clean simulation reset in launch file.
* Fixed JTC interpolation.
* Fixed deprecated load_yaml() function.
* Update driver node to new WebotsController node.

2023.0.4 (2023-05-23)
------------------
* Start ros control nodes when Webots is ready.

2023.0.2 (2023-02-07)
------------------
* Fixed URDF relative URLs to assets.
* Updated supervisor launch.

2022.1.3 (2022-11-02)
------------------
* Added macOS support.

2022.1.2 (2022-10-21)
------------------
* Added WSL support.

2022.1.0 (2022-09-23)
------------------
* The 'robot' and 'moveit_demo' scenarios now show a use case of the new URDF importer.

1.2.2 (2022-01-19)
------------------
* Remove the 'moveit' dependency.

1.2.0 (2021-12-21)
------------------
* Adapt the worlds to the new R2022a FLU convention.
* Fix synchornosiation issue when the trajectory controller was receiving goals but was not ready to execute them.

1.1.2 (2021-11-03)
------------------
* Utilize the 'webots_ros2_driver' and 'ros2_control' instead of 'webots_ros2_core'.
* Add MoveIt2 example.
* Upgrade the multi-robot example.
* Remove non-useful simulations.

1.0.0 (2020-09-01)
------------------
* Use the webots_ros2_core::webots_robotic_arm_node node.

0.0.3 (2020-06-15)
------------------
* Added an 'universal_robot_multiple' simulation and launch file.
* Added an 'universal_robot_rviz' simulation and launch file.
* Added support for the goal tolerance in the action server.
* Fixed the action server that was never reaching the `SUCCESSFUL` state.

0.0.2 (2019-09-23)
------------------
* Initial version
