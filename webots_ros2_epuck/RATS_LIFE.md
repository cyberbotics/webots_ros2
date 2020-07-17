# Rat's Life

Here we want to benchmark difference between e-puck physical and simulated robot which use ROS2 interface.
The benchmark is highly inspired by a paper [_The Rat’s Life Benchmark: Competing Cognitive Robots_](http://www.vernon.eu/euCognition/NA045-3/PerMIS08_SS1-OMichel.pdf).

![Webots Map](./assets/map_webots.png)


## Mapping

To run the simulation execute:
```bash
ros2 launch webots_ros2_epuck rats_life_waypoints_launch.py use_sim_time:=true
```

## Navigation

To run the simulation execute:
```bash
ros2 launch webots_ros2_epuck rats_life_launch.py use_sim_time:=true
```