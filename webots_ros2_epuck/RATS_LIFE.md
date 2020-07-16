# Rat's Life

Here we want to benchmark difference between e-puck physical and simulated robot which use ROS2 interface.
The benchmark is highly inspired by a paper [_The Rat’s Life Benchmark: Competing Cognitive Robots_](http://www.vernon.eu/euCognition/NA045-3/PerMIS08_SS1-OMichel.pdf).

To run the simulation execute:
```bash
ros2 launch webots_ros2_epuck rats_life_launch.py
```

And to set the initial pose:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
    "header": { "frame_id": "map" },
    "pose": {
        "pose": {
            "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
            "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
        }
    }
}'
```
