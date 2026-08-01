# Introduction to ROS — Team 20 Autonomous Driving Project

This repository contains Team 20's ROS 2 Jazzy autonomous-driving system for the
Introduction to ROS course. The graded ROS workspace is under [`project/`](project/) and
targets Ubuntu 24.04 with the bundled Unity simulator.

The system follows a predefined route, detects traffic lights and vehicles with RGB-D
perception, generates a smooth local trajectory, makes traffic and obstacle decisions,
and controls the simulated vehicle through the complete route. Production perception
uses RGB and depth images with YOLO; it does not use the Unity semantic camera.

## Team contributions

| Team member | Main contribution |
| --- | --- |
| Lei Fang | `route_manager` |
| Yida Lyu | `decision`, `controller`, part of integration |
| Tao Zhang | Perception, part of integration |
| Jianing Huang | `Trajectory_Generator`, part of integration |

In the repository, `decision` is implemented by the `decision_making` package and
`Trajectory_Generator` is stored as the `trajectory_generator` package.

The Unity simulation bridge and `dummy_controller` were supplied with the project and are
not listed as Team 20 contributions.

## Documentation

- [`project/src/README.md`](project/src/README.md): supported functions, dependencies,
  build and test commands, full-system startup, validation, and troubleshooting.
- [`project/INTEGRATION_README.md`](project/INTEGRATION_README.md): staged integration and
  scenario-validation procedure.
- The main Team 20 functional packages contain package-specific READMEs with their
  interfaces, parameters, and design details.

## Quick start

After checking out `release/full-system-v1.8.0` and installing ROS 2 Jazzy:

```bash
cd project
bash src/setup_script.sh
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch system_bringup full_system.launch.py enable_motion:=false
```

The command above starts the complete stack in safe mode. After confirming that Unity,
TF, sensor topics, perception, planning, and decision making are ready, start a full drive
from waypoint 0 with:

```bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=true \
  enable_visualization:=true
```

Never run `dummy_controller` and the real `controller` together: `/car_command` must have
exactly one publisher.
