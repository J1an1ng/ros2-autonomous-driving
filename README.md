# Introduction to ROS — Team 20 Autonomous Driving Project

This repository contains Team 20's ROS 2 Jazzy autonomous-driving system for the
Introduction to ROS course. The graded ROS workspace is under [`project/`](project/) and
targets Ubuntu 24.04 with the bundled Unity simulator.

The system follows a predefined route, detects traffic lights and vehicles with RGB-D
perception, generates a smooth local trajectory, makes traffic and obstacle decisions,
and controls the simulated vehicle through the complete route. Production perception
uses RGB and depth images with YOLO; it does not use the Unity semantic camera.

## Team contributions

| Team member | Main contribution | Traceable project artifacts |
| --- | --- | --- |
| Lei Fang | `route_manager` | Route YAML model, waypoint recording/progress, route context, speed limits, tests, and package documentation |
| Yida Lyu | `decision`, `controller`, part of integration | Decision state machine, obstacle behavior, Pure Pursuit/PID control, safety handling, and the interfaces from final intent to `/car_command` |
| Tao Zhang | Perception, part of integration | YOLO RGB-D detection, traffic-light classification, vehicle ranging, occupancy mapping, perception configuration, and perception-to-decision interfaces |
| Jianing Huang | `Trajectory_Generator`, part of integration | Route projection, path smoothing/validation, speed-profile output, and route-to-planning interfaces |

In the repository, `decision` is implemented by the `decision_making` package and
`Trajectory_Generator` is stored as the `trajectory_generator` package.

The Unity simulation bridge and `dummy_controller` were supplied with the project and are
not listed as Team 20 contributions.

“Part of integration” is shared work by Yida Lyu, Tao Zhang, and Jianing Huang. It covers
bringup/configuration alignment, ROS interface integration around their modules,
end-to-end testing, dynamic validation, and documentation. The repository does not claim
a more granular individual split where the team did not provide one.

## Code ownership and reuse

- Team 20 wrote or substantively developed `perception`, `route_manager`,
  `trajectory_generator`, `decision_making`, `controller`, and `system_bringup`, together
  with their project-specific configuration, tests, integration tools, and documentation.
- The Unity simulator/ROS bridge and `dummy_controller` were supplied with the course
  project. Formatting or ROS 2 integration work does not transfer ownership of that
  supplied code to Team 20.
- YOLOv8n/Ultralytics, COCO-pretrained weights, ONNX Runtime, OpenCV, ROS 2, OctoMap, and
  other dependency code are reused third-party components. Team 20 owns the surrounding
  ROS integration and project behavior, not those algorithms or libraries.
- Pure Pursuit, PID, Hermite/Bézier interpolation, and state-machine concepts are known
  methods. Team 20 implemented and tuned the project code that applies them, but does not
  claim invention of the underlying algorithms.

The full component-by-component ownership and design rationale are documented in
[`project/PROJECT_STATUS_SUMMARY.md`](project/PROJECT_STATUS_SUMMARY.md).

## Documentation

- [`project/PROJECT_STATUS_SUMMARY.md`](project/PROJECT_STATUS_SUMMARY.md): English
  written summary of the full architecture, model and design choices, validation status,
  team contributions, and own/reused code boundaries.
- [`project/项目现状.md`](project/项目现状.md): Chinese version of the same project-status
  summary.
- [`project/src/README.md`](project/src/README.md): supported functions, dependencies,
  build and test commands, full-system startup, validation, and troubleshooting.
- [`project/INTEGRATION_README.md`](project/INTEGRATION_README.md): staged integration and
  scenario-validation procedure.
- The main Team 20 functional packages contain package-specific READMEs with their
  interfaces, parameters, and design details.

## Quick start

After checking out `release/full-system-v1.9.0` and installing ROS 2 Jazzy:

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
