# Introduction to ROS — Team 20 Autonomous Driving Project

This repository contains Team 20's ROS 2 Jazzy autonomous-driving system for the
Introduction to ROS course. The graded ROS workspace is under [`project/`](project/) and
targets Ubuntu 24.04 with the bundled Unity simulator.

The system follows a predefined route, detects traffic lights and vehicles with RGB-D
perception, generates a smooth local trajectory, makes traffic and obstacle decisions,
and controls the simulated vehicle through the complete route. Production perception
uses RGB and depth images with YOLO; it does not use the Unity semantic camera.

## Team contributions

| Team member | Student ID | Main contribution | Traceable project artifacts |
| --- | --- | --- | --- |
| Lei Fang | 03805710 | `route_manager` | Route YAML model, waypoint recording/progress, route context, speed limits, tests, and package documentation |
| Yida Lyu | 03816236 | `decision`, `controller`, part of integration | Decision state machine, obstacle behavior, Pure Pursuit/PID control, safety handling, and the interfaces from final intent to `/car_command` |
| Tao Zhang | 03822999 | `Perception`, part of integration | YOLO RGB-D detection, traffic-light classification, vehicle ranging, occupancy mapping, perception configuration, and perception-to-decision interfaces |
| Jianing Huang | 03814771 | `Trajectory_Generator`, part of integration, project-wide documentation | Route projection, path smoothing/validation, curvature and acceleration-constrained time parameterization, custom trajectory messages, route-to-planning interfaces, and repository-wide project documentation |

In the repository, `decision` is implemented by the `decision_making` package and
`Trajectory_Generator` is stored as the `trajectory_generator` package.

The Unity simulation bridge and `dummy_controller` were supplied with the project and are
not listed as Team 20 contributions.

“Part of integration” is shared work by Yida Lyu, Tao Zhang, and Jianing Huang. It covers
bringup/configuration alignment, ROS interface integration around their modules,
end-to-end testing, and dynamic validation. Jianing Huang wrote the repository-wide
project documentation. The repository does not claim a more granular individual split
for the shared integration work where the team did not provide one.

## Code ownership and reuse

- Team 20 wrote or substantively developed `perception`, `route_manager`,
  `autonomy_interfaces`, `trajectory_generator`, `decision_making`, `controller`, and
  `system_bringup`, together with their project-specific configuration, tests,
  integration tools, and documentation.
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
- [`project/docs/ros_graph.svg`](project/docs/ros_graph.svg): ROS package/topic graph with
  contribution ownership.
- [`project/mission_results/README.md`](project/mission_results/README.md): recorded
  waypoint 0--63 mission time, trajectory-constraint audit, and generated plots.
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
python3 tools/run_full_mission.py --visualization
```

This is the standard moving-run entry point. It launches the production stack,
records the required topics and logs, waits for `route_complete`, audits the
trajectory, and atomically replaces `mission_results/` with figures and metrics
from that run. A failed or interrupted run preserves the previous valid result.

Never run `dummy_controller` and the real `controller` together: `/car_command` must have
exactly one publisher.

## Validation requirements

A release is accepted only after the code-quality gate and complete ROS 2 test suite
pass, all nine packages build, and the production stack completes waypoint 0--63 in
Unity. The full run must record mission time, scenario-state evidence, trajectory
availability and dynamic constraints, traffic-light observations, and runtime logs.

The acceptance audit also requires one production `/car_command` publisher, no collision
or route departure, no autonomy-node failure before route completion, and operation with
the semantic camera disabled. These are project requirements; their measured values may
vary slightly between simulator runs.

## Latest complete-test example

The following numbers are evidence from the most recent retained run, not fixed behavior
or guaranteed constants of the system.

The validation machine ran Ubuntu 24.04.3 LTS under WSL2 on a 13th Gen Intel Core
i7-13700H, with 8 logical processors and approximately 12 GiB of memory available to
WSL2. Unity used the hardware-accelerated D3D12 renderer on an NVIDIA GeForce RTX 4060
Laptop GPU.

The recorded 2026-08-01 production run completed waypoint 0--63 in **211.065 s** from the
first forward control command to `route_complete`. Its 68,355 serialized trajectory
points passed the steering-derived curvature and lateral/longitudinal acceleration
constraints. Event1 remained in continuous following motion at or above 2.536 m/s rather
than stopping and restarting. The route-stage availability audit also required nonempty
trajectory evidence and bounded empty-publication fraction/duration; see the results linked
above for the exact timing definition, traffic-light record, fallback classification,
and teardown evidence.
