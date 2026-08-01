# Team 20 Autonomous Driving System — ROS 2 Jazzy

## Overview

This directory contains the ROS 2 packages for Team 20's course-project autonomous
driving stack. The system runs with the bundled Unity simulator and provides a complete
pipeline from simulated sensors to vehicle commands:

```text
simulation bridge
  ├─ RGB/depth/TF ──> perception ── observations/point cloud ────────┐
  ├─ pose TF ──> route_manager ─┬─ local route ─> trajectory_generator
  │                             │                       │              │
  │                             │                  ┌─ path/speed ──────┤
  │                             │                  └─ timed trajectory ──────────────┐
  │                             └─ route state/ID/distance ───────────┤
  └─ vehicle twist ──────────────────────────────────────────────────┤
                                                              v
                                                       decision_making
                                                   final speed/stop intent

trajectory_generator ── timed trajectory ─────────────────────┐
decision_making ─────── final speed/stop intent ──────────────┼─> controller
simulation bridge ───── vehicle twist/TF ─────────────────────┘       |
                                                                     v
                                                               /car_command
                                                                     |
                                                                     v
                                                           simulation / Unity
```

Production perception uses the left RGB camera, metric depth, camera calibration, and TF.
The Unity semantic/segmentation camera is disabled and is not required to run the project.

The arrows express ownership as well as data flow: route management supplies route
context, trajectory generation owns the feasible geometry/speed/time profile, perception
supplies observations, decision making owns the final permitted speed/stop intent, and
only the controller publishes `/car_command`. The controller uses the speed attached to
its selected trajectory lookahead point and caps it with the decision speed.

## Architecture and design rationale

| Area | Choice | Reason and trade-off |
| --- | --- | --- |
| Route | A deterministic 64-waypoint YAML route with semantic types and per-waypoint speed limits | The assessed map is fixed, so a recorded route is reproducible and easy to inspect. It does not provide global replanning for a changed map. |
| Trajectory | Ego projection, Hermite/Bézier smoothing, bicycle-model curvature validation, and forward/backward time parameterization | Produces a smooth path and a feasible velocity profile under steering, lateral-acceleration, and longitudinal acceleration/braking limits without MPC complexity. It rejects an unreachable initial-speed boundary instead of silently changing measured speed. The envelope does not model tyre friction circles, jerk, steering rate, or drivetrain dynamics and does not avoid obstacles. |
| Perception model | COCO YOLOv8n ONNX on normal RGB, combined with metric depth and TF | Avoids the Unity semantic-camera shortcut while keeping CPU inference practical. Depth adds range, but small simulator traffic lights require route gates, HSV evidence, and temporal filtering. |
| Obstacle model | Planning-path point-cloud corridor fused with YOLO RGB-D vehicle range | Visual class identity rejects static geometry; point-cloud agreement checks spatial plausibility. Route-gated fallbacks cover known sensor gaps without weakening the near-field safety guard. |
| Decision | Explicit state machine with a documented safety priority | Traffic-light, following, merge, and emergency behavior remains traceable and testable. It is less flexible than a learned policy but failures and transitions can be audited. |
| Control | Pure Pursuit steering plus PID speed control on `PlannedTrajectory` | A single lookahead index selects both geometry and the trajectory speed, which is capped by the safety decision. The transparent controller is easy to tune but is not MPC and depends on fresh TF, twist, trajectory, and decision inputs. |
| Safety | Input timeouts, latched stops, bounded commands, and one authoritative publisher | Missing/stale data produces braking instead of stale motion. This can cause conservative temporary stops during sensor gaps, which is preferred to uncontrolled continuation. |

The production bringup owns one vehicle model: it sends the same wheelbase and maximum
steering angle to planning and control. The controller still validates received curvature
independently, so a launch override or configuration mistake cannot make it follow a
trajectory outside its steering boundary.

## What the project can do

- Start the Ubuntu Unity simulator and ROS bridge from one launch command.
- Receive RGB, depth, camera calibration, IMU, vehicle pose, and vehicle velocity data.
- Detect traffic lights and vehicles with a YOLOv8 ONNX RGB-D pipeline.
- Estimate vehicle range by projecting depth-camera points into RGB detections.
- Generate filtered obstacle point clouds, a 3-D OctoMap, and a projected occupancy map.
- Load and follow a predefined 64-waypoint route from waypoint 0 through waypoint 63.
- Publish route phase, current waypoint, waypoint distance, local path, and speed limits.
- Smooth the local route and publish path pose, curvature, dynamically feasible speed,
  and time at every trajectory point using Team 20's custom ROS message.
- Enforce the controller's wheelbase/maximum-steering curvature limit, configured lateral
  acceleration, longitudinal acceleration/deceleration, route speed, and route-completion
  terminal-speed constraints.
- Stop for red, yellow, unknown, or stale traffic-light observations near controlled
  intersections and continue only after a confirmed green signal.
- Follow a slower vehicle with an adaptive speed command.
- Slow down for a vehicle merging from the right and apply emergency braking when needed.
- Track the planned trajectory with Pure Pursuit and regulate the most restrictive of
  trajectory and decision speed with PID control.
- Fail safely when required paths, transforms, sensor data, or decision messages are
  missing or stale.
- Visualize the route, trajectory, perception results, decision state, and telemetry in
  RViz.

The controlled traffic lights are associated with route waypoints 25, 32, 52, and 59.
The right-side merge scenario is handled around waypoints 43–48.

## Package structure

| Package | Responsibility |
| --- | --- |
| `simulation` | Unity TCP sensor receiver, UDP command transmitter, sensor messages, static TF, and simulator launch files. |
| `perception` | YOLO RGB-D traffic-light and vehicle detection, point-cloud filtering, OctoMap, and occupancy maps. |
| `route_manager` | Route loading, waypoint progress, local route context, route state, and speed-limit publication. |
| `autonomy_interfaces` | Team 20 custom `TrajectoryPoint` and `PlannedTrajectory` ROS messages. |
| `trajectory_generator` | Local-route validation/smoothing, curvature computation, time parameterization, and feasible target speed. |
| `decision_making` | Traffic-light state machine, adaptive following, merge handling, obstacle monitoring, and emergency stop. |
| `controller` | Pure Pursuit lateral control, PID speed control, safety checks, and `/car_command` publication. |
| `system_bringup` | Single launch entry point for the complete system and RViz visualizations. |
| `dummy_controller` | Project-supplied example controller; not part of Team 20's production control pipeline. |

Package-specific interface and parameter details are documented in the main Team 20
package READMEs. `dummy_controller` remains course-supplied example code and is not part
of the production architecture.

## System requirements

The recommended environment is:

- Ubuntu 24.04;
- ROS 2 Jazzy Jalisco desktop installation;
- a graphical Linux session with working OpenGL support for the bundled Unity build;
- Git and Git LFS;
- `colcon`, `rosdep`, and standard ROS build tools;
- enough memory and CPU capacity to run Unity, RViz, YOLO inference, and the ROS nodes.

Ubuntu 22.04, WSL, containers, and remote-display setups are not the assessed platform and
may require additional configuration. The included simulator executable is a Linux build.

Install the basic tools if they are not already available:

```bash
sudo apt update
sudo apt install git git-lfs python3-colcon-common-extensions python3-rosdep
git lfs install
```

Install ROS 2 Jazzy by following the official ROS documentation before continuing. If
`rosdep` has never been initialized on the machine, initialize it once:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that its sources already exist, do not run it again; use only
`rosdep update`.

## Get the latest project release

### New clone

The repository is hosted on LRZ GitLab. After adding your SSH key and obtaining course
repository access, clone the release directly:

```bash
git lfs install
git clone --branch release/full-system-v1.9.0 --single-branch \
  git@gitlab.lrz.de:00000000014c3605/i2ros-project-ss26-team20.git \
  introduction-to-ros-2026
cd introduction-to-ros-2026
git lfs pull
```

### Existing clone

From the repository root:

```bash
git fetch origin
git switch release/full-system-v1.9.0
git pull --ff-only origin release/full-system-v1.9.0
git lfs pull
```

Do not copy individual packages into another branch: the release contains matching
interfaces, parameters, route data, model files, simulator configuration, and launch
files that must be used together.

## Install dependencies and prepare Unity

All commands below assume the repository root is the current directory:

```bash
cd project
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
bash src/setup_script.sh
```

The setup script marks the Unity executable and simulator helper scripts as executable.
Runtime YOLO inference uses the ONNX model and ONNX Runtime included in the perception
package; Python Ultralytics is needed only for optional model training/export, not for a
normal project run.

## Build the workspace

Run from the `project/` directory:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Source both ROS and the workspace in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source <repository-path>/project/install/setup.bash
```

Replace `<repository-path>` with the absolute path to the cloned repository.

## Run the automated checks

Before driving the simulated vehicle, run the same static and functional checks used for
integration:

```bash
cd project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash tools/check_code_quality.sh
colcon test
colcon test-result --verbose
```

The final command must report zero failures and zero errors. The code-quality command
checks Team 20 Python and C++ code, the simulator launch/test Python, and the simulator
bridge C++. The project-supplied `dummy_controller` and vendored third-party sources are
excluded; simulator C++ is checked with documented copyright/include-path exemptions.

Acceptance requires all nine packages to build, the quality gate to report no findings,
and the complete test suite to report zero errors and zero failures. The latest retained
result is shown separately below as an example.

## Run the complete system

### 1. Safe startup with motion disabled

Start the complete pipeline first without allowing the controller to move the vehicle:

```bash
cd project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=false \
  enable_visualization:=true
```

This starts Unity, the TCP/UDP bridge, sensor TF publishers, perception, occupancy mapping,
route management, trajectory generation, decision making, the real controller, and two
RViz views. The controller publishes a safe brake command while motion is disabled.

Wait until the terminal shows that:

- Unity has connected to the ROS sensor receiver;
- 64 route waypoints have been loaded;
- the YOLO ONNX model has loaded successfully;
- RGB and depth messages are being received;
- TF `world -> OurCar/INS` is available;
- the planning, decision, and controller nodes are active.

Stop the safe startup with `Ctrl+C` before starting the moving run.

### 2. Full drive from waypoint 0

Use the standard mission runner. It always enables the production controller,
records the audit topics and logs, waits for route completion, and generates
`mission_results/` from that run:

```bash
python3 tools/run_full_mission.py --visualization
```

For better performance on a machine where RViz is not needed, keep Unity running but
disable the RViz windows:

```bash
python3 tools/run_full_mission.py
```

A successful complete run progresses from waypoint 0 to waypoint 63 and logs:

```text
Route completed at waypoint 63
```

At the endpoint, the decision and controller layers command a safe stop. The runner then
stops recording and the launch, validates the completed bag, and atomically replaces
`mission_results/`. A failed, interrupted, or audit-failing run keeps the previous valid
result. Raw bags and logs are retained under the Git-ignored `validation_evidence/`
directory. Direct `ros2 launch` remains useful for diagnostics, but does not publish a
mission report automatically.

### Latest complete-test example

This subsection records one retained validation run. It demonstrates that the acceptance
procedure has been completed, but its timing, message counts, obstacle distances, and
state counts are measurements from that run rather than fixed project guarantees.

The trajectory-planning validation on 2026-08-01 completed waypoint 0 through 63. Mission
time was **211.065 s**, measured from the first forward `/car_command` (`throttle > 0.01`,
`brake < 0.01`) to the first `route_complete`; the first-measured-motion cross-check was
210.925 s. The audit checked 68,355 points in 2,123 `/planning/trajectory` messages and
found no invalid values or non-monotonic timestamps. Maximum curvature was 0.215946 1/m
against the steering-derived 0.217704 1/m limit. Observed maxima were 3.0000 m/s² lateral
acceleration, 2.0003 m/s² longitudinal acceleration, and 3.0000 m/s² deceleration, within
the configured limits plus the documented numerical tolerances. Plots and
machine-readable data are in
[`../mission_results/`](../mission_results/README.md).

The route-stage audit additionally found 2,098 nonempty messages out of 2,112 before
completion (0.66% empty; longest continuous empty interval 0.902 s; maximum message gap
0.162 s), within the documented 5% / 10 s / 1.05 s availability gates. Of 105 runtime
path rejections, 104 were classified as `path exceeds the curvature limit` and one as
`path has fewer than two points`; each straight target-pose fallback was subjected to
the same feasibility checks before publication. The bag also recorded 1,459 UInt8
traffic-light messages (1,170 unknown, 196 red, 7 yellow, and 86 green), so the generated
traffic-light plot contains an actual state trace.

In Event1, adaptive following stayed continuous: measured speed remained at or above
2.536 m/s and the decision target at or above 2.949 m/s, with no near-zero samples. Event2
reached a minimum measured obstacle distance of 7.535 m without collision, emergency
braking, or merge slowdown.

## Validate a live run

In a second terminal, source ROS and the workspace before running these commands:

```bash
ros2 node list
ros2 topic hz /OurCar/INS/twist
ros2 topic hz /planning/path
ros2 topic hz /planning/trajectory
ros2 interface show autonomy_interfaces/msg/PlannedTrajectory
ros2 topic echo /route/current_waypoint_id
ros2 topic echo /decision/state
ros2 topic echo /perception/traffic_light_state
ros2 topic info /car_command
```

Expected behavior includes:

- exactly one publisher on `/car_command`, named `controller_node`;
- continuous pose, velocity, RGB, depth, route, and planning updates;
- finite `/planning/trajectory` points with increasing `time_from_start`;
- `FOLLOWING_VEHICLE` when a lead vehicle constrains speed;
- `STOPPING_FOR_LIGHT` or `WAITING_FOR_GREEN` at a red light;
- `RELEASED` followed by `CRUISE` after a confirmed green light;
- `SLOWING_FOR_MERGE` during the right-side merge event;
- a final safe stop after `Route completed at waypoint 63`.

Initial warnings about missing TF, path, or velocity can occur while Unity is opening and
the first sensor messages are arriving. Motion must remain stopped during this period.
Warnings that continue after Unity has connected should be investigated.

## Important launch arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `start_unity` | `true` | Start the bundled Unity simulator. Set to `false` when Unity is already running elsewhere. |
| `sensor_bind_host` | `127.0.0.1` | Address used by the TCP sensor receiver. Use `0.0.0.0` only when intentionally accepting a remote Unity connection. |
| `enable_motion` | `false` | Permit the real controller to move the vehicle. Must be explicitly enabled for a full drive. |
| `enable_visualization` | `true` | Start the live visualization node and RViz views. |
| `route_initial_waypoint_index` | `0` | Select the initial route waypoint. Use `0` for a complete run. |
| `enable_perception` | `true` | Start YOLO RGB-D perception and mapping. |
| `enable_octomap` | `true` | Start OctoMap within the perception pipeline. |
| `enable_route` | `true` | Start route management. |
| `enable_trajectory` | `true` | Start trajectory generation. |
| `enable_decision` | `true` | Start decision making and obstacle monitoring. |
| `enable_controller` | `true` | Start the production controller. |

The launch also accepts custom `simulation_config`, `yolo_model`, and `route_file` paths.
The defaults point to the matching files installed with this release.

## Run individual subsystems

The full-system launch is the recommended integration entry point. For focused debugging,
the main subsystems can also be launched separately after sourcing the workspace:

```bash
ros2 launch simulation simulation.launch.py
ros2 launch perception perception.launch.py
ros2 launch trajectory_generator trajectory_generator.launch.py
ros2 launch decision_making decision_making.launch.py
ros2 launch controller controller.launch.py
```

See the package READMEs for required inputs and standalone route-manager parameters. Do not
start the real controller and `dummy_controller` at the same time.

## Troubleshooting

### Unity executable is missing or does not start

Make sure Git LFS assets were downloaded and executable permissions were prepared:

```bash
git lfs pull
cd project
bash src/setup_script.sh
```

The bundled Unity build needs a graphical Linux session. On a remote machine, either
configure display forwarding or run Unity on another supported machine and launch with
`start_unity:=false` and the appropriate `sensor_bind_host`.

### Packages or launch files are not found

Source ROS before building and source the workspace after building:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Repeat the two `source` commands in every new terminal.

### The vehicle does not move

Check that `enable_motion:=true` was passed, Unity is connected, and controller inputs are
fresh. The controller intentionally brakes if TF, vehicle velocity, planning path, target
speed, or decision state is unavailable.

### Conflicting vehicle commands

Check the command topic:

```bash
ros2 topic info /car_command
```

There must be exactly one publisher. Stop `dummy_controller` or any manual command
publisher before starting the production controller.

### YOLO model or simulator data is missing

Run `git lfs pull`, rebuild, and source the workspace again. The default model must be
installed as `share/perception/models/yolov8n.onnx`.

## Further documentation

- [`../PROJECT_STATUS_SUMMARY.md`](../PROJECT_STATUS_SUMMARY.md): English architecture,
  design choices, ownership, validation evidence, figures, and bibliography.
- [`../项目现状.md`](../项目现状.md): Chinese version of the project-status summary.
- [`../INTEGRATION_README.md`](../INTEGRATION_README.md): staged readiness and scenario
  validation gates.
- [`perception/README.md`](perception/README.md): RGB-D projection, traffic-light
  detection, model replacement, and optional training.
- [`route_manager/README.md`](route_manager/README.md): route format, states, parameters,
  and topics.
- [`trajectory_generator/README.md`](trajectory_generator/README.md): smoothing, dynamic
  constraints, custom trajectory interface, and path validation.
- [`autonomy_interfaces/README.md`](autonomy_interfaces/README.md): Team 20 custom message
  definitions and ownership boundary.
- [`decision_making/README.md`](decision_making/README.md): state machine, obstacle fusion,
  safety, and tuning.
- [`controller/README.md`](controller/README.md): Pure Pursuit, PID, timeouts, and control
  parameters.
- [`simulation/README.md`](simulation/README.md): Unity bridge, sensor topics, networking,
  and simulator configuration.
- [`system_bringup/README.md`](system_bringup/README.md): full-system orchestration,
  safety switches, launch arguments, and visualization.
