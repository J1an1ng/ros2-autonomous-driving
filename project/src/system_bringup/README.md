# System Bringup

## Purpose and ownership

`system_bringup` is Team 20's integration package. It provides one production launch entry
point for simulation, perception, route management, trajectory generation, decision
making, control, and visualization. Integration is shared by Yida Lyu, Tao Zhang, and
Jianing Huang around their respective modules.

The package contains orchestration and visualization code; it does not duplicate the
domain logic owned by the individual packages.

## Architecture

`full_system.launch.py` starts the packages in one ROS graph:

```text
simulation
  ├─ sensors ──> perception ─────────────────────────> decision_making
  ├─ pose TF ──> route_manager ──> trajectory_generator
  │                                      ├─ /planning/target_speed ──> decision_making
  │                                      └─ /planning/path ───────────────────┐
  └─ vehicle twist/TF ────────────────────────────────────────────────────────┤
decision_making ── /decision/target_speed + stop intent ─────────────────────┤
                                                                               v
                                                                          controller
                                                                               │
                                                                          /car_command
                                                                               │
                                                                               v
                                                                           simulation
```

The controller is the only production publisher of `/car_command`. Do not start the
project-supplied `dummy_controller` at the same time.

## Design choices

- `enable_motion` defaults to `false`, so a normal launch starts the complete graph but
  keeps the vehicle braked until motion is explicitly authorized.
- Each subsystem has an enable flag for staged integration and fault isolation.
- Default model, route, controller configuration, Unity configuration, and RViz files are
  resolved from installed package shares rather than source-tree paths.
- Unity startup and RViz visualization are independent: `enable_visualization=false`
  reduces rendering load without disabling Unity or autonomy.
- The full benchmark always uses `route_initial_waypoint_index:=0`; a different index is a
  diagnostic option that requires a matching simulator spawn.

## Launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `start_unity` | `true` | Start the bundled Unity simulator. |
| `sensor_bind_host` | `127.0.0.1` | TCP sensor receiver bind address. |
| `enable_perception` | `true` | Start YOLO RGB-D detection and mapping. |
| `enable_octomap` | `true` | Start OctoMap through the perception launch. |
| `enable_route` | `true` | Start `route_manager_node`. |
| `enable_trajectory` | `true` | Start trajectory generation. |
| `enable_decision` | `true` | Start obstacle monitoring and the decision state machine. |
| `enable_controller` | `true` | Start the production controller. |
| `enable_visualization` | `true` | Start live visualization and two RViz views. |
| `enable_motion` | `false` | Permit controller motion; false commands a safe stop. |
| `route_initial_waypoint_index` | `0` | Initial route index for diagnostics/full route. |
| `simulation_config` | installed default | Unity sensor/event configuration. |
| `yolo_model` | installed `yolov8n.onnx` | Perception model path. |
| `route_file` | installed `route_map.yaml` | Production route path. |

## Run

Safe startup:

```bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=false \
  enable_visualization:=true
```

Full route after readiness checks:

```bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=true \
  enable_visualization:=false
```

See `project/INTEGRATION_README.md` for the staged gate and required topic checks.

## Visualization

When enabled, the package starts:

- `live_visualization`, which publishes driven trace and telemetry-friendly data;
- `driving_overview` RViz using `rviz/driving_overview.rviz`;
- `telemetry_charts` RViz using `rviz/telemetry_charts.rviz`.

Visualization is diagnostic only and is not an input to perception, planning, decision,
or control.

## Validation

In full-system-v1.9.0 the launch brought up all production modules, completed waypoint
0–63, exercised traffic-light/following/merge behavior, and shut down Unity and all ROS
nodes cleanly. Automated validation passed 179 tests with no errors or failures and one
expected skip.
