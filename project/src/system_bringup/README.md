# System Bringup

## Purpose and ownership

`system_bringup` is Team 20's integration package. It provides one production launch entry
point for simulation, perception, route management, trajectory generation, decision
making, control, and visualization. Integration is shared by Lei Fang, Yida Lyu, Tao
Zhang, and Jianing Huang around their respective modules.

The package contains orchestration and visualization code; it does not duplicate the
domain logic owned by the individual packages.

## Architecture

`full_system.launch.py` starts the packages in one ROS graph:

```text
simulation
  ├─ sensors ──> perception ─────────────────────────> decision_making
  ├─ pose TF ──> route_manager ──> trajectory_generator
  │                                      ├─ /planning/target_speed ──> decision_making
  │                                      └─ /planning/trajectory ─────────────┐
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

The obstacle monitor still consumes `/planning/path` as a geometric corridor, while the
controller consumes the richer Team 20 `/planning/trajectory` message. The latter ties
the Pure Pursuit point to its curvature-constrained, time-parameterized speed.

## Design choices

- `enable_motion` defaults to `false`, so a normal launch starts the complete graph but
  keeps the vehicle braked until motion is explicitly authorized.
- Each subsystem has an enable flag for staged integration and fault isolation.
- Default model, route, controller configuration, Unity configuration, and RViz files are
  resolved from installed package shares rather than source-tree paths.
- One shared `wheelbase` / `maximum_steering_angle` pair is injected into trajectory
  planning and control, keeping the bicycle-model feasibility boundary consistent.
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
| `wheelbase` | `2.63` | Shared planner/controller bicycle-model wheelbase in metres. |
| `maximum_steering_angle` | `0.52` | Shared planner/controller steering-angle limit in radians. |

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
python3 tools/run_full_mission.py
```

Add `--visualization` when RViz is needed. The runner records one complete mission and,
only after route and trajectory validation pass, atomically replaces
`mission_results/`. Direct launch is intended for motion-disabled diagnostics.

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
0--63, and exercised traffic-light and adaptive-following behavior in both vehicle
events. Mission time was 210.546 s from the first forward command to `route_complete`.
No runtime process death occurred; after completion, 24 process-death log entries were
classified as teardown-only. Automated validation passed 214 tests with no errors or
failures and one expected skip.
