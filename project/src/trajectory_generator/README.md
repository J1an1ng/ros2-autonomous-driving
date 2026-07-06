# Trajectory Generator

Minimal ROS 2 trajectory generation package for the autonomous driving project.

## Purpose

`trajectory_generator_node` consumes the current short-term route target from
`route_manager`, looks up the ego vehicle pose from TF, and publishes a simple
interpolated local path plus a target speed for the controller.

This is Version 1. It intentionally does not implement MPC, collision avoidance,
or complex trajectory optimization.

## Inputs

| Topic | Type | Description |
| --- | --- | --- |
| `/route/target_pose` | `geometry_msgs/msg/PoseStamped` | Current short-term route target. |
| `/route/speed_limit` | `std_msgs/msg/Float32` | Speed limit associated with the route target. |
| `/route/route_state` | `std_msgs/msg/String` | Semantic route state such as `normal`, `stop_zone`, or `intersection`. |
| `/route/current_waypoint_id` | `std_msgs/msg/Int32` | Current target waypoint id, used for diagnostics. |
| TF | `map_frame -> vehicle_frame` | Ego pose lookup. Defaults to `world -> OurCar/INS`. |

## Outputs

| Topic | Type | Description |
| --- | --- | --- |
| `/planning/path` | `nav_msgs/msg/Path` | Interpolated local path from ego pose to target pose. |
| `/planning/target_speed` | `std_msgs/msg/Float32` | Selected target speed for the current route state. |

## Parameters

Configured by `config/trajectory_generator.yaml`.

| Parameter | Default | Description |
| --- | --- | --- |
| `map_frame` | `world` | Global frame for planning. |
| `vehicle_frame` | `OurCar/INS` | Ego vehicle frame used for TF lookup. |
| `update_rate` | `10.0` | Planning update rate in Hz. |
| `path_resolution` | `1.0` | Approximate spacing between interpolated path points in meters. |
| `min_target_distance` | `0.5` | Minimum distance threshold for very close targets. |
| `default_speed` | `2.0` | Speed used before `/route/speed_limit` is received. |
| `max_speed` | `8.0` | Maximum commanded target speed in normal route states. |
| `slow_speed` | `2.0` | Maximum commanded target speed in intersection states. |
| `stop_speed` | `0.0` | Target speed for stop states. |

## Speed Logic

- If `route_state` contains `stop`, target speed is `stop_speed`.
- If `route_state` contains `intersection`, target speed is
  `min(speed_limit, slow_speed)`.
- Otherwise target speed is `min(speed_limit, max_speed)`.
- If no speed limit has been received, `default_speed` is used.

## Launch

```bash
ros2 launch trajectory_generator trajectory_generator.launch.py
```

Or run the node directly:

```bash
ros2 run trajectory_generator trajectory_generator_node
```

## Current Limitations

- The path is generated using straight-line spatial interpolation only.
- No obstacle avoidance is performed.
- No MPC or dynamic optimization is implemented.
- Traffic-light behavior is represented only through route or decision states
  that affect target speed.
- The node currently consumes `/route/target_pose`. A future extension can
  consume a richer `/route/local_path` if Route Manager publishes multiple
  upcoming waypoints or a precomputed route segment.
