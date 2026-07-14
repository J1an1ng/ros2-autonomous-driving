# Trajectory Generator

ROS 2 reference-path generation package for the autonomous driving project.

## Purpose

`trajectory_generator_node` consumes the local route from `route_manager`,
looks up the ego vehicle pose from TF, and publishes a resampled forward path
plus a nominal target speed. If no valid local route is available, it falls
back to the current short-term route target.

It intentionally does not implement MPC, collision avoidance, or traffic-rule
decisions. Those remain downstream responsibilities.

## Inputs

| Topic | Type | Description |
| --- | --- | --- |
| `/route/local_path` | `nav_msgs/msg/Path` | Primary path source containing upcoming route poses. |
| `/route/target_pose` | `geometry_msgs/msg/PoseStamped` | Current short-term route target. |
| `/route/speed_limit` | `std_msgs/msg/Float32` | Speed limit associated with the route target. |
| `/route/route_state` | `std_msgs/msg/String` | Semantic route state such as `normal`, `stop_zone`, or `intersection`. |
| `/route/current_waypoint_id` | `std_msgs/msg/Int32` | Current target waypoint id, used for diagnostics. |
| TF | `map_frame -> vehicle_frame` | Ego pose lookup. Defaults to `world -> OurCar/INS`. |

## Outputs

| Topic | Type | Description |
| --- | --- | --- |
| `/planning/path` | `nav_msgs/msg/Path` | Smoothed, validated, forward reference path. |
| `/planning/target_speed` | `std_msgs/msg/Float32` | Nominal target speed selected from route information. |

## Parameters

Configured by `config/trajectory_generator.yaml`.

| Parameter | Default | Description |
| --- | --- | --- |
| `map_frame` | `world` | Global frame for planning. |
| `vehicle_frame` | `OurCar/INS` | Ego vehicle frame used for TF lookup. |
| `update_rate` | `10.0` | Planning update rate in Hz. |
| `path_resolution` | `1.0` | Approximate spacing between resampled path points in meters. |
| `min_target_distance` | `0.5` | Minimum distance threshold for very close targets. |
| `smoothing_strength` | `0.25` | Fraction of each adjacent segment trimmed when forming the bounded quadratic Bézier spline; clamped to `0.45`. |
| `max_curvature` | `2.0` | Maximum allowed absolute path curvature in `1/m`. |
| `max_yaw_step` | `1.1` | Maximum allowed wrapped yaw change between adjacent samples in radians. |
| `max_start_distance` | `5.0` | Maximum distance in meters between ego and the first generated point. |
| `max_spline_deviation` | `2.0` | Maximum allowed smoothing deviation from the cropped input polyline in meters. |
| `continuity_blend_distance` | `5.0` | Distance from path start over which the previous and current paths are blended. Set to `0.0` to disable blending. |
| `continuity_candidate_weight` | `0.5` | Initial weight of the new path in the continuity blend; clamped to `[0, 1]`. |
| `max_continuity_deviation` | `2.0` | Maximum corresponding-point difference over the blend window. Larger path jumps or branch changes are used without blending. |
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

This speed is nominal planning output. Traffic-light and obstacle decisions are
applied downstream by `decision_making`, which publishes the final target speed
and stop requirement.

## Path Processing

The local-route pipeline runs in this order:

1. Convert `/route/local_path` poses to internal 2D route points and remove
   consecutive duplicates within 1 mm.
2. Project the ego position onto every route segment. Select the nearest
   0.5-meter candidate corridor first, then use wrapped heading error to
   disambiguate nearby route branches without preferring a distant segment.
3. Crop by route order at that projection. This keeps only increasing route
   arc length, even when a later portion lies geometrically behind the ego.
4. Replace sharp corners with bounded quadratic Bézier spline sections, then
   resample the complete path at `path_resolution` along arc length.
5. Derive yaw from neighboring samples and signed curvature from each
   three-point circumcircle. Every internal point also stores cumulative `s`.
6. Reject paths containing non-finite values, non-increasing `s`, excessive
   sample spacing, yaw jumps, curvature, start distance, or spline deviation.
7. For the same path source, reproject the previous published path to the
   current ego pose and blend it with the new path near the ego. Source changes,
   excessive path deviation, and route-branch changes reset continuity.
   Geometry and quality checks are rerun after blending; an invalid blend falls
   back to the valid current candidate.

When a supplied local path is rejected, the node emits a throttled warning with
the quality reason and reports whether target-pose fallback is being used.

If the local path is unavailable or rejected, `/route/target_pose`
straight-line interpolation is used. If neither source is usable, an empty
`/planning/path` is published and continuity state is cleared.

`nav_msgs/msg/Path` carries position and yaw only. Arc length and curvature are
computed and validated internally; adding them to a controller-facing message
can be done later without changing this path-generation pipeline.

## Launch

```bash
ros2 launch trajectory_generator trajectory_generator.launch.py
```

Or run the node directly:

```bash
ros2 run trajectory_generator trajectory_generator_node
```

## Current Limitations

- The fallback path uses straight-line spatial interpolation only.
- The spline is a local geometric smoother, not a vehicle-model-constrained
  optimizer.
- Curvature currently gates path quality but does not yet shape target speed.
- No obstacle avoidance is performed.
- No MPC or dynamic optimization is implemented.
- Traffic-light and obstacle decisions are intentionally left to
  `decision_making`.
