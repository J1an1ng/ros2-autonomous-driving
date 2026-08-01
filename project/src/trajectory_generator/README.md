# Trajectory Generator

ROS 2 geometric and time-parameterized trajectory-planning package for the
autonomous driving project.

## Purpose

`trajectory_generator_node` consumes the local route from `route_manager`,
looks up the ego vehicle pose from TF, reads current vehicle speed, and
publishes a resampled forward path with a dynamically feasible speed/time
profile. If no valid local route is available, it falls back to the current
short-term route target.

It intentionally does not implement MPC, collision avoidance, or traffic-rule
decisions. Obstacle and traffic-light authority remains in `decision_making`;
the planner constrains only path geometry and vehicle motion along that path.

## Inputs

| Topic | Type | Description |
| --- | --- | --- |
| `/route/local_path` | `nav_msgs/msg/Path` | Primary path source containing upcoming route poses. |
| `/route/target_pose` | `geometry_msgs/msg/PoseStamped` | Current short-term route target. |
| `/route/speed_limit` | `std_msgs/msg/Float32` | Speed limit associated with the route target. |
| `/route/route_state` | `std_msgs/msg/String` | Semantic route state such as `normal`, `stop_zone`, or `intersection`. |
| `/route/current_waypoint_id` | `std_msgs/msg/Int32` | Current target waypoint id, used for diagnostics. |
| `/OurCar/INS/twist` | `geometry_msgs/msg/TwistStamped` | Current speed used as the first boundary condition of the velocity profile. |
| TF | `map_frame -> vehicle_frame` | Ego pose lookup. Defaults to `world -> OurCar/INS`. |

## Outputs

| Topic | Type | Description |
| --- | --- | --- |
| `/planning/path` | `nav_msgs/msg/Path` | Smoothed, validated, forward reference path. |
| `/planning/trajectory` | `autonomy_interfaces/msg/PlannedTrajectory` | Team 20 message containing pose, curvature, feasible speed, and time from start at every trajectory point. |
| `/planning/target_speed` | `std_msgs/msg/Float32` | Dynamically feasible lookahead speed used by the existing decision/controller chain. |

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
| `max_curvature` | `2.0` | Additional configured absolute-curvature ceiling in `1/m`. |
| `wheelbase` | `2.63` | Bicycle-model wheelbase in meters; shared with the production controller. |
| `maximum_steering_angle` | `0.52` | Physical steering-angle ceiling in radians; shared with the production controller. |
| `max_yaw_step` | `1.1` | Maximum allowed wrapped yaw change between adjacent samples in radians. |
| `max_start_distance` | `5.0` | Maximum distance in meters between ego and the first generated point. |
| `max_spline_deviation` | `2.5` | Maximum allowed smoothing deviation from the cropped input polyline in meters. |
| `use_waypoint_yaw` | `true` | Use route-pose yaw as a tangent constraint when interpolating sparse waypoints. |
| `continuity_blend_distance` | `5.0` | Distance from path start over which the previous and current paths are blended. Set to `0.0` to disable blending. |
| `continuity_candidate_weight` | `0.5` | Initial weight of the new path in the continuity blend; clamped to `[0, 1]`. |
| `max_continuity_deviation` | `2.0` | Maximum corresponding-point difference over the blend window. Larger path jumps or branch changes are used without blending. |
| `default_speed` | `2.0` | Speed used before `/route/speed_limit` is received. |
| `max_speed` | `14.0` | Maximum nominal target speed (50.4 km/h); lower route waypoint limits still take priority where required by path geometry. |
| `stop_speed` | `0.0` | Terminal target speed for `route_complete`. |
| `max_lateral_acceleration` | `3.0` | Maximum planned `v² |curvature|` in m/s². |
| `max_longitudinal_acceleration` | `2.0` | Maximum planned forward acceleration in m/s². |
| `max_longitudinal_deceleration` | `3.0` | Maximum planned braking magnitude in m/s². |
| `target_speed_lookahead_distance` | `4.0` | Arc-length lookahead used to expose one feasible speed to the legacy scalar-speed interface. |
| `velocity_timeout` | `2.75` | Maximum age of measured speed, matching the controller. A diagnostic bag measured a 2.5905 s maximum gap under full load; without a fresh measurement the planner publishes no trajectory and the controller safe-stops. |

## Kinematic and Dynamic Feasibility

Path feasibility is established first: samples must be finite, ordered in arc
length, close to the ego pose, bounded in curvature and yaw change, and within
the configured smoothing-deviation envelope. The curvature limit is linked to
the same bicycle model used by the controller:

```text
kappa_vehicle = tan(maximum_steering_angle) / wheelbase
kappa_effective = min(max_curvature, kappa_vehicle)
```

With the production values, `kappa_effective = 0.2177 1/m`. The accepted path
is then time parameterized in four deterministic passes:

1. Apply route and global speed limits, then cap every point by lateral
   acceleration:

   ```text
   v_curve = sqrt(max_lateral_acceleration / abs(curvature))
   ```

2. Run a forward reachability pass from measured ego speed:

   ```text
   v[i] <= sqrt(v[i-1]^2 + 2 * max_longitudinal_acceleration * ds)
   ```

3. Run a backward braking pass, including a zero terminal speed for
   `route_complete`:

   ```text
   v[i] <= sqrt(v[i+1]^2 + 2 * max_longitudinal_deceleration * ds)
   ```

4. Integrate point time with the trapezoidal relation
   `dt = 2 * ds / (v[i-1] + v[i])`. The denominator has a small numerical floor,
   so every non-duplicate spatial sample has strictly increasing time.

The measured speed at point zero is a hard boundary condition, not a value that
the backward pass may silently reduce. If the available distance cannot satisfy
a lower curvature, route, or terminal cap at the configured deceleration, the
profile is reported as infeasible. The node publishes an empty trajectory and a
zero scalar target for that planning cycle; the controller then safe-stops until
a reachable profile is available. This makes an unsafe boundary mismatch visible
instead of claiming a dynamically impossible trajectory.

The implementation is a pure computational module in `velocity_profile.py`.
It has no ROS dependency and is covered by focused tests for curvature limits,
acceleration reachability, braking before a curve, terminal stopping, and
monotonic time. This separation keeps policy, transport, and trajectory
mathematics independently testable.

## Speed and Control Integration

- At `route_complete`, the profile ends at `stop_speed`.
- A `stop_zone` remains a nominal route state and uses its speed limit. Dynamic
  traffic-light stops are owned by the decision node, so a stable green can
  release the vehicle without a second planning-layer zero-speed latch.
- For every non-terminal route state, including intersection states, the
  nominal ceiling is `min(speed_limit, max_speed)`. Route semantics do not add
  a generic intersection slowdown; path curvature and longitudinal limits
  shape the actual profile below that ceiling.
- If no speed limit has been received, `default_speed` is used.

`/planning/trajectory` is the complete controller-facing representation. The
controller selects one Pure Pursuit lookahead point and uses the speed stored on
that same point. It caps this speed with `/decision/target_speed`, so the dynamic
profile and traffic-light/obstacle decision are both authoritative. The scalar
`/planning/target_speed` still selects the profile sample at the configured 4 m
lookahead for `decision_making`; `/planning/path` remains available to the
obstacle corridor and RViz. The planner cannot override a safety decision.

## Team 20 Custom Interface

The `autonomy_interfaces` package is Team 20 code and supplies the assignment's
custom ROS message:

```text
TrajectoryPoint
  geometry_msgs/Pose pose
  float32 curvature
  float32 target_speed
  builtin_interfaces/Duration time_from_start

PlannedTrajectory
  std_msgs/Header header
  autonomy_interfaces/TrajectoryPoint[] points
```

A topic is appropriate because a trajectory is a continuously refreshed data
stream, not a synchronous request/reply operation. The standard `Path` output
is retained for compatibility and RViz; the custom message makes curvature,
speed, and timing explicit and inspectable.

## Path Processing

The local-route pipeline runs in this order:

1. Convert `/route/local_path` poses to internal 2D route points. Collapse
   sub-metre survey clusters while preserving the latest pose and tangent.
2. Project the ego position onto every route segment. Select the nearest
   0.5-meter candidate corridor first, then use wrapped heading error to
   disambiguate nearby route branches without preferring a distant segment.
3. Crop by route order at that projection. This keeps only increasing route
   arc length, even when a later portion lies geometrically behind the ego.
4. When enabled, interpolate sparse route segments with cubic Hermite curves
   constrained by waypoint yaw. Replace remaining sharp corners with bounded
   quadratic Bézier sections, then resample at `path_resolution` along arc
   length.
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

`nav_msgs/msg/Path` carries position and yaw only. The parallel Team 20
`PlannedTrajectory` message exposes the internally computed curvature, feasible
speed, and timing without breaking existing consumers.

## Launch

```bash
ros2 launch trajectory_generator trajectory_generator.launch.py
```

The standalone launch exposes `wheelbase` and `maximum_steering_angle`. Production
`full_system.launch.py` passes one shared pair to both trajectory planner and controller;
changing either vehicle-model value therefore cannot silently update only one side.

Or run the node directly:

```bash
ros2 run trajectory_generator trajectory_generator_node
```

## Current Limitations

- The fallback path uses straight-line spatial interpolation only.
- The planner uses a bicycle-model curvature ceiling and a point-mass
  acceleration envelope; it does not model tyre friction circles, steering
  actuator rate, jerk, mass, or drivetrain dynamics.
- Hermite/Bézier geometry is validated and time parameterized, not jointly
  optimized as an MPC problem.
- No obstacle avoidance is performed.
- No MPC or dynamic optimization is implemented.
- Traffic-light and obstacle decisions are intentionally left to
  `decision_making`.
- Route completion is declared by `route_manager` only after the vehicle enters the final
  waypoint reach radius. If it is still moving at that instant, zero terminal speed is
  no longer reachable over the remaining zero-length horizon; the planner explicitly
  rejects that profile and the controller safe-stops. Anticipatory stopping would require
  an upstream final-waypoint semantic before the reach event.

## Validation Evidence

The latest 2026-08-01 waypoint 0--63 Unity run audited 70,008 points from 2,115
`/planning/trajectory` messages. Maximum absolute curvature was 0.217622 1/m against the
steering-derived 0.217704 1/m limit. Maximum observed lateral acceleration,
longitudinal acceleration, and longitudinal deceleration were respectively 3.0000,
2.0008, and 3.0000 m/s². Before route completion, 2,096 of 2,103 trajectory
publications were nonempty; the 0.33% empty fraction, 0.596 s longest continuous empty
interval, and 0.192 s maximum message gap passed the documented 5% / 10 s / 1.05 s
availability gates. All 102 runtime local
path rejections were classified as curvature-gate rejections. Each used the straight
target-pose fallback, which was rechecked against the same feasibility rules before
publication; an invalid fallback would instead trigger controller safe stop.
The 0.0008 m/s² acceleration excess is within the documented float/chord numerical
tolerance; all timestamps were strictly increasing and all values were finite. See
[`../../mission_results/README.md`](../../mission_results/README.md) for mission time,
plots, and machine-readable metrics.
