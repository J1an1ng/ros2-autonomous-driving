# Route Manager

## Purpose and ownership

`route_manager` is Team 20's deterministic route-context package and Lei Fang's primary
contribution. It loads the recorded course route, tracks vehicle progress, and publishes
the ordered local route and semantic context used by trajectory generation and decision
making.

The package deliberately does not smooth paths, detect obstacles, make traffic decisions,
or produce vehicle commands. Those responsibilities belong to `trajectory_generator`,
`perception`, `decision_making`, and `controller` respectively.

## Design choice

The assessed Unity map and mission are fixed, so the project uses a reviewed YAML route
instead of a general-purpose global planner. This makes every waypoint, speed limit, stop
reference, and route state reproducible and traceable in `config/route_map.yaml`.

The trade-off is that a changed map or destination requires a new route recording. The
runtime manager follows route order rather than choosing a new global path around a
blockage.

## Nodes

### `route_manager_node`

The production node runs at 10 Hz and:

1. loads and validates the configured route file;
2. looks up `world -> OurCar/INS` (configurable) in TF;
3. marks a waypoint reached inside its radius or passed after the ego vehicle crosses its
   forward progress plane;
4. publishes the current target, local path, distance, type, ID, and speed limit;
5. publishes `route_complete` and zero route speed after waypoint 63.

Using a forward progress plane as a second completion condition prevents a slightly
missed waypoint from stalling the route. Waypoint 0 is an exception: it must be reached
and cannot be skipped only by being geometrically beyond it.

The local path normally retains the previous waypoint before the active target. This lets
the trajectory generator project the vehicle onto the active route segment instead of
drawing a straight line directly to a distant waypoint.

### `waypoint_recorder`

The interactive recorder reads the current vehicle pose from TF and writes a YAML route.
It records position, yaw, type, speed, and segment membership, and derives segment/zone
metadata. It is a route-authoring tool and is not started by the production full-system
launch.

## Route data model

The installed production route contains IDs 0–63. Its top-level YAML fields are:

| Field | Meaning |
| --- | --- |
| `map_frame` | Global route frame, normally `world`. |
| `vehicle_frame` | Vehicle TF frame used while recording. |
| `waypoints` | Ordered waypoint dictionaries consumed at runtime. |
| `segments` | Traceability metadata grouping waypoint ranges. |
| `zones` | Metadata for stop, curve, and intersection-approach regions. |

Runtime waypoint fields are:

| Field | Required | Meaning |
| --- | --- | --- |
| `x`, `y` | yes | Position in metres in the map frame. |
| `id` | recommended | Public route ID; list index is the fallback. |
| `z` | no | Height in metres; default `0.0`. |
| `yaw_deg` | no | Recorded heading in degrees; default `0.0`. |
| `speed_limit` | no | Nominal route speed in m/s; default `2.0`. |
| `type` | no | Semantic route state; default `normal`. |
| `reach_distance` | no | Per-waypoint reach-radius override in metres. |
| `segment_id` | no | Traceability group used by the route file. |

The loader rejects an empty path, missing file, non-file path, malformed YAML structure,
an empty waypoint list, non-dictionary waypoints, and waypoints missing `x` or `y`.

## Runtime interfaces

The node has no ROS topic subscription. Vehicle position comes from TF.

| Output topic | Type | Consumer and meaning |
| --- | --- | --- |
| `/route/target_pose` | `geometry_msgs/msg/PoseStamped` | Current short-term target for trajectory fallback. |
| `/route/local_path` | `nav_msgs/msg/Path` | Previous/current/upcoming ordered waypoint window for trajectory generation. |
| `/route/speed_limit` | `std_msgs/msg/Float32` | Nominal speed limit for the selected target, in m/s. |
| `/route/current_waypoint_id` | `std_msgs/msg/Int32` | Public ID of the current target waypoint. |
| `/route/current_waypoint_distance` | `std_msgs/msg/Float32` | Planar ego-to-target distance in metres. |
| `/route/route_state` | `std_msgs/msg/String` | Current semantic type or `route_complete`. |

Route semantics provide context but do not automatically impose a generic intersection
slowdown. Dynamic traffic-light stops are owned by `decision_making`; keeping a non-zero
planning speed at the controlled signal waypoints allows a confirmed green to release the
vehicle without a route-layer deadlock.

## Parameters

| Parameter | Default | Constraint and purpose |
| --- | ---: | --- |
| `route_file` | empty | Must resolve to an installed/readable YAML file. The full-system launch supplies the installed production route. |
| `map_frame` | `world` | Global frame used in route messages and TF lookup. |
| `vehicle_frame` | `OurCar/INS` | Ego TF frame. |
| `waypoint_reach_distance` | `3.0` | Global reach radius in metres; use a positive value. A waypoint may override it. |
| `lookahead_index` | `1` | Offset from the progress index to the published target. Full-system production uses `0`. |
| `local_path_size` | `5` | Number of local route poses; must be greater than zero. |
| `initial_waypoint_index` | `0` | Diagnostic start index; must be inside the route. Full benchmarks use `0`. |

Waypoint 25 uses `reach_distance: 1.5` so its distance reference remains active through
the configured 2 m traffic-light stop boundary. Controlled traffic-light waypoints 25,
32, 52, and 59 use an 8 m/s far-field route ceiling; decision making applies the final
distance-based stop envelope. Geometry-specific curve limits remain in the route file.

## Build and run

From `project/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select route_manager --symlink-install
source install/setup.bash
```

The recommended production entry point is:

```bash
ros2 launch system_bringup full_system.launch.py enable_motion:=false
```

For a standalone route-manager run using the installed route:

```bash
route_file="$(ros2 pkg prefix --share route_manager)/config/route_map.yaml"
ros2 run route_manager route_manager_node --ros-args \
  -p route_file:="$route_file" \
  -p lookahead_index:=0
```

Run the recorder only when intentionally creating a new route. Use an explicit source-tree
output path so the recorded file is reviewable before installation:

```bash
ros2 run route_manager waypoint_recorder --ros-args \
  -p output_file:=/absolute/path/to/project/src/route_manager/config/route_map.yaml
```

Recorder commands include `n`, `ca`, `ce`, `c`, `cx`, `ia`, `s`, `i`, `ix`, `slow`,
`seg <id>`, `undo`, `list`, and `q`.

## Tests and validation

```bash
colcon test --packages-select route_manager
colcon test-result --verbose
```

Tests cover route-file validation, reach-radius and progress-plane behavior, waypoint 25's
local reach override, all 64 waypoint IDs, protected curve/intersection geometry, speed
profiles, recorder defaults, flake8, and PEP 257. In full-system-v1.9.0 the node advanced
from waypoint 0 through 63 and logged `Route completed at waypoint 63`.

## Limitations

- This is route tracking, not global planning or obstacle avoidance.
- Progress is planar; height does not affect waypoint completion.
- The route must be rerecorded or edited when the map/mission changes.
- `segments` and `zones` are traceability metadata; runtime behavior is driven by the
  ordered waypoint fields and downstream decision logic.
