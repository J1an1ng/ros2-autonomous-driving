# Decision Making

This ROS 2 Jazzy package implements the first version of the project's decision layer: a
traffic-light state machine that sits between trajectory planning and vehicle control.

The implementation covers the traffic-light behavior and both optional events from project
task 2.4: slowing for a vehicle merging from the right and emergency braking for an obstacle
directly ahead.

## Responsibility and integration boundary

The node does not generate a path and does not publish low-level throttle, steering, or
brake commands. It accepts the speed proposed by trajectory planning and publishes the
speed that the controller is allowed to follow.

```text
route_manager --------> trajectory_generator ---- /planning/path ------------> controller
       |                         |
       |                         +---- /planning/target_speed ---+
       |                                                        |
       +---- route state and waypoint ID -----------------------+--> decision_making
perception ---- /perception/traffic_light_state ----------------+         |
           +--- /perception/pcl/points_preprocessed --> obstacle_monitor -+
simulation ---- /OurCar/INS/twist ------------------------------+         |
                                                                          v
                                                        /decision/target_speed
                                                                          |
                                                                          v
                                                                      controller
                                                                          |
                                                                          v
                                                                     /car_command
```

There must be only one final publisher of `/car_command`: the controller. The decision
node must not run in parallel with another node that directly overrides the controller's
speed decision.

## Node

Executables:

```text
traffic_light_state_machine_node
obstacle_monitor_node
```

Node name:

```text
/traffic_light_state_machine_node
/obstacle_monitor_node
```

### Subscribed topics

| Topic | Type | Producer | Purpose |
| --- | --- | --- | --- |
| `/perception/traffic_light_state` | `std_msgs/msg/UInt8` | `traffic_light_detector_node` | Stable light state: `0 UNKNOWN`, `1 RED`, `2 YELLOW`, `3 GREEN`. |
| `/planning/target_speed` | `std_msgs/msg/Float32` | `trajectory_generator_node` | Target speed selected by planning, in m/s. |
| `/route/route_state` | `std_msgs/msg/String` | `route_manager_node` | Semantic route state used to enable traffic-light decisions near an intersection. |
| `/route/current_waypoint_id` | `std_msgs/msg/Int32` | `route_manager_node` | Current target waypoint ID, used for optional waypoint filtering and diagnostics. |
| `/route/current_waypoint_distance` | `std_msgs/msg/Float32` | `route_manager_node` | Planar distance to the current target; bounds UNKNOWN-light creeping before the stop line. |
| `/OurCar/INS/twist` | `geometry_msgs/msg/TwistStamped` | simulation bridge | Vehicle velocity used to confirm a complete stop. |
| `/decision/front_obstacle_distance` | `std_msgs/msg/Float32` | `obstacle_monitor_node` | Nearest point in the ego-lane ROI, in metres. `inf` means no point. |
| `/decision/merge_obstacle_distance` | `std_msgs/msg/Float32` | `obstacle_monitor_node` | Nearest point in the right-front merge ROI, in metres. `inf` means no point. |

The `obstacle_monitor_node` subscribes to
`/perception/pcl/points_preprocessed` (`sensor_msgs/msg/PointCloud2`), transforms the points
from their source frame into `OurCar/INS`, fuses the path-relative point cloud with the
semantic vehicle detector, and publishes the two control-facing distances above.

### Published topics

| Topic | Type | Consumer | Meaning |
| --- | --- | --- | --- |
| `/decision/target_speed` | `std_msgs/msg/Float32` | controller | Final permitted target speed in m/s. The controller should use this instead of `/planning/target_speed`. |
| `/decision/state` | `std_msgs/msg/String` | diagnostics | Current state-machine state name. |
| `/decision/stop_required` | `std_msgs/msg/Bool` | controller/diagnostics | `true` while a traffic-light stop is latched. |
| `/decision/debug` | `std_msgs/msg/String` | diagnostics | Compact description of the current state, reason, route context, light and speed output. |

The two control-facing obstacle distances are vehicle-specific fused outputs. Across the
route, a fresh semantic vehicle must agree with the planning-path point-cloud corridor
within the larger of 2.5 m or 20% of semantic range. This rejects semantic cross traffic
and point-cloud-only static structures while enabling longitudinal following at every
waypoint. Point-cloud agreement expires after 0.75 s. Event2 waypoints 43--48 retain a
validated direct-semantic exception because
their point-cloud regions measure unrelated road geometry near the NPC. The raw
vehicle-frame safety cloud is not used for semantic range matching; it remains independent
as a last-resort generic collision guard below 0.8 m.

The controller should treat `/decision/target_speed == 0.0` or
`/decision/stop_required == true` as a request to stop. The exact conversion to throttle
and brake remains the controller's responsibility.

## State machine

| State | Behavior | Transition |
| --- | --- | --- |
| `CRUISE` | Pass through the fresh planning speed. | Enter `APPROACHING_INTERSECTION` in a configured route state/waypoint. |
| `APPROACHING_INTERSECTION` | Unknown/stale data and a remembered red/yellow are limited to `unknown_light_approach_speed` while a fresh waypoint distance remains outside the applicable stop boundary. | At/before the boundary, unknown/stale data or a remembered red/yellow enters `STOPPING_FOR_LIGHT`; continuously fresh green with no pending red/yellow grants one passage and enters `RELEASED`. |
| `STOPPING_FOR_LIGHT` | Publish zero target speed and latch the stop even if route manager advances. | A confirmed green immediately enters `RELEASED`; otherwise, a fresh measured speed below `stopped_speed_threshold` enters `WAITING_FOR_GREEN`. |
| `WAITING_FOR_GREEN` | Continue publishing zero. Unknown, red, yellow or stale light data cannot release the stop. | A continuously fresh green signal for `green_release_delay` enters `RELEASED`. |
| `RELEASED` | Resume planning speed with a one-time passage authorization. Light loss or a later phase change cannot stop the vehicle inside the intersection. | Leaving the controlled route state/waypoint clears the authorization and returns to `CRUISE`. |
| `SLOWING_FOR_MERGE` | Limit the planning speed to `merge_target_speed` while a point is detected in the right-front ROI. A latch older than `merge_stale_timeout` commands a full stop. | Release only after a fresh obstacle observation exceeds the trigger distance plus its hysteresis margin. |
| `EMERGENCY_BRAKING` | Publish zero and request an immediate stop. | Complete a full ego stop, satisfy the minimum hold time, and release only after the front distance exceeds the static following gap plus its hysteresis margin. |
| `FOLLOWING_VEHICLE` | Continuously adjust target speed from gap error and filtered range rate. Hold current speed inside the desired-gap deadband. | Return to planning speed after the lead vehicle leaves the activation range. |

The vehicle-related states are output behaviors. The underlying traffic-light state
remains active, so following or emergency control cannot clear a red-light stop latch.

Decision priority is:

```text
emergency braking > traffic-light stop > stale-merge stop > adaptive following > merge slowdown > planning speed
```

The stop latch is necessary because the current route manager advances to the next
waypoint based only on distance. Without the latch, `/planning/target_speed` can become
non-zero again while the light is still red.

After a stopped vehicle has observed continuously fresh green and entered `RELEASED`, the
authorization remains latched until it leaves the controlled zone. This is intentional:
the signal leaves the camera field of view before the vehicle reaches the route waypoint,
and braking again at that point would stop the vehicle inside the intersection.
The same continuous-green requirement can grant the authorization while the vehicle is
still approaching, so a signal that legitimately leaves the camera field of view before
the hard boundary does not turn back into UNKNOWN braking.

An associated red/yellow is remembered at first recognition. Instead of stopping at the
camera-recognition distance, the vehicle continues at the bounded approach speed and
brakes at `traffic_light_stop_distance`. A green that has passed the perception
confirmation policy immediately clears that brake latch, including during the final
approach. A boundary stop caused only by UNKNOWN may also be cancelled by confirmed green,
or by a fresh distance proving that the hard stop boundary is still ahead.

## Safety and timeout behavior

- A missing or stale planning speed produces `/decision/target_speed = 0.0`.
- A missing or stale vehicle speed cannot falsely confirm that the car has stopped.
- At an allowlisted controlled intersection, missing, unknown or stale traffic-light data
  permits only bounded low-speed approach. It requests a stop at
  `unknown_light_stop_distance` and cannot release `WAITING_FOR_GREEN`.
- A missing or stale waypoint distance cannot authorize UNKNOWN-light creeping.
- Traffic-light loss outside an allowlisted controlled intersection does not stop the
  vehicle. This is necessary because the detector reports `UNKNOWN` during normal driving.
- Non-finite planning and speed values are ignored.
- Yellow is treated as red by default and can be configured otherwise.
- Emergency braking remains latched if obstacle data becomes stale after a hazard was seen.
- A shrinking speed-dependent threshold cannot by itself release emergency braking.
  Full braking may hand back to adaptive following before standstill only after the
  minimum hold time and sustained evidence that the ego vehicle is no longer closing
  significantly on the lead vehicle.
- A latched merge observation survives ordinary point-cloud gaps. Once it is older than
  `merge_stale_timeout`, the state machine requests a fail-safe stop with reason
  `merge_obstacle_stale_stop`; only a fresh safe observation clears the latch.
- Front and merge distances maintain independent receive times. Semantic and raw-safety
  front updates cannot refresh the merge timestamp or publish synthetic merge clearance;
  a new merge observation is emitted only for an actual preprocessed point-cloud sample.
- Emergency distance is the greater of `emergency_min_distance` and
  `emergency_min_distance + closing_speed * reaction_time
  + closing_speed^2 / (2 * deceleration)`. A stable gap therefore does not
  cause full braking merely because ego speed increases.

The distance messages have no header, so front and merge freshness are measured
independently from their local ROS receive times rather than source timestamps.

## Parameters

Defaults are stored in `config/traffic_light_state_machine.yaml`.

| Parameter | Default | Description |
| --- | --- | --- |
| `controlled_route_states` | `intersection_approach`, `stop_zone`, `intersection` | Route states in which a light may trigger a stop. Matching is case-insensitive. |
| `traffic_light_waypoint_ids` | `25, 32, 52, 59` | Stage-4-verified controlled intersections. Waypoint 0 is the initial stop zone and is excluded. |
| `stopped_speed_threshold` | `0.15` | Maximum absolute 3D speed considered stopped, in m/s. |
| `traffic_light_timeout` | `0.75` | Maximum accepted age of a light message, in seconds. |
| `planning_speed_timeout` | `0.5` | Maximum accepted age of a planning-speed message, in seconds. |
| `vehicle_speed_timeout` | `0.5` | Maximum accepted age of a vehicle-speed message, in seconds. |
| `green_release_delay` | `0.3` | Time for which an accepted green must remain continuously fresh before release. |
| `strict_green_waypoint_ids` | `[25]` | Controlled lights where green evidence must clearly exceed red evidence before release. |
| `traffic_light_score_timeout` | `0.75` | Maximum age of the paired red/green evidence scores. |
| `strict_green_over_red_ratio` | `1.05` | Required green-to-red score ratio at strict waypoints. |
| `strict_green_max_red_score` | `4` | Maximum residual red score allowed when accepting green at a strict waypoint. |
| `treat_yellow_as_red` | `true` | Whether yellow triggers the same stop latch as red. |
| `require_green_in_controlled_zone` | `true` | Require a fresh green for unrestricted passage at allowlisted intersections. |
| `waypoint_distance_timeout` | `0.5` | Maximum accepted age of the route waypoint distance, in seconds. |
| `unknown_light_stop_distance` | `4.0` | Hard stop boundary for an unknown/stale light, measured to the controlled waypoint in metres. This stays outside the 3 m waypoint-advance radius. |
| `unknown_light_approach_speed` | `1.0` | Maximum approach speed outside that boundary while the light is unknown/stale, in m/s. |
| `traffic_light_stop_distance` | `4.0` | Stop boundary for an associated red/yellow. The colour is latched when first seen, but the car creeps toward this boundary before braking. |
| `traffic_light_activation_distance` | `20.0` | Maximum distance at which a detected colour is associated with the current controlled waypoint; farther colours are treated as unrelated. |
| `obstacle_timeout` | `1.0` | Maximum accepted age of obstacle distances. The margin covers the approximately 2.5 Hz Unity depth pipeline without merge-latch oscillation. |
| `emergency_min_distance` | `4.0` | Minimum front-obstacle emergency distance, in metres. |
| `emergency_standstill_distance` | `6.0` | Desired remaining gap after stopping behind a vehicle, in metres. |
| `emergency_reaction_time` | `0.5` | Reaction-time term in the dynamic stopping distance, in seconds. |
| `emergency_deceleration` | `0.7` | Conservative available deceleration measured from Event2 full-brake logs, in m/s². |
| `emergency_release_margin` | `2.0` | Additional clearance beyond the static following gap required before releasing a completed emergency stop, in metres. |
| `emergency_min_hold_time` | `1.0` | Minimum emergency-brake latch duration after a front hazard is detected. |
| `emergency_trigger_max_distance` | `10.0` | Near-field ceiling for full emergency activation; farther closing traffic is handled continuously. |
| `emergency_follow_release_min_distance` | `5.4` | Minimum measured gap for handing emergency braking back to adaptive following before standstill. |
| `emergency_follow_release_min_range_rate` | `-0.6` | Minimum filtered gap rate for early handoff, in m/s; the gap may close only slowly. |
| `emergency_follow_release_max_speed` | `2.0` | Maximum ego speed at which early emergency-to-following handoff is permitted, in m/s. |
| `emergency_follow_release_confirmation` | `0.1` | Time for which early-handoff conditions must remain true, in seconds. |
| `merge_trigger_distance` | `12.0` | Right-front obstacle distance that activates early Event2 slowdown before the short-range emergency corridor is needed. |
| `merge_release_margin` | `2.0` | Additional clearance required to release merge slowdown, in metres. |
| `merge_target_speed` | `1.5` | Maximum crawl speed while yielding to a merging vehicle, in m/s. |
| `merge_stale_timeout` | `3.0` | Maximum age of a latched merge observation before a fail-safe stop is requested. A fresh safe observation is required to release it. |
| `following_enabled` | `true` | Enable continuous adaptive vehicle following. |
| `following_activation_distance` | `20.0` | Maximum semantic-vehicle distance at which following control is active, in metres. |
| `following_time_headway` | `2.2` | Time-headway contribution to the desired gap, in seconds. |
| `following_distance_gain` | `0.10` | Gain converting distance error into target-speed correction. |
| `following_range_rate_alpha` | `0.5` | Low-pass weight for new gap-rate samples. |
| `following_range_rate_timeout` | `1.5` | Maximum age of a gap-rate estimate, in seconds. |
| `following_distance_deadband` | `0.75` | Gap-error band in which stable following holds current speed, in metres. |
| `following_range_rate_deadband` | `0.2` | Relative-speed band in which the gap is considered stable, in m/s. |
| `following_target_acceleration` | `1.0` | Maximum upward target-speed slew rate, in m/s². |
| `following_target_deceleration` | `4.0` | Maximum downward target-speed slew rate, in m/s². |
| `following_max_abs_range_rate` | `6.0` | Clamp applied to measured gap rate, in m/s. |
| `vehicle_behavior_global` | `true` | Enable fused longitudinal following and emergency protection at every waypoint. |
| `vehicle_behavior_waypoint_ids` | unset | Optional longitudinal-vehicle allowlist used only when global behavior is disabled. |
| `merge_behavior_waypoint_ids` | `43..48` | Waypoint allowlist for Event2-specific right-front merge yielding. |
| `semantic_direct_control_waypoint_ids` | `43..48` | Validated waypoints where semantic vehicle depth may control without point-cloud range agreement. |
| `update_rate` | `20.0` | Decision output rate, in Hz. |

The obstacle monitor transforms `/planning/path` into `OurCar/INS` and projects each cloud
point onto that polyline. The default emergency corridor covers arc length `0.5..30 m` and
points within 1.2 m of the future path. The merge band covers arc length `0.5..15 m` and
signed path-relative offset `-2.5..-0.8 m`; negative offset is right of the local path
tangent. This prevents a road-end barrier outside a turn from being interpreted as an
in-lane obstacle. A second, last-resort vehicle-frame safety corridor uses the raw
`/perception/pcl/points` camera cloud and covers only `x=0.5..3.5 m` and
`|y|<=1.5 m`; at least five supporting points are required, and its fifth-nearest
distance is combined with the path-relative result. The short upper bound
stays below the known waypoint-48 fixed return near 3.97 m while catching an obstacle that
path projection loses immediately before contact. If the path is missing, stale for more
than 0.75 s, or cannot be
transformed, the same numerical bounds are applied as conservative rectangular
vehicle-frame ROIs. Height defaults to `z=-0.5..2.0 m`; the lower bound rejects curb and
road-shoulder returns near `z=-0.57..-0.67 m` while retaining vehicle-body points.
At waypoint 48, a mapped road-end structure can simultaneously occupy the path, safety and
merge corridors. Suppression is recognized as soon as raw safety is within 4 m while the
latest path and merge distances are both within 9.5 m; it is then latched until the waypoint
changes and omitted from the front-stop output. The merge crawl remains active so steering
can carry the vehicle around the turn. Event2's measured emergency signature remains
actionable because its raw safety distance is near `2.5 m` while the path distance remains
about `11.8..12.7 m`.
`use_latest_tf=true` matches the
upstream occupancy node's transform policy and avoids millisecond-scale future
extrapolation caused by the serialized Unity sensor stream. These values require tuning in
Unity.

The route contains five `stop_zone` waypoints. Stage 4 confirmed that waypoint 0 is the
initial spawn and that the four controlled intersections are 25, 32, 52, and 59. The
waypoint allowlist prevents lights visible on adjacent or departing roads from affecting
the current maneuver.

## Build and run

The obsolete `project/src/CMakeLists.txt` workspace placeholder was removed because colcon
treated it as a package and did not recursively discover packages below `project/src`.

```bash
cd project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select decision_making
source install/setup.bash
ros2 launch decision_making decision_making.launch.py
```

Run the node directly:

```bash
ros2 run decision_making traffic_light_state_machine_node
```

## Tests

The core state machine is ROS-independent. From the repository root:

```bash
PYTHONPATH=project/src/decision_making \
  pytest -q project/src/decision_making/test
```

The tests cover:

- normal planning-speed pass-through;
- red-light stop latching after route-manager waypoint advancement;
- confirmation of a complete stop using only fresh speed data;
- delayed green release;
- rejection of unknown and stale light data while waiting;
- conservative yellow-light handling;
- optional waypoint allowlisting;
- fail-safe green confirmation only at allowlisted intersections;
- distance-bounded UNKNOWN-light approach and stale-distance rejection;
- fail-safe behavior after planning-speed timeout.
- merge-speed limiting and release hysteresis;
- merge-latch retention, stale fail-safe stopping, and fresh-safe release;
- independence of front-only updates from merge-source freshness;
- speed-dependent emergency braking;
- global semantic/point-cloud-confirmed longitudinal following;
- rejection of unmatched semantic cross traffic outside validated direct-control waypoints;
- separation of path-corridor matching from the raw near-field guard;
- behavior priority when traffic-light, merge and emergency events overlap;
- emergency release hysteresis and stale-data fail-safe behavior.

### Manual ROS topic test

Run the decision node, then publish mock inputs in separate terminals:

```bash
ros2 topic pub -r 10 /planning/target_speed std_msgs/msg/Float32 "{data: 4.0}"

ros2 topic pub -r 10 /route/route_state std_msgs/msg/String \
  "{data: 'stop_zone'}"

ros2 topic pub -r 10 /route/current_waypoint_id std_msgs/msg/Int32 "{data: 25}"

ros2 topic pub -r 10 /OurCar/INS/twist geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}"

ros2 topic pub -r 10 /perception/traffic_light_state std_msgs/msg/UInt8 \
  "{data: 1}"
```

Expected result for red:

```bash
ros2 topic echo /decision/target_speed   # 0.0
ros2 topic echo /decision/state          # WAITING_FOR_GREEN after stop confirmation
ros2 topic echo /decision/stop_required  # true
```

Stop the red publisher and publish green continuously:

```bash
ros2 topic pub -r 10 /perception/traffic_light_state std_msgs/msg/UInt8 \
  "{data: 3}"
```

After `green_release_delay`, the expected target speed returns to `4.0` and the state
becomes `RELEASED`.

Obstacle behavior can be tested without a point cloud by publishing the monitor outputs:

```bash
# Vehicle merging from the right: expect SLOWING_FOR_MERGE and at most 2.5 m/s.
ros2 topic pub -r 10 /decision/merge_obstacle_distance std_msgs/msg/Float32 \
  "{data: 6.0}"

# Immediate front hazard: expect EMERGENCY_BRAKING, target speed 0 and stop_required true.
ros2 topic pub -r 10 /decision/front_obstacle_distance std_msgs/msg/Float32 \
  "{data: 2.0}"
```

The retained production Unity configuration uses spawn 0 with both NPC events enabled.
Run the complete route with:

```bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=true \
  enable_visualization:=false
```

## Current limitations and future work

- The four traffic-light waypoint IDs were validated under motion in Stage 4.
- The perception output contains only a color, without confidence, traffic-light ID or
  source timestamp. Route context is therefore used to reduce false triggers.
- The state machine commands a target speed of zero; stopping distance and brake strength
  depend on the future controller.
- It does not modify `/planning/path`.
- Point-cloud ROI detection measures proximity, not object identity or relative velocity.
  Static roadside objects can therefore cause false positives if the ROI is too wide.
- Event2-specific merge yielding is intentionally restricted to waypoints 43--48.
- Global vehicle following requires a fresh semantic/point-cloud range match outside the
  validated Event2 direct-control window; a vehicle temporarily hidden from either sensor
  can therefore be released after the configured timeout.
- The full-route no-event and two-event scenarios have been validated in Unity. Additional
  vehicle placements, lighting conditions and sensor dropouts remain useful regression
  cases.

When decision behaviors are tuned or extended, update the topic tables, priority rules,
parameters, tests and limitations in this README.
