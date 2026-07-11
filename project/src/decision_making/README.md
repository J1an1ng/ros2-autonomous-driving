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
| `/OurCar/INS/twist` | `geometry_msgs/msg/TwistStamped` | simulation bridge | Vehicle velocity used to confirm a complete stop. |
| `/decision/front_obstacle_distance` | `std_msgs/msg/Float32` | `obstacle_monitor_node` | Nearest point in the ego-lane ROI, in metres. `inf` means no point. |
| `/decision/merge_obstacle_distance` | `std_msgs/msg/Float32` | `obstacle_monitor_node` | Nearest point in the right-front merge ROI, in metres. `inf` means no point. |

The `obstacle_monitor_node` subscribes to
`/perception/pcl/points_preprocessed` (`sensor_msgs/msg/PointCloud2`), transforms the points
from their source frame into `OurCar/INS`, and publishes the two distance topics above.

### Published topics

| Topic | Type | Consumer | Meaning |
| --- | --- | --- | --- |
| `/decision/target_speed` | `std_msgs/msg/Float32` | controller | Final permitted target speed in m/s. The controller should use this instead of `/planning/target_speed`. |
| `/decision/state` | `std_msgs/msg/String` | diagnostics | Current state-machine state name. |
| `/decision/stop_required` | `std_msgs/msg/Bool` | controller/diagnostics | `true` while a traffic-light stop is latched. |
| `/decision/debug` | `std_msgs/msg/String` | diagnostics | Compact description of the current state, reason, route context, light and speed output. |

The controller should treat `/decision/target_speed == 0.0` or
`/decision/stop_required == true` as a request to stop. The exact conversion to throttle
and brake remains the controller's responsibility.

## State machine

| State | Behavior | Transition |
| --- | --- | --- |
| `CRUISE` | Pass through the fresh planning speed. | Enter `APPROACHING_INTERSECTION` in a configured route state/waypoint. |
| `APPROACHING_INTERSECTION` | Pass through planning speed while observing the light. | Red, or yellow by default, enters `STOPPING_FOR_LIGHT`; leaving the controlled zone returns to `CRUISE`. |
| `STOPPING_FOR_LIGHT` | Publish zero target speed and latch the stop even if route manager advances. | A fresh measured speed below `stopped_speed_threshold` enters `WAITING_FOR_GREEN`. |
| `WAITING_FOR_GREEN` | Continue publishing zero. Unknown, red, yellow or stale light data cannot release the stop. | A continuously fresh green signal for `green_release_delay` enters `RELEASED`. |
| `RELEASED` | Resume the planning speed. | Leaving the controlled zone returns to `CRUISE`; a new red/yellow in the same zone stops again. |
| `SLOWING_FOR_MERGE` | Limit the planning speed to `merge_target_speed` while a point is detected in the right-front ROI. | Release after the obstacle exceeds the trigger distance plus its hysteresis margin. |
| `EMERGENCY_BRAKING` | Publish zero and request an immediate stop. | Release only after a fresh front distance exceeds the speed-dependent stopping distance plus its hysteresis margin. |

`SLOWING_FOR_MERGE` and `EMERGENCY_BRAKING` are output behavior states. The underlying
traffic-light state remains active, so an emergency cannot accidentally clear a red-light
stop latch.

Decision priority is:

```text
emergency braking > traffic-light stop > merge slowdown > planning speed
```

The stop latch is necessary because the current route manager advances to the next
waypoint based only on distance. Without the latch, `/planning/target_speed` can become
non-zero again while the light is still red.

## Safety and timeout behavior

- A missing or stale planning speed produces `/decision/target_speed = 0.0`.
- A missing or stale vehicle speed cannot falsely confirm that the car has stopped.
- A missing, unknown or stale traffic-light state cannot release `WAITING_FOR_GREEN`.
- Traffic-light loss during normal driving does not stop the vehicle by itself. This is
  necessary because the detector reports `UNKNOWN` when no relevant light is visible.
- Non-finite planning and speed values are ignored.
- Yellow is treated as red by default and can be configured otherwise.
- Emergency braking remains latched if obstacle data becomes stale after a hazard was seen.
- A stale merge observation is cleared so it cannot restrict vehicle speed forever.
- Emergency distance is the greater of `emergency_min_distance` and
  `speed * reaction_time + speed^2 / (2 * deceleration)`.

The perception message has no header, so freshness is measured from its local ROS receive
time rather than a source timestamp.

## Parameters

Defaults are stored in `config/traffic_light_state_machine.yaml`.

| Parameter | Default | Description |
| --- | --- | --- |
| `controlled_route_states` | `intersection_approach`, `stop_zone`, `intersection` | Route states in which a light may trigger a stop. Matching is case-insensitive. |
| `traffic_light_waypoint_ids` | `[]` | Optional waypoint allowlist. Empty means all waypoints with a controlled route state. |
| `stopped_speed_threshold` | `0.15` | Maximum absolute 3D speed considered stopped, in m/s. |
| `traffic_light_timeout` | `0.75` | Maximum accepted age of a light message, in seconds. |
| `planning_speed_timeout` | `0.5` | Maximum accepted age of a planning-speed message, in seconds. |
| `vehicle_speed_timeout` | `0.5` | Maximum accepted age of a vehicle-speed message, in seconds. |
| `green_release_delay` | `0.3` | Time for which green must remain continuously fresh before release, in seconds. |
| `treat_yellow_as_red` | `true` | Whether yellow triggers the same stop latch as red. |
| `obstacle_timeout` | `0.5` | Maximum accepted age of obstacle distances, in seconds. |
| `emergency_min_distance` | `4.0` | Minimum front-obstacle emergency distance, in metres. |
| `emergency_reaction_time` | `0.5` | Reaction-time term in the dynamic stopping distance, in seconds. |
| `emergency_deceleration` | `5.0` | Assumed available deceleration in the stopping-distance model, in m/s². |
| `emergency_release_margin` | `2.0` | Additional clearance required to release emergency braking, in metres. |
| `merge_trigger_distance` | `10.0` | Right-front obstacle distance that activates merge slowdown, in metres. |
| `merge_release_margin` | `2.0` | Additional clearance required to release merge slowdown, in metres. |
| `merge_target_speed` | `2.0` | Maximum target speed while yielding to a merging vehicle, in m/s. |
| `update_rate` | `20.0` | Decision output rate, in Hz. |

Obstacle-monitor ROI parameters are in the `OurCar/INS` vehicle frame, following REP-103:
`+x` forward and `+y` left. The default emergency corridor is `x=0.5..30 m` and
`|y|<=1.2 m`. The merge ROI is `x=0.5..15 m`, `y=-4.0..-0.8 m`; negative y is the right
side of the car. Height defaults to `z=-1.0..2.0 m`. These values require tuning in Unity.

The route currently contains five `stop_zone` waypoints although the assignment describes
four traffic lights. Before the final track test, the team must verify the four relevant
waypoint IDs and populate `traffic_light_waypoint_ids`. Until then, the default relies only
on semantic route states.

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
- fail-safe behavior after planning-speed timeout.
- merge-speed limiting and release hysteresis;
- speed-dependent emergency braking;
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
# Vehicle merging from the right: expect SLOWING_FOR_MERGE and at most 2 m/s.
ros2 topic pub -r 10 /decision/merge_obstacle_distance std_msgs/msg/Float32 \
  "{data: 6.0}"

# Immediate front hazard: expect EMERGENCY_BRAKING, target speed 0 and stop_required true.
ros2 topic pub -r 10 /decision/front_obstacle_distance std_msgs/msg/Float32 \
  "{data: 2.0}"
```

## Current limitations and future work

- The four actual traffic-light waypoint IDs still require track validation.
- The perception output contains only a color, without confidence, traffic-light ID or
  source timestamp. Route context is therefore used to reduce false triggers.
- The state machine commands a target speed of zero; stopping distance and brake strength
  depend on the future controller.
- It does not modify `/planning/path`.
- Point-cloud ROI detection measures proximity, not object identity or relative velocity.
  Static roadside objects can therefore cause false positives if the ROI is too wide.
- The merge event is not yet restricted to a verified route segment because the route map
  has no dedicated merge-event label. Its ROI and thresholds require Unity track testing.
- Emergency and merge behaviors have unit tests but still require end-to-end validation
  with perception, planning and the final controller.
- A project-level launch file must eventually start simulation, perception, route manager,
  trajectory generator, decision making and controller together.

When decision behaviors are tuned or extended, update the topic tables, priority rules,
parameters, tests and limitations in this README.
