# Decision Making

This ROS 2 Jazzy package implements the first version of the project's decision layer: a
traffic-light state machine that sits between trajectory planning and vehicle control.

The implementation covers the mandatory traffic-light behavior from project task 2.4. It
does not yet implement the optional vehicle-merging or emergency-braking events.

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

Executable:

```text
traffic_light_state_machine_node
```

Node name:

```text
/traffic_light_state_machine_node
```

### Subscribed topics

| Topic | Type | Producer | Purpose |
| --- | --- | --- | --- |
| `/perception/traffic_light_state` | `std_msgs/msg/UInt8` | `traffic_light_detector_node` | Stable light state: `0 UNKNOWN`, `1 RED`, `2 YELLOW`, `3 GREEN`. |
| `/planning/target_speed` | `std_msgs/msg/Float32` | `trajectory_generator_node` | Target speed selected by planning, in m/s. |
| `/route/route_state` | `std_msgs/msg/String` | `route_manager_node` | Semantic route state used to enable traffic-light decisions near an intersection. |
| `/route/current_waypoint_id` | `std_msgs/msg/Int32` | `route_manager_node` | Current target waypoint ID, used for optional waypoint filtering and diagnostics. |
| `/OurCar/INS/twist` | `geometry_msgs/msg/TwistStamped` | simulation bridge | Vehicle velocity used to confirm a complete stop. |

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
| `update_rate` | `20.0` | Decision output rate, in Hz. |

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

## Current limitations and future work

- The four actual traffic-light waypoint IDs still require track validation.
- The perception output contains only a color, without confidence, traffic-light ID or
  source timestamp. Route context is therefore used to reduce false triggers.
- The state machine commands a target speed of zero; stopping distance and brake strength
  depend on the future controller.
- It does not modify `/planning/path`.
- Vehicle merging and emergency braking from task 2.4 are not implemented yet.
- A project-level launch file must eventually start simulation, perception, route manager,
  trajectory generator, decision making and controller together.

When optional decision behaviors are added, update the topic tables, priority rules,
parameters, tests and limitations in this README.
