# Vehicle Controller

This ROS 2 Jazzy package implements task 2.5 of the autonomous-driving assignment. It
receives Team 20's time-parameterized trajectory and the final decision-layer speed,
follows the trajectory with Pure Pursuit, regulates speed with PID, and publishes
throttle, steering, and brake commands to the Unity bridge.

## Scope and design

The package reuses Team 20's `autonomy_interfaces/msg/PlannedTrajectory` message and adds
no external control library.

```text
route_manager --> trajectory_generator --> /planning/trajectory --+
                                                                  |
perception ----> decision_making ------> /decision/target_speed ---+--> controller
                                         /decision/stop_required --+       |
simulation ----------------------------> /OurCar/INS/twist --------+       |
simulation ----------------------------> TF world -> OurCar/INS ---+       v
                                                                  /car_command
                                                                        |
                                                                        v
                                                               simulation / Unity
```

There must be only one publisher of `/car_command`. Do not run `dummy_controller` while
the real controller is running.

The implementation is split into:

- `control_core`: ROS-independent path geometry, Pure Pursuit, and PID speed control;
- `controller_node`: ROS subscriptions, TF lookup, input timeouts, safety decisions, and
  publication of `simulation/msg/VehicleControl`.

## Interfaces

### Inputs

| Topic / source | Type | Producer | Use |
| --- | --- | --- | --- |
| `/planning/trajectory` | `autonomy_interfaces/msg/PlannedTrajectory` | `trajectory_generator` | Validated forward poses with curvature, feasible speed, and time from start in `world`. |
| `/decision/target_speed` | `std_msgs/msg/Float32` | `decision_making` | Final permitted speed in m/s. |
| `/decision/stop_required` | `std_msgs/msg/Bool` | `decision_making` | Explicit traffic-light or emergency stop. |
| `/OurCar/INS/twist` | `geometry_msgs/msg/TwistStamped` | `simulation` | Measured vehicle speed. |
| `world -> OurCar/INS` | TF | `simulation` | Vehicle position and heading. |

The controller intentionally does not subscribe to `/planning/target_speed`: doing so
would bypass traffic-light, merge, and emergency decisions. It instead combines the
trajectory speed at the selected Pure Pursuit point with `/decision/target_speed` and
uses the lower value, so neither planning feasibility nor decision-layer safety can be
overridden.

### Output

| Topic | Type | Consumer |
| --- | --- | --- |
| `/car_command` | `simulation/msg/VehicleControl` | Simulation UDP command transmitter |

The message fields are `throttle`, `steering`, `brake`, and the currently unused
`reserved`. Throttle and brake are clamped to `[0, 1]`; steering is clamped to `[-1, 1]`
when normalized steering is enabled. The controller never commands throttle and brake at
the same time.

## Control behavior

### Lateral control

The node finds the trajectory point nearest to the vehicle, walks forward by the
configured lookahead distance, transforms that point into the vehicle frame, and applies
the Pure Pursuit steering law. The lookahead distance increases with measured speed:

```text
lookahead = clamp(lookahead_base + lookahead_speed_gain * speed,
                  lookahead_min, lookahead_max)
```

The steering output is angle-limited and rate-limited. `steering_sign=-1.0` records the
closed-loop-validated Unity convention in which positive Unity steering turns right.

### Longitudinal control

The target speed is `min(decision_target_speed, trajectory_speed_at_lookahead,
maximum_target_speed)`. This joins lateral and longitudinal control at the same selected
trajectory point: the controller cannot follow a bend while using an unrelated scalar
sample. PID uses `/OurCar/INS/twist` as feedback instead of estimating speed from pose
changes. Positive PID output becomes throttle and negative output becomes brake. Output
saturation, integral limiting, a speed-error deadband, and basic anti-windup are included.

### Fail-safe behavior

The controller publishes zero throttle, zero steering, and `safe_stop_brake` when:

- motion is disabled by parameter;
- the trajectory, decision target speed, stop state, or vehicle twist is missing or stale;
- TF is unavailable;
- decision making requests a stop or publishes a non-positive target speed;
- the trajectory has the wrong frame, contains non-finite/negative values, has
  non-increasing point times, exceeds the controller's own bicycle-model curvature
  limit, or has no forward lookahead;
- a control input or calculation is non-finite.

Safe stop also resets PID and previous steering state. The controller starts in a stopped
state until every required input has arrived.

## Build and test

From the repository's `project` directory:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simulation autonomy_interfaces controller
source install/setup.bash
```

Build the complete current stack with:

```bash
colcon build --packages-select \
  simulation perception route_manager autonomy_interfaces trajectory_generator \
  decision_making controller system_bringup
```

Run controller unit tests with:

```bash
colcon test --packages-select controller
colcon test-result --verbose
```

The tests exercise coordinate transformation, lookahead-point index selection,
straight/left/right Pure Pursuit behavior, steering-sign reversal, the most-restrictive
decision/trajectory speed rule, throttle/brake selection and mutual exclusion, and
invalid-input braking.

## Full-stack startup

Build and source the workspace first. Use separate terminals and source
`/opt/ros/jazzy/setup.bash` and `project/install/setup.bash` in every terminal.

1. Start the simulation bridge and Unity:

   ```bash
   ros2 launch simulation simulation.launch.py
   ```

2. Start perception:

   ```bash
   ros2 launch perception perception.launch.py
   ```

3. Start the route manager with its installed route:

   ```bash
   route_file="$(ros2 pkg prefix --share route_manager)/config/route_map.yaml"
   ros2 run route_manager route_manager_node --ros-args \
     -p route_file:="$route_file" \
     -p map_frame:=world \
     -p vehicle_frame:=OurCar/INS \
     -p waypoint_reach_distance:=3.0 \
     -p lookahead_index:=0
   ```

4. Start trajectory generation:

   ```bash
   ros2 launch trajectory_generator trajectory_generator.launch.py
   ```

5. Start decision making:

   ```bash
   ros2 launch decision_making decision_making.launch.py
   ```

6. Check inputs before allowing motion:

   ```bash
   ros2 topic hz /planning/trajectory
   ros2 topic echo /decision/target_speed
   ros2 topic echo /decision/stop_required
   ros2 topic hz /OurCar/INS/twist
   ros2 run tf2_ros tf2_echo world OurCar/INS
   ros2 topic info /car_command
   ```

7. Start the controller:

   ```bash
   ros2 launch controller controller.launch.py
   ```

For a first dry run, start the node directly with motion disabled and watch the log and
command topic:

```bash
ros2 run controller controller_node --ros-args \
  --params-file "$(pwd)/src/controller/config/controller.yaml" \
  -p enable_motion:=false

ros2 topic echo /car_command
```

## Parameters and tuning

Defaults are stored in `config/controller.yaml`.

For production, `system_bringup/full_system.launch.py` owns the vehicle-model values and
injects the same `wheelbase` and `maximum_steering_angle` into controller and trajectory
planner. The YAML values are standalone defaults. The controller independently computes
`tan(maximum_steering_angle) / wheelbase` and rejects any trajectory point beyond that
limit, so a planner/controller configuration mismatch fails safe.

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `map_frame` | `world` | Required frame of `/planning/trajectory`. |
| `vehicle_frame` | `OurCar/INS` | TF frame used as the planar vehicle reference. |
| `enable_motion` | `true` | `false` keeps publishing the safe-stop command. |
| `control_rate` | `20.0` | Command rate in Hz. |
| `wheelbase` | `2.63` | Bicycle-model wheelbase in metres; verify in Unity. |
| `lookahead_base` | `3.0` | Base Pure Pursuit lookahead in metres. |
| `lookahead_speed_gain` | `0.5` | Additional lookahead per m/s. |
| `lookahead_min`, `lookahead_max` | `2.0`, `7.0` | Lookahead bounds in metres. |
| `maximum_target_speed` | `14.0` | Straight-line command ceiling in m/s (50.4 km/h); lower route, decision and event limits still take priority. |
| `steering_sign` | `-1.0` | Measured convention: positive Unity steering turns right, so Pure Pursuit is inverted. |
| `maximum_steering_angle` | `0.52` | Model steering-angle limit in radians. |
| `maximum_steering_rate` | `2.0` | Normalized steering change per second. |
| `speed_kp`, `speed_ki`, `speed_kd` | `0.35`, `0.05`, `0.02` | Longitudinal PID gains. The derivative term acts on measured speed, preventing a target-speed transition from creating a one-cycle throttle/brake kick. |
| `maximum_throttle` | `0.70` | Maximum acceleration command. Half throttle plateaued near 4.56 m/s in Unity; the moderate increase improves straight-line speed while route and decision limits remain authoritative. |
| `maximum_brake` | `1.0` | PID brake ceiling. |
| `safe_stop_brake` | `1.0` | Brake used for missing data and stop requests. |
| `trajectory_timeout` | `1.0` | Maximum planned-trajectory age in seconds. |
| `decision_timeout` | `0.75` | Maximum speed/stop-decision age in seconds. |
| `twist_timeout` | `2.75` | Maximum measured-speed age in seconds. A diagnostic full-stack bag measured a 2.5905 s maximum twist gap under YOLO/mapping load; this leaves a bounded 0.1595 s margin. |

Tune in this order and at low speed:

1. Run with `enable_motion=false` and verify all topics, TF, and single-publisher ownership
   of `/car_command`.
2. Limit route/decision speed and `maximum_throttle`; verify that positive steering turns
   the expected direction. Change only `steering_sign` if it is reversed.
3. Tune `lookahead_base` and bounds. Increase lookahead if steering oscillates; reduce it
   if the vehicle cuts corners or reacts too slowly.
4. Tune `speed_kp` first with `speed_ki=0` and `speed_kd=0`, then add a small integral term
   for steady-state error. Add derivative only if damping is needed and twist is stable.
5. Verify red-light stopping and green release, then merge slowdown and emergency braking.
6. Raise maximum speed or throttle only after low-speed full-route tracking is stable.

The current defaults are the full-system-v1.9.0 closed-loop-validated production values.
Changing vehicle geometry, simulator physics, route speed, PID/lookahead gains, steering
normalization, or braking strength requires a new motion-disabled check followed by
closed-loop regression testing.

The launch integration regression test publishes valid, empty, wrong-frame,
non-monotonic, over-curvature, and stale `PlannedTrajectory` inputs through the public ROS
topic. Only the valid message permits throttle; every other case must produce full-brake
safe stop.
