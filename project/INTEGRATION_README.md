# Team 20 Integration and Simulation Plan

This document is the source of truth for integration work on the `temp_inte` branch. Before
starting or changing any stage, check its entry criteria, test procedure, pass criteria,
and required evidence here. Update the status table and test log after completing a stage.

The final objective follows the assignment: start from benchmark spawn position 0, drive
the predefined urban route without leaving the road or colliding, obey four dynamic traffic
lights, handle the enabled NPC events, reach the goal, and run the complete ROS graph from
one launch file.

## Status

| Stage | Scope | Status | Evidence |
| --- | --- | --- | --- |
| 0 | Integration entry point and configuration | Passed | Build, unit tests, safe headless launch; see test log |
| 1 | Simulation bridge and sensor/TF contract | Passed | Unity bridge, sensor/TF, and low-amplitude command checks; see test log |
| 2 | Low-speed controller validation | Passed | Straight/curve tracking and six safety injections; see test log |
| 3 | Route, trajectory, and no-event closed loop | Not started | — |
| 4 | Perception pipeline and traffic-light sensing | Not started | — |
| 5 | Decision making and individual events | Not started | — |
| 6 | Full benchmark and repeatability | Not started | — |

Status values are `Not started`, `In progress`, `Blocked`, and `Passed`. A later stage must
not be marked `Passed` while an earlier required stage is blocked.

## System contract

```text
Unity / simulation bridge
  ├─ /OurCar/INS/pose, /OurCar/INS/twist, TF, camera and depth
  ├─> route_manager ─> trajectory_generator ─> /planning/path
  ├─> perception ─────> decision_making ──────> final target speed / stop
  └─< /car_command <──────────── controller <── path + decision + twist + TF
```

Only `controller_node` may publish `/car_command` during integration. Never run
`dummy_controller_node` at the same time.

### Topics that must remain stable

| Topic | Type | Expected rate / behavior |
| --- | --- | --- |
| `/OurCar/INS/pose` | `geometry_msgs/msg/PoseStamped` | Continuous while Unity is connected |
| `/OurCar/INS/twist` | `geometry_msgs/msg/TwistStamped` | Continuous while Unity is connected |
| `/OurCar/Sensors/RGBCameraLeft/image_raw` | `sensor_msgs/msg/Image` | Sensor rate |
| `/OurCar/Sensors/SemanticCamera/image_raw` | `sensor_msgs/msg/Image` | Sensor rate |
| `/OurCar/Sensors/DepthCamera/image_raw` | `sensor_msgs/msg/Image` | Sensor rate |
| `/route/local_path` | `nav_msgs/msg/Path` | About 10 Hz |
| `/planning/path` | `nav_msgs/msg/Path` | About 10 Hz, frame `world` |
| `/planning/target_speed` | `std_msgs/msg/Float32` | About 10 Hz, nominal speed |
| `/perception/traffic_light_state` | `std_msgs/msg/UInt8` | `0 UNKNOWN`, `1 RED`, `2 YELLOW`, `3 GREEN` |
| `/decision/target_speed` | `std_msgs/msg/Float32` | About 20 Hz, final permitted speed |
| `/decision/stop_required` | `std_msgs/msg/Bool` | About 20 Hz |
| `/car_command` | `simulation/msg/VehicleControl` | About 20 Hz from exactly one publisher |

### Evidence to collect during runtime stages

Use rosbag for the smallest relevant topic set. The full benchmark set is:

```text
/OurCar/INS/pose
/OurCar/INS/twist
/route/current_waypoint_id
/route/route_state
/route/local_path
/planning/path
/planning/target_speed
/perception/traffic_light_state
/perception/traffic_light_debug
/decision/target_speed
/decision/stop_required
/decision/state
/decision/debug
/decision/front_obstacle_distance
/decision/merge_obstacle_distance
/car_command
/tf
/tf_static
```

Record the Git commit, simulation config, controller config, decision config, route map,
test date, machine, and observed result with every bag.

## Stage 0 — Integration entry point and configuration

### Goal

Make the stack deterministic and launchable before allowing vehicle motion. Stage 0 is a
build and startup gate, not a driving test.

### Required implementation

- Use spawn position 0 as the common route origin.
- Disable optional NPC events for the first no-event stages; later stages enable them
  explicitly.
- Use the ROS 2 bridge camera topic names consistently.
- Install the route map into the `route_manager` package share directory.
- Provide one `full_system.launch.py` that can start the complete graph.
- Expose launch switches so stages can enable only the modules they need.
- Default the full-system launch to `enable_motion:=false`.
- Declare all runtime dependencies, including OctoMap.

### Entry criteria

- ROS 2 Jazzy on Ubuntu 24.04.
- Git LFS assets present; the Unity executable is a real ELF file, not an LFS pointer.
- Clean `temp_inte` working tree before implementation.

### Procedure

1. Install dependencies listed in `project/src/README.md` and package manifests.
2. From `project/`, run a clean build of all seven functional packages plus bringup.
3. Source the resulting install space.
4. Validate launch syntax and installed paths.
5. Start the full-system launch with Unity disabled and controller motion disabled:

   ```bash
   ros2 launch system_bringup full_system.launch.py \
     start_unity:=false \
     enable_motion:=false
   ```

6. Confirm that nodes start or wait safely for simulation data. Startup must not fail
   because a config or executable is missing.
7. Confirm `/car_command` has no more than one publisher.

### Pass criteria

- All packages build and install without errors.
- Route map, controller YAML, decision YAML, trajectory YAML, Unity config, and launch files
  resolve from installed package paths.
- Full-system launch starts the expected nodes from one command.
- No dummy controller is included.
- Controller remains in safe stop while `enable_motion=false` or required inputs are absent.
- Default Unity configuration uses spawn 0 and optional events are disabled.
- Camera topic configuration matches the bridge topic contract.

### Exit artifact

- One Stage 0 commit on `temp_inte`.
- Build/test summary added to the test log at the end of this document.

## Stage 1 — Simulation bridge, sensors, and TF

### Goal

Prove that Unity and ROS exchange data correctly before any autonomous motion.

### Configuration

- `spawnIndex: 0`
- `eventEnable: [false, false]`
- `enable_motion:=false`
- Start simulation, perception may remain disabled initially.

### Procedure

1. Start only simulation and safe controller components through bringup switches.
2. Confirm Unity connects to the TCP receiver on the configured sensor port.
3. Measure pose, twist, RGB, semantic, depth, and CameraInfo topic rates.
4. Inspect frame IDs and image encodings.
5. Verify the complete TF chain:

   ```text
   world -> OurCar/INS -> OurCar/Center -> OurCar/Sensors/SensorBase
         -> DepthCamera / RGBCameraLeft / RGBCameraRight / SemanticCamera
   ```

6. Compare a stationary pose and a short manually controlled movement with Unity.
7. Publish a safe zero command, then small isolated throttle/brake/steering commands only
   with the test area clear.
8. Restart Unity and the bridge once to document reconnect behavior.

### Measurements

- Topic rates and dropped-message symptoms.
- Pose axes and yaw convention.
- Twist units and sign.
- Image sizes, FOV, encoding, and depth scale.
- Steering sign and whether Unity expects normalized steering.
- UDP command latency and behavior when command publication stops.

### Pass criteria

- Required topics are continuous and use the documented names.
- TF has no missing or duplicate parent relationships.
- Depth values correspond approximately to visible distances.
- Positive throttle moves forward, brake stops, and steering convention is known.
- No autonomous motion occurs while disabled.

## Stage 2 — Low-speed controller validation

### Goal

Validate Pure Pursuit and PID against Unity using controlled paths, without route,
perception, or event uncertainty.

### Entry criteria

- Stage 1 passed.
- Steering convention and command ranges documented.
- Open test area and conservative speed/throttle limits.

### Procedure

1. Keep `enable_motion=false` and publish a straight mock path, target speed, stop state,
   twist, and TF; inspect computed safe behavior.
2. Enable motion with target speed at approximately `0.5–1.0 m/s`.
3. Test straight tracking in both initial headings.
4. Test one gentle left curve and one gentle right curve.
5. Trigger every safety input independently:
   - stale/missing path;
   - stale/missing target speed;
   - stale/missing stop state;
   - stale/missing twist;
   - missing TF;
   - explicit stop request.
6. Tune in this order:
   - `steering_sign` and normalized range;
   - wheelbase;
   - lookahead base/min/max and speed gain;
   - steering-rate limit;
   - P gain, then I, then D;
   - throttle and brake ceilings.

### Measurements

- RMS and maximum cross-track error.
- Heading error and steering oscillation frequency.
- Speed rise time, overshoot, steady-state error, and braking distance.
- Count and reason for safe stops.
- Time spent at throttle/brake/steering saturation.

### Pass criteria

- Correct left/right steering with no persistent oscillation.
- Stable straight and gentle-curve tracking at low speed.
- Throttle and brake are never positive together.
- Every stale-input and explicit-stop test produces safe braking.
- Controller parameters and their measured effects are recorded.

## Stage 3 — Route, trajectory, and no-event closed loop

### Goal

Drive the predefined route with geometry, trajectory, and control only. Traffic lights are
mocked green or decision is supplied a safe pass-through input; NPC events stay disabled.

### Entry criteria

- Stages 1 and 2 passed.
- Spawn 0 confirmed to match the start of `route_map.yaml`.
- Maximum speed initially limited to `1–2 m/s`.

### Procedure

1. Verify waypoint 0 and the first local path before enabling motion.
2. Run route manager and trajectory generator with controller disabled.
3. Inspect every path message for:
   - `world` frame;
   - first point near ego;
   - forward ordering;
   - no fallback/rejection warnings;
   - reasonable spacing, yaw, and curvature.
4. Enable low-speed motion and drive route segments in increasing length.
5. Observe every waypoint transition, especially duplicate/close points and long gaps.
6. Test recovery when the vehicle passes more than `waypoint_reach_distance` from a point.
7. Complete the route with events disabled.
8. Increase speed only after the full low-speed route passes.

### Measurements

- Waypoint ID over time and missed/stuck waypoint count.
- Path rejection/fallback count.
- Distance from ego to first planned point.
- Cross-track error per segment and curve.
- Route completion time and final stopping behavior.

### Pass criteria

- Route progresses from start to final waypoint without manual index changes.
- No straight-line fallback crosses buildings, curbs, or opposing lanes.
- Vehicle remains on the road through all turns.
- Local path updates do not create dangerous steering jumps.
- Vehicle stops at the final route point.

## Stage 4 — Perception and traffic-light sensing

### Goal

Validate the required perception pipeline independently from autonomous decisions.

### Entry criteria

- Stage 1 sensor contract passed.
- OctoMap runtime installed and launchable.
- Vehicle can be placed safely at selected spawn/test positions.

### Procedure: depth and occupancy

1. Visualize depth and CameraInfo.
2. Compare raw point cloud and preprocessed obstacle cloud.
3. Confirm ground removal does not remove NPCs or retain excessive road points.
4. Validate camera-to-world transforms while stationary and moving.
5. Start OctoMap and inspect 3D and projected maps.
6. Place an obstacle at known distances in front and right-front ROIs.
7. Compare measured distance topics with approximate Unity ground truth.

### Procedure: traffic lights

1. Test RGB and semantic topics independently.
2. Confirm RGB and semantic resolution, timestamp proximity, viewpoint alignment, and ROI.
3. At each of the four intersections, record red, yellow, green, and no-light cases.
4. Record raw score/debug output, stable state delay, false positives, and missed states.
5. Determine the correct route waypoint IDs for all four controlled intersections.
6. Repeat under approach motion, not only while stationary.

### Pass criteria

- Point cloud and OctoMap represent obstacles in correct frames and approximate positions.
- Front/merge ROI distances react to test obstacles without persistent roadside false alarms.
- Four relevant traffic lights are detected before the stopping decision must act.
- UNKNOWN is published when no relevant light is visible.
- Red cannot be incorrectly released by stale or unrelated green evidence.
- Verified traffic-light waypoint IDs and detector parameters are recorded.

## Stage 5 — Decision making and individual events

### Goal

Validate traffic rules, merge yielding, and emergency braking one behavior at a time.

### Entry criteria

- Stage 3 no-event route passed.
- Stage 4 perception passed.
- Four traffic-light waypoint IDs configured and verified.

### Part A: traffic lights

1. Enable traffic-light decision with NPC events disabled.
2. Test each intersection with red at approach.
3. Confirm complete stop, stop latch after route advancement, and no premature release.
4. Test continuously fresh green release.
5. Test yellow and UNKNOWN/stale input.
6. Run all four intersections in one route.

### Part B: merging event

1. Enable only the merge event.
2. Use an appropriate spawn position for repeated local tests, then repeat from spawn 0.
3. Verify right-front distance, slowdown latch, hysteresis, and release.
4. Test no-event roadside scenery for false slowdown.

### Part C: emergency event

1. Enable only the emergency-brake event.
2. Test at conservative approach speed.
3. Verify speed-dependent trigger distance, full stop, minimum NPC clearance, and release.
4. Interrupt obstacle data after detection and verify fail-safe latching.

### Pass criteria

- Complete red-light stop at all four intersections.
- Green is required and stable before release.
- Merge event avoids collision and resumes after adequate clearance.
- Emergency event avoids collision with a positive safety margin.
- Behavior priority is emergency > traffic stop > merge slowdown > planning speed.
- No behavior remains latched incorrectly after its documented release condition.

## Stage 6 — Full benchmark and repeatability

### Goal

Run the assignment configuration end to end and produce final evidence.

### Configuration

```text
spawnIndex: 0
eventEnable: [true, true]
all production nodes enabled
enable_motion: true
```

### Procedure

1. Perform a clean build from a fresh workspace and follow only repository documentation.
2. Start the complete system using one launch command.
3. Confirm the readiness checklist before enabling motion.
4. Run the full mission at least three times with rosbag recording.
5. For each run, record:
   - route completion and time;
   - road departures and collisions;
   - result at each traffic light;
   - merge and emergency minimum distance;
   - safe-stop reasons and stale inputs;
   - path fallback/rejection count;
   - command saturation and control error.
6. Reboot/restart the stack once and repeat to check startup determinism.
7. Generate plots for documentation: route trace, path tracking error, speed target versus
   measured speed, traffic-light state, obstacle distance, and command outputs.

### Pass criteria

- One launch file starts the documented complete system.
- Three consecutive runs finish without leaving the road or collision.
- All red lights produce complete stops and green permits progress.
- Both optional events are handled without collision.
- No unexplained node crash, topic loss, permanent latch, or manual intervention.
- Build and runtime behavior match the repository README.
- Final limitations and any non-passing requirement are documented honestly.

## Readiness checklist before any motion

```text
[ ] Correct Git commit and configuration recorded
[ ] Unity spawn and event configuration checked
[ ] /car_command has exactly one publisher
[ ] TF world -> OurCar/INS available
[ ] /OurCar/INS/twist fresh
[ ] /planning/path fresh, non-empty, frame=world, starts near ego
[ ] /decision/target_speed and /decision/stop_required fresh
[ ] Steering sign and command range verified for this Unity build
[ ] Maximum speed/throttle appropriate for current test stage
[ ] Rosbag recording started
[ ] Operator ready to stop Unity/controller
```

## Test log

Add concise entries in chronological order.

### Stage 0

- Date: 2026-07-16.
- Branch: `temp_inte`.
- Configuration verified: spawn position 0, both optional events disabled, and full-system
  controller motion disabled by default.
- Clean out-of-tree build: all seven functional packages plus `system_bringup` built and
  installed successfully under `/tmp/team20_stage0_install`.
- Installed-artifact checks: the route map, controller configuration, Unity configuration,
  and `full_system.launch.py` resolve from their package share directories.
- Launch argument check: `ros2 launch system_bringup full_system.launch.py --show-args`
  passed; `enable_motion` defaults to `false`.
- Unit tests: 40 trajectory/decision Python tests passed; the controller C++ test target
  passed (10 controller-core test cases).
- Safe headless smoke test:

  ```bash
  ros2 launch system_bringup full_system.launch.py \
    start_unity:=false \
    enable_motion:=false \
    enable_octomap:=false
  ```

  The launch started the simulation bridge, six static TF publishers, both available
  perception nodes, route manager, trajectory generator, both decision nodes, and the
  production controller. The route manager loaded all 60 waypoints. With Unity absent,
  data-dependent nodes waited for TF/sensor input as intended. A sampled `/car_command`
  contained `throttle: 0.0`, `steering: 0.0`, and `brake: 1.0`; no dummy controller was
  launched.
- Environment prerequisite: this machine does not currently have the OctoMap runtime
  installed. Installation needs an interactive sudo password and was therefore not
  performed automatically. Before Stage 4, run:

  ```bash
  sudo apt install \
    ros-jazzy-octomap-server \
    ros-jazzy-octomap-ros \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-rviz-plugins
  ```

  Then repeat the smoke test without `enable_octomap:=false`.
- Follow-up observations for Stage 1: when no Unity client is connected, the blocking TCP
  receiver may require launch to escalate from SIGINT/SIGTERM to SIGKILL; several existing
  Python nodes also report a duplicate `rclpy.shutdown()` error during Ctrl-C teardown.
  These do not affect startup or the safe-stop result, but reconnect and clean shutdown
  behavior must be evaluated with the real simulator in Stage 1.

### Stage 1

- Date: 2026-07-16.
- OctoMap prerequisite verified: all four Debian packages are installed, ROS resolves
  `octomap_server`, and `octomap_server_node` starts from the full-system launch.
- Corrected the Unity ego-state interface: Unity's wire name `OurCar/CoM` is now exposed
  through the canonical `/OurCar/INS/pose` and `/OurCar/INS/twist` ROS topics, matching TF,
  route, trajectory, decision, and controller consumers.
- Local Unity runs now bind the sensor TCP receiver to `127.0.0.1` by default, reducing
  exposure of the simulation binary stream. The receiver also rejects a connection whose
  first magic value is not Unity's `0xDEADC0DE`, closes it, and waits for the real Unity
  client. For a Unity instance on another machine, explicitly use
  `sensor_bind_host:=0.0.0.0` and configure networking securely.
- Measured steady-state topic rates on this machine:
  - pose/twist: approximately 29–33 Hz;
  - RGB left: approximately 2.3–2.6 Hz;
  - semantic image: approximately 1.8–2.1 Hz;
  - depth image: approximately 2.5 Hz.
- All four camera streams are 640x480. RGB and semantic images use `rgb8`; depth uses
  `16UC1` millimetres. One stationary depth frame contained nonzero distances from 1.4 m
  to 52.936 m with an 8.376 m median, consistent with the visible urban scene.
- Verified the complete transform chain from `world` through `OurCar/INS`, vehicle center,
  sensor base, depth camera, both RGB cameras, and semantic camera. At spawn 0 the initial
  pose was approximately `(-62.800, -12.293, 0.838)` with yaw 90 degrees.
- A persistent low-amplitude command probe established the Unity command convention:
  - 8% throttle produces forward motion;
  - positive brake reduces speed to zero and a 2-second full-brake hold does not reverse;
  - positive steering decreases ROS yaw and therefore turns right;
  - `controller.steering_sign` is consequently set to `-1.0` so a positive Pure Pursuit
    lateral target produces a left turn in Unity.
- The UDP transmitter repeats its last received command at 100 Hz. Stopping a temporary
  publisher does **not** clear the last throttle command. Manual tests must use one
  persistent publisher and explicitly stream a zero or brake command before exiting.
- The stack was restarted multiple times and Unity reconnected successfully each time.
  After guarding shutdown against an invalid ROS context, the TCP receiver, transmitter,
  static TF publishers, and Unity process all exit cleanly on Ctrl-C.
- `enable_motion:=false` was retained throughout normal integration launches. The only
  motion was the documented low-amplitude command probe with the production controller
  disabled; the vehicle was stopped before shutdown.

### Stage 2

- Date: 2026-07-16.
- Configuration: spawn 0, optional events disabled, route/perception/trajectory/decision
  nodes disabled, controlled paths published directly in frame `world`, and target speeds
  limited to 0.5–0.6 m/s.
- No controller algorithm or gain was replaced. The Stage 1 Pure Pursuit/PID parameters
  were retained because the measured response was stable. The only controller adjustment
  was increasing `twist_timeout` from 0.5 s to 1.0 s: Unity serializes all sensor streams
  over one TCP connection, and camera bursts occasionally delayed twist by slightly more
  than 0.5 s. A final regression contained no nuisance twist safe stops, while a sustained
  twist loss still produced full braking after approximately 1.03 s.
- A local health scanner connected to TCP port 9998 before Unity during one test. The TCP
  receiver previously remained attached to this invalid client. It now disconnects a
  non-Unity stream and returns to `accept()`, after which Unity connected and published
  state normally. This is a bridge robustness fix discovered by Stage 2, not a change to
  the controller interface.
- Controlled tracking results:

  | Test | Target | CTE RMS / max | Heading RMS / max | Other result |
  | --- | ---: | ---: | ---: | --- |
  | Straight final regression | 0.6 m/s | 0.069 / 0.124 m | 0.018 / 0.041 deg | rise 4.82 s, max 0.563 m/s, steady absolute speed error 0.044 m/s |
  | Gentle left, radius 25 m | 0.6 m/s | 0.084 / 0.129 m | 1.20 / 2.25 deg | max normalized steering 0.256, no sign oscillation |
  | Gentle right, radius 25 m | 0.6 m/s | 0.072 / 0.132 m | 0.81 / 1.56 deg | max normalized steering 0.229, no sign oscillation |
  | Straight, initial path heading +5 deg | 0.5 m/s | 0.070 / 0.125 m | 4.02 / 5.01 deg | correct smooth correction, no sign oscillation |
  | Straight, initial path heading -5 deg | 0.5 m/s | 0.070 / 0.126 m | 3.99 / 5.00 deg | correct smooth correction, no sign oscillation |

  The right-curve run began from the previous test endpoint and did not reach the requested
  speed; its lateral metrics remain valid, but longitudinal acceptance uses the independent
  straight regression. In every moving run, throttle and brake were never positive
  together. The final straight stop used full braking, reached below 0.05 m/s in 0.96 s,
  and travelled approximately 0.335 m after the stop request.
- Isolated fault injection used fresh synthetic path, decision, twist, and dynamic TF
  inputs, then removed one source at a time. Missing TF was verified in a fresh controller
  process so no prior transform remained buffered:

  | Injected condition | Full-brake detection latency |
  | --- | ---: |
  | stale path | 0.965 s |
  | stale target speed | 0.711 s |
  | stale stop state | 0.759 s |
  | stale twist | 1.029 s |
  | explicit stop request | 0.056 s |
  | missing TF | 0.001 s (first observed post-injection command) |

  All six cases output `throttle=0.0`, `steering=0.0`, `brake=1.0`; no sample contained
  simultaneous throttle and brake.
- Motion-disabled mock-path check: the vehicle moved less than 0.1 mm, throttle remained
  zero, and the controller continuously commanded full safe-stop braking.
- Out-of-tree rebuild of `simulation` and `controller` passed. `colcon test` reported
  11 tests, 0 errors, 0 failures, and 0 skipped, including all 10 controller-core GTests.
- Stage 2 conclusion: low-speed control and all required fail-safe paths pass. Stage 3 may
  begin at 0.5–1.0 m/s; do not raise the speed limit until route geometry and waypoint
  transitions have been validated with motion disabled.

### Stage 3 checkpoint (testing incomplete)

- Date: 2026-07-17.
- Branch: `temp_inte`.
- Scope: route manager, trajectory generator, and production controller with both optional
  events disabled. Traffic decisions were bypassed by a persistent low-speed test publisher;
  perception was disabled. Repository defaults remain spawn 0, all configured camera streams
  enabled, and controller motion disabled until explicitly requested.
- The motion-disabled route audit passed: `/route/local_path` and `/planning/path` used frame
  `world`, began within approximately 1 mm of ego at the initial transition, remained forward
  ordered, and produced no target-pose fallback after the route-window fix.
- Route progression now retains the previous waypoint in each local window, advances when a
  waypoint is reached or its forward progress plane is crossed, consumes duplicate/close
  points in one update, publishes `route_complete` with zero speed at the final point, and
  accepts an `initial_waypoint_index` diagnostic parameter. The benchmark default is still 0;
  nonzero indices are only valid with a matching local test spawn.
- Sparse route geometry now uses waypoint yaw as a bounded Hermite tangent constraint before
  the existing smoothing and validation pipeline. Sub-metre survey clusters are collapsed,
  endpoint projection preserves waypoint tangent yaw, and the allowed spline deviation is
  2.5 m. An offline audit of 119 route-window/ego combinations found no rejection except the
  natural single-point window at the final waypoint.
- The original waypoint 33–35 chord crossed a continuous guardrail. Camera inspection and
  repeatable collisions near `(127.2, 5.7)` isolated this as route geometry rather than a
  controller lookahead problem. Waypoint 34 now remains on the westbound approach and
  waypoint 35 enters the opening. The corrected turn passed both a spawn-5 local test and a
  continuous spawn-0 run using the unchanged Stage 2 controller parameters.
- Completed real-Unity coverage so far:
  - segmented low-speed runs covered waypoints 0–25 and 32–51, including all tight turns in
    that interval;
  - a continuous spawn-0 run at 0.8 m/s advanced from waypoint 0 to 52 without manual route
    index changes, travelled 603.088 m in the 1,200-second capture window, reported no invalid
    paths, no stuck/implausible-speed abort, and no simultaneous throttle and brake;
  - the duplicate 14/15 pair advanced in one update as intended;
  - measured maximum speed was 0.8641 m/s. Cross-track summaries from the temporary probe are
    conservative because its first-point metric includes expected projection distances and
    occasional stale-state intervals.
- Unity still intermittently pauses true-state delivery, even with camera output disabled in
  the temporary `/tmp` runtime copy. The controller responds with the previously verified
  stale-input full brake and resumes when state returns; these interruptions extended the
  route duration but did not cause a collision.
- **Checkpoint status:** Stage 3 is not yet passed. Testing stopped safely at target waypoint
  52. The next session must resume the unchanged simulation state if available, or repeat an
  appropriate continuous run, cover waypoints 52–59, observe `route_complete`, verify the
  final zero-speed publication and physical stop, then rerun build/tests before marking Stage
  3 complete. No Stage 4 work should start before that gate passes.
- Checkpoint verification: an out-of-tree build of the complete dependency closure through
  `system_bringup` finished successfully (7 packages). All 35 route-progress and trajectory
  functional tests passed, as did all 10 controller-core GTests. Installed route and trajectory
  configuration files matched the source files byte-for-byte, Python compilation and
  `git diff --check` passed. The legacy route-manager copyright/flake8/pep257 tests could not
  be collected on this machine because the corresponding `ament_copyright`, `ament_flake8`,
  and `ament_pep257` Python modules are not installed; no functional test failed.
