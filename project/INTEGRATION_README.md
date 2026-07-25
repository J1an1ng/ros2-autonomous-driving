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
| 3 | Route, trajectory, and no-event closed loop | Passed | Full route and waypoint-52 regression; see test log |
| 4 | Perception pipeline and traffic-light sensing | Passed | Maps, ROI regression, four traffic lights, stale-frame guard; see test log |
| 5 | Decision making and individual events | Passed | Semantic/depth vehicle detection, point-cloud fusion, adaptive following and both NPC events validated |
| 6 | Full benchmark and repeatability | Passed | Historical three-run benchmark plus current waypoint 0--63 no-event and all-event regressions |

Status values are `Not started`, `In progress`, `Blocked`, and `Passed`. A later stage must
not be marked `Passed` while an earlier required stage is blocked.

The detailed Stage 6 section below preserves the 2026-07-17 evidence for the older
waypoint 0--59 route and its then-observed Event2 limitation. The current production result
is documented in `PROJECT_STATUS_SUMMARY.md`: on 2026-07-25 the waypoint 0--63 route passed
both the no-event baseline and the two-event run with semantic/point-cloud vehicle fusion.

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

### Stage 3 completion

- Date: 2026-07-17.
- Branch: `temp_inte`.
- Scope: route manager, trajectory generator, and production controller with both optional
  events disabled. Traffic decisions were bypassed by a persistent low-speed test publisher;
  perception was disabled. Repository defaults remain spawn 0, all configured camera streams
  enabled, and controller motion disabled until explicitly requested.
- The route was checked against assignment Figure 1 before the final regression. Waypoints
  45–59 follow the required upper-road westbound leg, the southbound turn, and the lower-road
  westbound leg to the marked goal; no alternate or diagnostic route was substituted.
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
- Completed real-Unity coverage:
  - segmented low-speed runs covered waypoints 0–25 and 32–59, including all tight turns in
    that interval;
  - a continuous spawn-0 run at 0.8 m/s advanced from waypoint 0 to 52 without manual route
    index changes, travelled 603.088 m in the 1,200-second capture window, reported no invalid
    paths, no stuck/implausible-speed abort, and no simultaneous throttle and brake;
  - the duplicate 14/15 pair advanced in one update as intended;
  - measured maximum speed was 0.8641 m/s. Cross-track summaries from the temporary probe are
    conservative because its first-point metric includes expected projection distances and
    occasional stale-state intervals.
- A focused spawn-5 run naturally entered waypoint 45 from waypoint 32 and reproduced the
  apparent waypoint-52 blockage at `(45.567, 38.812)`, within 0.6 m of the earlier spawn-0
  failure. The vehicle remained centred and aligned south; the controller commanded
  throttle 0.35, brake 0, and steering approximately -0.083. The actual cause was Unity
  true-state jitter: `/OurCar/INS/twist` had a measured maximum inter-message interval of
  1.072 s, just over the previous 1.0 s timeout, so repeated full safe-stop braking prevented
  the vehicle from restarting. There is no waypoint-52 obstacle and no route change was made.
- Raising only the runtime twist timeout allowed the vehicle to leave the identical stopped
  pose immediately. The source default is now 1.5 s, providing approximately 0.43 s margin
  over the observed gap while retaining bounded fail-safe behaviour for sustained state loss.
- A final regression used the source value of 1.5 s, spawn 5, initial waypoint 32, and one
  continuous Unity/route-manager session. It progressed through every waypoint 32–59,
  crossed the former failure position at 0.95–0.97 m/s, travelled 275.463 m, and reported a
  maximum speed of 0.9897 m/s. The probe's fixed 500-second window expired 5.211 m before
  completion, so a second capture continued without resetting Unity or route state.
  `route_complete` was then observed at waypoint 59, the published route speed was 0, and the
  vehicle stopped in 0.931 s at `(-47.236, 2.618)`. There were no invalid paths and no
  simultaneous throttle/brake commands.
- **Stage 3 status:** the required no-event route and terminal stop have passed. Stage 4 may
  begin.
- Checkpoint verification: an out-of-tree build of the complete dependency closure through
  `system_bringup` finished successfully (7 packages). All 35 route-progress and trajectory
  functional tests passed, as did all 10 controller-core GTests. Installed route and trajectory
  configuration files matched the source files byte-for-byte, Python compilation and
  `git diff --check` passed. The legacy route-manager copyright/flake8/pep257 tests could not
  be collected on this machine because the corresponding `ament_copyright`, `ament_flake8`,
  and `ament_pep257` Python modules are not installed; no functional test failed.
- Post-fix verification rebuilt `simulation` and `controller` out of tree, confirmed the
  installed controller configuration matches the source byte-for-byte, and passed all 10
  controller-core GTests. `git diff --check` also passed.

### Stage 4 completion

- Date: 2026-07-17.
- A fresh out-of-tree build of all eight workspace packages passed. The required Jazzy OctoMap
  packages and `octomap_server_node` were present.
- At stationary spawn 0, all camera streams were 640x480; RGB/semantic used `rgb8`, depth
  used `16UC1`, and all intrinsics reported `fx=fy=240`, `cx=320`, `cy=240`. Approximate
  measured rates were 2.4--2.5 Hz for depth/cloud/maps, 1.9 Hz for left RGB, 1.7 Hz for
  semantic, and 33--35 Hz for pose. Nearest RGB/semantic timestamp difference had a zero
  median and a 0.803 s worst sampled edge/missed-frame case.
- Raw depth cloud used the depth-camera frame; the ground-filtered cloud, 2D grids, and
  OctoMap used `world`. At spawn 0 the filtered cloud contained about 1,225 points. The
  depth projected map contained 418 occupied cells, its inflated map 3,755, and OctoMap's
  projected map 11; binary and full `OcTree` messages were continuous at 0.25 m resolution.
- The obstacle monitor originally requested the cloud image timestamp exactly, although the
  occupancy node transforms with the latest TF and retains the image timestamp. Serialized
  Unity sensors made that request several milliseconds newer than TF, so neither distance
  topic was published. `use_latest_tf=true` now makes both nodes use the same transform
  policy. A six-point synthetic cloud then reproduced front and right-front distances to
  better than `1e-4 m` (8.01561 m and 6.32456 m respectively), while points behind, above,
  or outside the ROIs were rejected.
- Dynamic captures covered waypoints 1--59 and `route_complete`. The preprocessed cloud
  stayed in `world`; individual runs contained 25--746 continuous cloud/map samples and
  demonstrated that the local map origin followed the moving vehicle. The vehicle completed
  the final approach at approximately `(-46.55, 2.61)` without a perception-node or TF exit.
- Initial ROI runs found two classes of fixed roadside return: curb points near
  `z=-0.57..-0.67 m` and vertical structures near `y=-2.8..-3.2 m`. The default height was
  tightened from -1.0 to -0.5 m and the right-front corridor from `y=-4.0..-0.8 m` to
  `y=-2.5..-0.8 m`. The known-obstacle regression still reproduced 8.01561 m front and
  6.32456 m merge distances to better than `1e-4 m`. On the straight waypoint-32 approach,
  the nearest merge return was then 10.33 m, above the 10 m trigger; the waypoint-52 light
  crossing produced only infinite front/merge distances.
- RGB and semantic images were saved in pairs every four seconds along the route. Inspection
  proved that route labels `intersection` at 9/16/38 do not identify the assignment's four
  controlled lights; those locations contain ordinary road or a T turn and correctly produce
  UNKNOWN. The five `stop_zone` entries are the initial spawn plus four controlled lights, so
  the verified allowlist is `25, 32, 52, 59`.
- Stable traffic-light evidence under motion:

  | Waypoint | Representative stable evidence before/at the light | Result |
  | --- | --- | --- |
  | 25 | GREEN near `(226.1,28.5)`; RED near `(226.8,16.0)`; stationary YELLOW score 60 | RED/YELLOW/GREEN passed |
  | 32 | GREEN near `(155.5,4.3)`; YELLOW near `(146.8,4.2)`; RED near `(141.6,4.2)` | RED/YELLOW/GREEN passed |
  | 52 | RED near `(46.3,24.7)`; GREEN near `(46.4,22.4)`; YELLOW near `(46.6,15.2)` | RED/YELLOW/GREEN passed |
  | 59 | GREEN near `x=-31.4`; RED near `x=-40.4`; YELLOW at the final approach | RED/YELLOW/GREEN passed |

- Lights on adjacent/departing roads were also seen near non-controlled waypoints and then
  returned to UNKNOWN after leaving view. Configuring the four-ID decision allowlist prevents
  that unrelated evidence from affecting control. RGB/semantic nearest-stamp medians were
  usually zero; critical light runs had sampled maxima of 0.414--0.933 s, while missed frames
  outside critical approaches occasionally exceeded one second. The detector now rejects a
  semantic image whose timestamp differs from RGB by more than 1.0 s and publishes UNKNOWN.
  A synthetic runtime regression observed both `semantic_gate=stale` for a 2.0 s difference
  and `semantic_gate=enabled` for aligned frames; all 34 emitted states were UNKNOWN as
  required.
- The final out-of-tree build installed all eight workspace packages successfully. The final
  ROS test summary reported 24 tests with zero errors, failures, or skips; this included all
  13 decision-state-machine cases and all 10 controller-core GTests. The semantic freshness
  runtime probe, decision Python compilation, and `git diff --check` also passed, and the final
  allowlist is installed from the decision YAML.
- **Stage 4 status:** perception, four traffic-light IDs, and stale-frame safety have passed.
  Stage 5 must still validate behavior with decisions enabled. In particular, the straight
  vehicle-frame front ROI sees real fixed geometry at some T turns, and point-cloud proximity
  cannot identify object class or intent. Emergency/merge activation therefore requires
  event/route-context validation before the full benchmark; do not weaken emergency distance
  merely to hide those geometrically valid returns.

### Stage 5 partial validation and waypoint48 blocker

- Date: 2026-07-17.
- The traffic-light decision now associates colours only within 20 m, remembers an
  associated red/yellow, approaches at at most 1 m/s, and brakes at a 4 m route boundary.
  The boundary remains outside the route manager's 3 m waypoint-advance radius.
- A no-event run from waypoint27 stopped at waypoint32, completed the required stop, released
  on stable green, and reached waypoint38. The final stop was approximately 3.6 m from the
  controlled waypoint instead of the earlier approximately 15.9 m recognition distance.
- The first waypoint32 T-junction correction rounded the reference and limited waypoints
  33--35 to 3 m/s. It removed the original late-turn guardrail overrun and reached waypoint38
  without `EMERGENCY_BRAKING`, but later visual inspection showed that its northward motion
  began too early and cut the inner kerb. That geometry is retained as a known failure bound,
  not as the accepted result.
- The related Python suite passed all 66 functional tests, the three affected ROS packages
  built successfully out of tree, and `git diff --check` passed.
- The first longer no-event continuation was intentionally stopped at waypoint48. With both
  Unity NPC events disabled, fixed turn geometry entered both rectangular vehicle-frame ROIs:
  merge distance fell from 3.21 m to 2.99 m and repeatedly selected
  `SLOWING_FOR_MERGE`; front distance then fell from 4.54 m to 3.97 m and latched
  `EMERGENCY_BRAKING`. The stationary vehicle remained approximately at `(66.21, 53.23)`
  with waypoint48 still about 17.1 m away.
- This reproduced the Stage 4 warning: a straight vehicle-frame proximity rectangle cannot
  distinguish the guardrail/road-end structure of a turn from an NPC occupying the planned
  lane. The obstacle monitor now projects cloud points onto `/planning/path` and reports
  along-path distance only inside path-relative front and right-side corridors. A stale or
  unavailable path retains the old rectangular filter as a conservative fallback. The
  obstacle timeout is 1.0 s, covering the measured approximately 2.5 Hz depth rate.
- The same waypoint27 run with the path-relative filter passed the old waypoint48 deadlock,
  reached waypoint50, continued through waypoint52 and waypoint59, and entered
  `route_complete` at approximately `(-48.76, 2.62)`. One short edge-point emergency near
  waypoint48 released automatically as the planned path curved away; it no longer stranded
  the vehicle. There were no simultaneous throttle/brake commands.
- Waypoint52 and waypoint59 both received stable green approach authorization in this run;
  explicit-red stop cases at those two lights still need isolated phase-aligned tests.
- Spawn survey results for local event tests are now documented by measurement: spawn1 is
  approximately `(-49.0, 52.4)`, spawn2 `(70.31, 52.73)`, spawn3 `(141.6, 52.9)`, spawn4
  `(229.1, 15.1)`, and spawn5 `(141.37, 3.29)`. Event 1 enabled from spawn2 produced no NPC
  obstacle through waypoints13--28, so that interval is not its trigger region.
- The path-corridor and existing route/decision/trajectory suites pass 70 functional tests,
  the affected decision package builds successfully out of tree, and `git diff --check`
  passes. Stage 5 remains in progress for event-location gating and the two isolated-red
  signal cases; the emergency distance was not weakened.

### Stage 5 waypoint32 intermediate-turn regression

- Date: 2026-07-17.
- The original route kept travelling west until approximately `x=124 m` before turning and
  caused a late-turn guardrail risk. The first correction was already at approximately
  `(129.37, 4.41)` while turning and was observed to cut the inner kerb. These two measured
  trajectories define the local failure bounds.
- Waypoint34 is now `(127.000, 3.700)` with a 150 degree tangent, and waypoint35 returns to
  the northbound centre reference at `(123.747, 8.864)`. Speeds remain 3 m/s; global
  pure-pursuit parameters are unchanged.
- A spawn5, no-event run from waypoint32 stopped for the signal, released on stable green,
  reached waypoint38, and never entered `EMERGENCY_BRAKING`. There were no simultaneous
  throttle/brake commands; minimum reported front clearance was 20.16 m.
- The measured intermediate trajectory passed `(128.59, 4.04)`, `(127.51, 4.44)`, and
  `(124.11, 7.45)` before settling near `x=124.2 m` in the northbound lane. It therefore
  delays the turn relative to the inner-kerb failure while retaining substantially more
  steering preview than the original guardrail failure.
- All 70 route, trajectory, and decision functional tests pass, the affected route and
  trajectory packages build in an isolated overlay, and `git diff --check` passes. The
  temporary Unity configuration was restored to spawn3 with both NPC events disabled after
  this focused regression.

### Stage 5 event-intersection and waypoint52 approach regression

- Date: 2026-07-17.
- The first event-intersection failure was a route-progress and geometry issue rather than an
  NPC stop. Waypoints40 and 41 had been closer than the 3 m waypoint-reach radius, so both
  turn constraints could be advanced in one timer cycle. Waypoints39--42 now start the left
  turn earlier, remain more than 3 m apart, and use a 2 m/s limit through the corner.
- A spawn5 no-event closed-loop run followed `(123.78,43.87)`, `(122.77,46.86)`,
  `(120.43,49.78)`, and `(116.37,51.83)`, then settled in the westbound lane near
  `y=52.54` and reached waypoint44 without emergency braking or simultaneous throttle and
  brake.
- The repeatable stop near `(65.4,53.3)` before waypoint52 was caused by fixed road-end
  geometry entering the obstacle corridor; it was not evidence that the distant visible NPC
  had braked. Waypoints46--48 now keep the generated reference south of the former spline
  bulge while retaining the later, wider corner entry needed to avoid the inner kerb.
- The first overly early road-end correction was rejected after it physically stuck at
  `(48.73,49.10)` with `CRUISE`, 0.35 throttle, zero brake, and an 8.57 m front clearance.
  The accepted correction passed that point, reached waypoint51 near `(45.44,45.24)`, was
  released through waypoint52, and reached waypoint54. The completion probe reported no
  emergency braking, no simultaneous throttle/brake, and a minimum 11.86 m front clearance.
- Both optional NPC events were disabled for these fault-isolation runs. The five affected
  functional test files pass 72 tests, the route and trajectory packages build in the
  isolated overlay, and `git diff --check` passes. NPC Event2 validation can now resume from
  this no-event baseline.

### Stage 5 NPC Event1 validation

- Date: 2026-07-17.
- Configuration was verified before the accepted run: benchmark spawn0, Event1 enabled,
  Event2 disabled, route initial index 0. A discarded setup attempt used a stale temporary
  spawn6 configuration and produced no Event1 evidence; it is not part of the result.
- The RGB contact sequence visibly records the red NPC entering and merging after the first
  intersection. Unity independently logged `OurCar hit the collider Trigger Collider Event
  1` and `Triggered event collider Trigger Collider Event 1`, confirming that the rendered
  vehicle and the enabled event were active rather than static scenery.
- From waypoints11--13, the measured right-front merge distance decreased from 15.18 m to a
  minimum 3.13 m. The decision layer selected `SLOWING_FOR_MERGE`, limited commanded speed to
  2 m/s, yielded while the NPC occupied the lane, and returned to `CRUISE` after clearance.
- The ego vehicle did not enter `EMERGENCY_BRAKING`, retained a minimum 15.14 m reported
  front-path clearance, produced no simultaneous throttle/brake command, and resumed through
  waypoint17. This passes the Event1 generation, rendering, trigger, yielding, clearance and
  release checks.
- A separate short `SLOWING_FOR_MERGE` activation occurred at waypoints5--7 before the Event1
  trigger and released by itself. It is consistent with the documented lack of route-segment
  gating for the merge ROI and should be treated as a nuisance false slowdown, not Event1
  evidence; it did not prevent completion of the focused run.
- Unity was stopped after the focused run. The temporary simulator configuration was restored
  to spawn0 with both optional events disabled. Event2 was not enabled or tested.

### Stage 5 NPC Event1 less-conservative tuning

- Date: 2026-07-17.
- The Event1 merge trigger was reduced from 10 m to 8 m, and the merge target speed was
  increased from 2.0 m/s to 2.5 m/s. Normal trajectory and controller speed ceilings were
  increased from 8 m/s to 10 m/s (36 km/h); all lower route, intersection, curve, signal and
  emergency limits retain priority.
- A repeated spawn0 run with Event1 enabled and Event2 disabled first entered
  `SLOWING_FOR_MERGE` at a measured 7.63 m instead of the former approximately 9.85 m. It
  reached a minimum 2.42 m right-front merge distance, did not enter
  `EMERGENCY_BRAKING`, produced no simultaneous throttle/brake command, cleared the NPC and
  reached waypoint17.
- Normal-route output visibly used the new 10 m/s command ceiling. Actual speed on this short
  section remained near 3.1 m/s because of available acceleration, the 2 m/s first-intersection
  limit and serialized simulator safe-stop interruptions; the ceiling itself was no longer the
  constraint.
- Static geometry at waypoints5--8 still caused a short self-releasing merge slowdown. A
  waypoint allowlist was considered but not retained because the environment could not provide
  the required final closed-loop rerun. Route-gating remains a documented follow-up rather than
  unvalidated production behavior.
- The five affected functional suites pass 72 tests, all three modified packages build in an
  isolated overlay, and `git diff --check` passes. The simulator configuration was restored to
  spawn0 with both events disabled; Event2 was not tested.

### Stage 5 NPC Event2 focused validation

- Date: 2026-07-17.
- The focused setup used benchmark spawn5, route initial index32, Event1 disabled, and Event2
  enabled. Unity logged `OurCar hit the collider Trigger Collider Event 2` and `Triggered
  event collider Trigger Collider Event 2`, so the event trigger itself was active.
- The accepted probe began at waypoint45 and reached waypoint54, after which the vehicle
  continued through waypoint56. Around waypoints47--48 the right-front distance decreased
  from 7.59 m to a minimum 2.39 m. The decision layer selected `SLOWING_FOR_MERGE`, limited
  the target to 2.5 m/s, returned to `CRUISE` after the return cleared, and produced no
  simultaneous throttle/brake command.
- No finite front-path obstacle distance and no `EMERGENCY_BRAKING` state were recorded. The
  79-frame forward RGB sequence also did not resolve an NPC in the ego lane. Therefore this
  run confirms event activation and a transient right-side return, but it does **not** pass
  the Stage 5 emergency-event criteria of front-hazard detection, full stop, positive NPC
  clearance, and post-hazard release.
- The old waypoint48 static-barrier deadlock did not recur. The only complete stop occurred
  at waypoint52 for a red traffic light, approximately 3.83 m from the waypoint. A stable
  green released the stop, proving that the former waypoint52 stop was not caused by Event2.
- Before modifying production thresholds, the next diagnostic must localize the Event2 actor
  in synchronized RGB/semantic/depth frames and compare its points with the path-relative
  front and merge corridors. Likely branches are event timing/trajectory mismatch, the actor
  remaining outside the 1.2 m front corridor on the bend, or merge behavior yielding early
  enough that the actor clears before becoming a front hazard. A controlled comparison with
  merge slowdown disabled only for diagnosis can distinguish the last case; it must not be
  retained as the final fix.
- Testing was paused at this functional blocker without changing controller, decision, route,
  trajectory, or perception code. The temporary simulator configuration was restored to
  spawn0 with both events disabled.

### Stage 5 waypoint52 green-release regression

- Date: 2026-07-17.
- The full-route run stopped correctly for red at waypoint52 but did not release when the
  signal appeared green in Unity. Decision logs proved that the stop latch behaved correctly:
  it continued receiving only `RED`, `YELLOW`, or stale `UNKNOWN`, never stable `GREEN`.
- The selected semantic component was the same centre component used by an earlier successful
  waypoint52 run. The difference was colour competition inside that component. During the
  failed green phase the weighted green score was 40--44 while small yellow
  housing/reflection contributions scored 32--39. The former
  `yellow_over_green_ratio=0.75` therefore classified the visible green as yellow.
- `yellow_over_green_ratio` is now 0.90. This is a local perception-threshold correction: it
  does not change semantic component selection, red priority, the four-frame stability guard,
  or the decision-layer stop and release conditions. A true yellow phase with no green bulb
  evidence remains yellow.
- A spawn5, no-event closed-loop rerun from waypoint32 reached waypoint52 on red, entered
  `STOPPING_FOR_LIGHT` at approximately 3.77 m, fully stopped in `WAITING_FOR_GREEN`, and
  stayed latched. When the signal changed, four fresh frames with green score 44 and yellow
  noise score 38--39 produced stable `GREEN`; the decision entered `RELEASED`, crossed
  waypoint52, and continued through waypoint54 without simultaneous throttle and brake.
- The perception package builds successfully in an isolated overlay. The separate decision
  probe could not create its ROS log file under the restricted test shell, so the result is
  based on the complete node transition and controller command logs from the launch session.
  The temporary Unity configuration was restored to spawn0 with both events disabled.

### Stage 5 waypoint25 green-release regression

- Date: 2026-07-17.
- In the subsequent full-route run, the first controlled intersection stopped correctly for
  red but remained stopped after the signal visibly turned green. This is not the waypoint52
  colour-threshold regression: at waypoint25 the detector changed from the lower forward-facing
  signal to a large unlit overhead/back-facing semantic component as the vehicle moved close to
  the stop line. The resulting raw and stable state was `UNKNOWN`, so the decision latch
  correctly refused to release.
- The `strict_center` component selector now measures two-dimensional distance to the expected
  forward-signal position: horizontal image centre and `strict_center_y_ratio=0.62` of the
  traffic-light ROI height. Previously it compared horizontal position only. This keeps the
  lower forward-facing component selected without changing colour thresholds, stop distance,
  decision-state transitions, or controller behavior.
- Focused spawn3 runs from waypoint20 consistently selected the forward component down to the
  closest useful views (component centre approximately y=176--190), produced stable `GREEN`,
  entered `RELEASED`, and crossed waypoint25. The exact red-stop-to-green transition is retained
  as a checkpoint in the next full-route run because the focused simulator launches reached
  the intersection during its deterministic green phase.
- The all-event full-route rerun also reached waypoint25 during green. It retained component 2
  through the close approach, entered `RELEASED` at approximately 9.83 m, and crossed waypoint25.
  No `UNKNOWN` latch or unintended stop recurred. The run subsequently passed the red-stop/green-
  release cycle and right turn at waypoint32, then both NPC areas.
- The same run was paused at waypoint52. It stopped correctly for red at approximately 3.97 m,
  but the next visually green phase scored green 44 and yellow reflection 45 for the correct
  semantic component. The current `yellow_over_green_ratio=0.90` therefore still emitted stable
  `YELLOW`. This is a recurrence of the waypoint52 colour-competition issue, not the waypoint25
  component-selection issue. A minimal follow-up is to require yellow to exceed concurrent green
  evidence (for example ratio 1.05); true-yellow frames observed in this run later had green score
  zero and would remain unaffected. Full-route testing stopped at this blocker.
- Follow-up tuning intentionally targets the deterministic simulator rather than variable real-
  world illumination. Red and green now use broader hue/saturation/value gates; yellow is limited
  to hue 42--58 degrees, higher saturation/value, at least 10 accepted pixels and score 20.
  Yellow must score at least 1.25 times concurrent green to override it. A focused waypoint52 run
  confirmed that the former green phase no longer generated yellow evidence, but its broadened red
  gate retained housing reflections (red score 44 versus green 40). Therefore green now overrides
  concurrent red reflection once it reaches 0.80 of the red score. Pure red frames in the same run
  had green score zero, preserving the stop. The perception package rebuilt successfully; the next
  all-event full-route run is the closed-loop validation checkpoint.

### Stage 5 aggressive traffic-colour full-route validation

- Date: 2026-07-17.
- The accepted run used benchmark spawn0, route index0, both optional NPC events enabled, and
  the deterministic-simulator colour profile described above. The vehicle traversed all 60
  waypoints and the route manager entered `route_complete` at waypoint59.
- Waypoint25 retained the intended forward-facing semantic component, classified green, entered
  `RELEASED` at approximately 11.37 m, and crossed the first controlled intersection without an
  UNKNOWN-state latch.
- Waypoint32 approached at 1 m/s, latched the red stop at approximately 3.70 m, entered
  `WAITING_FOR_GREEN`, and changed to `RELEASED` approximately 1.15 s later when stable green
  arrived. The vehicle then completed the validated intermediate right-turn geometry and reached
  waypoint38 without emergency braking.
- Event1 and Event2 were both present in the full run. Each produced finite right-front merge
  evidence and `SLOWING_FOR_MERGE`, then released back to cruise. Event2 still did not produce the
  task-required NPC emergency-brake/front-hazard sequence; this remains the previously documented
  scenario-definition blocker and is not counted as a newly passed emergency-event test.
- Waypoint52 stopped for red at approximately 3.96 m and remained in `WAITING_FOR_GREEN`. On the
  green phase, the narrow yellow gate produced no competing yellow evidence; green evidence was
  allowed to dominate the residual red housing reflection. The state machine entered `RELEASED`
  with approximately 3.86 m remaining, crossed waypoint52, and continued normally.
- Waypoint59 independently repeated the same safety cycle: red stop at approximately 3.77 m,
  stable-green release, and final `route_complete` with approximately 2.79 m remaining. This
  confirms that the aggressive green/red competition rule did not remove genuine red stops.
- No `EMERGENCY_BRAKING` transition and no simultaneous non-zero throttle/brake command appeared
  anywhere in the accepted full-route logs.
- The perception package built successfully in `/tmp/team20_aggressive_tl_install`. The complete
  launch was stopped only after `route_complete`; the Python decision node's known double-shutdown
  `RCLError` occurred after Ctrl-C and is teardown-only, not a runtime test failure. The temporary
  simulator configuration was restored to spawn0 with both optional events disabled.

### Stage 5 acceptance decision

- Date: 2026-07-17.
- Stage 5 is accepted as complete for integration. The project owner explicitly waived the
  remaining focused Event2 and final-profile waypoint25 red-phase tests after reviewing the
  successful all-event full-route run.
- Event2 generation, rendering, trigger, merge slowdown, clearance, and route continuation are
  verified. Its Unity NPC does not execute the planned brake-in-lane maneuver, so no corresponding
  ego `EMERGENCY_BRAKING` transition can be claimed. This remains a documented scenario limitation
  and must be reported in Stage 6 evidence rather than represented as a passed emergency-brake test.

### Stage 6 completion

- Date: 2026-07-17.
- Branch: `temp_inte`; benchmark baseline commit: `bfc5c00` (`Complete Stage 5
  integration validation`). The three benchmark runs used that identical functional
  configuration: spawn0, route index0, both Unity NPC events enabled, all production nodes,
  and `enable_motion=true`.
- A fresh out-of-tree build installed all seven functional packages successfully in
  `/tmp/team20_stage6_bfc5c00`. All 72 decision/route/trajectory Python functional tests and
  all 10 controller-core GTests passed. The only clean-workspace test failure was the
  pre-existing route-manager flake8
  test, which reports 260 formatting/import-order findings in existing Python files; no
  functional test failed and Stage 6 did not mass-reformat unrelated code.
- The pre-motion readiness gate passed with exactly one `/car_command` publisher, valid
  `world -> OurCar/INS` TF, a non-empty `world` planning path, fresh twist and decision
  inputs, and full braking while motion was disabled. Approximate healthy rates were
  24--25 Hz twist, 9.9 Hz planning path, and 19.6 Hz decision output.
- Three consecutive full missions were recorded without manual intervention. Each traversed
  waypoints 0--59 and entered `route_complete`; no collision or road departure was observed,
  no runtime node died, no path rejection/fallback occurred, and no command contained
  simultaneous non-zero throttle and brake.

  | Run | Mission time | Max / mean speed | Tracking RMSE / p95 | Event1 min merge | Event2 min merge | Result |
  | --- | ---: | ---: | ---: | ---: | ---: | --- |
  | 1 | 648.845 s | 3.155 / 1.697 m/s | 0.7563 / 2.0004 m | 2.437 m | 2.444 m | Complete |
  | 2 | 651.614 s | 3.162 / 1.752 m/s | 0.7775 / 2.1516 m | 2.377 m | 2.443 m | Complete |
  | 3 | 651.287 s | 3.153 / 1.625 m/s | 0.7440 / 2.0827 m | 2.438 m | 2.394 m | Complete |

- All phase-aligned red signals produced a complete stop and stable green released the
  vehicle. Recorded stop distances were 3.69--3.97 m at waypoint32, 3.74--3.92 m at
  waypoint52, and 3.70--3.83 m at waypoint59. Waypoint25 was green on these deterministic
  runs and released on approach. No permanent traffic-light latch recurred.
- Both NPC regions produced finite right-front evidence, selected `SLOWING_FOR_MERGE`, and
  released so the route could continue. Short nuisance merge slowdowns from static roadside
  geometry also occurred (15--20 state entries per run) but all self-released.
- Event2 retains the Stage 5 scenario limitation: all three bags have no finite front-path
  obstacle at waypoints45--49 and no `EMERGENCY_BRAKING` entry. The Unity actor therefore did
  not execute the assignment's intended brake-in-lane maneuver. Stage 6 verifies collision-
  free handling of the event behavior actually generated, but does not claim a passed NPC
  emergency-brake test.
- Serialized Unity input occasionally produced long twist gaps (12.11--14.20 s maxima,
  principally startup or sensor-stream pauses). The controller entered its documented safe
  stop and recovered automatically when data resumed. Run2 also had an approximately 17 s
  startup TF delay and recovered without intervention; planning-path and decision-output
  maximum gaps during the recorded missions remained below 0.226 s and 0.097 s.
- The raw evidence remains outside Git because the three MCAP bags total approximately
  119 MiB: `/tmp/team20_stage6_evidence/run{1,2,3}/bag`, with matching ROS logs in each run
  directory. Reproducible summary data and six SVG plots are committed under
  `project/stage6_results/`; `project/tools/analyze_stage6_bags.py` regenerates them.
- The three recorded runs exposed four Python `process has died` messages only after
  `route_complete` during teardown. The affected entry points now tolerate Jazzy external
  shutdown/context invalidation and use `rclpy.try_shutdown()`. A fresh full-stack restart
  confirmed all four remain alive, receive simulation data, and later report
  `process has finished cleanly`, with no traceback or `process has died`. The complete stack
  still takes longer than launch's five-second SIGINT grace period and is then terminated
  cleanly by SIGTERM; this shutdown latency affects Unity, TF and C++ nodes too and remains a
  non-functional cleanup limitation.
- This stage used an extracted official Jazzy rosbag2 overlay in `/tmp/rosbag_overlay`
  because rosbag2 was absent from the system installation and sudo was unavailable. For a
  normal machine setup, install `ros-jazzy-rosbag2` before reproducing the recordings.
- **Stage 6 status:** passed with the explicit Event2 scenario and shutdown-latency
  limitations above. The assignment route, traffic-light behavior, collision-free
  repeatability, single-launch startup, controller safety, and evidence-generation goals
  are demonstrated in three consecutive end-to-end missions.
