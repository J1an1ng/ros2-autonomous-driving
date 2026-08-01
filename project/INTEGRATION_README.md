# YOLO RGB-D integration and validation

This is the integration gate for the non-segmentation-camera solution. The complete
system must first be inspected with motion disabled, then validated by event, and only
then driven through the full route.

## Current release status: full-system-v1.9.0

The release implements the complete RGB-D YOLO autonomy chain, including route progress,
time-parameterized trajectory generation, traffic-light handling, adaptive following,
safe longitudinal decisions, and production vehicle control. The semantic camera is not
part of the production sensor contract.

Release acceptance is defined by the four gates below: static quality/build/test checks,
safe motion-disabled inspection, individual scenario checks, and a complete waypoint
0--63 production run. The final run must retain mission timing, state transitions,
trajectory constraints and availability, traffic-light data, and runtime logs.

Run-dependent numbers are intentionally excluded from this status description. The
latest retained run is presented only as an example in `v1.9.0 benchmark record`, and
its generated metrics and plots are stored in `mission_results/`.

## System contract

```text
Unity RGB + Depth + TF
          │
          ├─> YOLO RGB-D perception ─> traffic-light and vehicle topics ─┐
          ├─> occupancy/point cloud ──────────────────────────────────────┤
          └─> route -> planning -> decision -> controller -> /car_command │
                                                                           │
                       exactly one /car_command publisher <─────────────────┘
```

Production sensor requirements:

| Topic | Requirement |
| --- | --- |
| `/OurCar/Sensors/RGBCameraLeft/image_raw` | continuous colour image |
| `/OurCar/Sensors/RGBCameraLeft/camera_info` | valid intrinsics |
| `/OurCar/Sensors/DepthCamera/image_raw` | metric `16UC1`, `mono16`, or `32FC1` depth |
| `/OurCar/Sensors/DepthCamera/camera_info` | valid intrinsics |
| TF DepthCamera -> RGBCameraLeft | available |
| `/OurCar/INS/pose`, `/OurCar/INS/twist` | continuous |

The Unity segmentation-camera block remains present only as a disabled simulator
configuration entry (`enabled: false`). No production node may subscribe to it.

Trajectory planning additionally publishes Team 20's
`autonomy_interfaces/msg/PlannedTrajectory` on `/planning/trajectory`. Every point
contains pose, signed curvature, dynamically feasible target speed, and time from start.
The controller consumes this message directly and uses the speed attached to the same
point selected for Pure Pursuit. The scalar `/planning/target_speed` is selected from
the constrained profile for the decision layer; the controller caps its trajectory
sample with the resulting `/decision/target_speed`.
`full_system.launch.py` injects one shared `wheelbase` and
`maximum_steering_angle` into planner and controller. The controller rechecks every
received point against its own derived curvature cap and safe-stops on a mismatch.

## Gate 1: static checks

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

Pass criteria:

- the model is installed under `share/perception/models/yolov8n.onnx`;
- `yolo_rgbd_detector_node` starts and logs `Loaded YOLO ONNX model`;
- all functional tests pass;
- `ros2 interface show autonomy_interfaces/msg/PlannedTrajectory` succeeds;
- trajectory tests cover the steering-derived curvature cap, forward acceleration pass,
  backward braking pass, infeasible initial boundary, terminal stop, monotonic time, and
  ROS-message serialization;
- the simulation configuration reports the segmentation camera disabled;
- no production launch starts the old detector nodes.

## Gate 2: Unity with motion disabled

```bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=false \
  enable_visualization:=true
```

Inspect:

```bash
ros2 topic hz /OurCar/Sensors/RGBCameraLeft/image_raw
ros2 topic hz /OurCar/Sensors/DepthCamera/image_raw
ros2 topic echo /perception/front_vehicle_debug
ros2 topic echo /perception/traffic_light_debug
ros2 topic echo /perception/front_vehicle_distance
```

The RViz image `/perception/yolo/debug_image` must show correctly aligned boxes. For a
vehicle, compare its displayed RGB-D range with visible distance and
`/decision/front_obstacle_distance`. Test each controlled signal phase at waypoints 25,
32, 52 and 59.

Pass criteria:

- no systematic left/right depth offset inside boxes;
- no non-vehicle road structure is accepted as a front vehicle;
- visible event vehicles are detected over the required control distance;
- red, yellow and green states are stable and the selected box belongs to the current
  intersection;
- missing detections produce safe values rather than stale detections.

If this gate fails because boxes are missing or unstable, collect project images and
fine-tune as described in `src/perception/README.md`.

## Gate 3: individual low-speed scenarios

Keep only one `/car_command` publisher and enable motion. Validate independently:

1. one red-to-green stop and release;
2. the right-front merge event at waypoints 43--48;
3. the braking lead-vehicle event;
4. a no-event section to measure false positives.

Record at least:

```text
/OurCar/INS/pose
/OurCar/INS/twist
/route/current_waypoint_id
/route/current_waypoint_distance
/planning/path
/planning/trajectory
/planning/target_speed
/perception/front_vehicle_detected
/perception/front_vehicle_distance
/perception/front_vehicle_debug
/perception/traffic_light_state
/perception/traffic_light_scores
/perception/traffic_light_debug
/decision/front_obstacle_distance
/decision/merge_obstacle_distance
/decision/target_speed
/decision/state
/decision/debug
/car_command
/tf
/tf_static
```

No collision, road departure, stale-light release, or unintended full braking is allowed.

## Gate 4: full benchmark

After the three individual scenarios pass:

```bash
python3 tools/run_full_mission.py
```

Add `--visualization` when RViz is required. The runner records the audit topics and logs,
waits for `route_complete`, runs the route/trajectory gate, and atomically replaces
`mission_results/` only after success. Failed or interrupted runs retain their evidence
under `validation_evidence/` but do not overwrite the last valid report.

Run both no-event and two-event configurations from waypoint 0 through 63. Retain the
simulation config, perception/decision configs, exact ONNX model checksum, test date and
raw evidence with the result.

### Latest complete-test example: current `main`

The following measurements belong to one retained production run. They demonstrate that
the gates can be satisfied; they are not fixed thresholds or guaranteed outputs.

The 2026-08-01 production run used source commit
`40b1ecc6150752e022f67385f9812de72badbbb8` and:

```bash
python3 tools/run_full_mission.py --visualization
```

Observed results:

- All nine packages built, and `colcon test-result --verbose` reported 216 tests with
  0 errors, 0 failures, and 1 expected skip.
- Event1 entered `FOLLOWING_VEHICLE` and returned to `CRUISE` after the lead-vehicle
  constraint cleared. Measured speed stayed at or above 1.967 m/s and the decision target
  at or above 1.933 m/s, with no near-zero samples or stop-and-restart cycle.
- The Event2 vehicle reached a minimum measured front/merge distance of 7.502 m. The
  decision layer used `FOLLOWING_VEHICLE` around waypoints 43--47 and recovered without
  a collision; this run did not need `EMERGENCY_BRAKING` or `SLOWING_FOR_MERGE`.
- The controlled signals at waypoints 25, 32, 52, and 59 were observed; stop requests
  occurred at all four waypoints. The red phases at waypoints 32 and 59 reached
  `WAITING_FOR_GREEN` before release.
- The route manager advanced through all 64 waypoints and published route completion at
  waypoint 63; the decision and controller layers then maintained a safe stop.
- Mission time was **210.231 s**, defined from the first forward `/car_command`
  (`throttle > 0.01`, `brake < 0.01`) to the first `route_complete`. First measured
  motion (> 0.1 m/s) to completion was 210.051 s.
- The rosbag audit checked 70,008 points across 2,115 `/planning/trajectory` messages.
  Maximum curvature was 0.217622 1/m against the steering-derived 0.217704 1/m limit.
  Maximum lateral acceleration, longitudinal acceleration, and longitudinal
  deceleration were 3.0000, 2.0008, and 3.0000 m/s². The small serialization/chord
  excess is inside the documented 0.05 m/s² numerical tolerance. There were zero
  non-monotonic point times and zero invalid numeric points.
- Before completion, 2,096 / 2,103 trajectory publications were nonempty. The 0.33%
  empty fraction, 0.596 s longest empty interval, and 0.192 s maximum message gap passed
  the explicit 5% / 10 s / 1.05 s route-stage availability gates; regressions cover
  missing messages, single-empty recovery, trailing empty output, and silent topic loss.
- All 102 runtime local-path rejections were bicycle-model curvature-gate events. The
  straight target-pose fallback was revalidated by the same feasibility gate
  before publication; an invalid fallback causes an empty trajectory and safe stop.
- The bag recorded 1,386 `/perception/traffic_light_state` messages as
  `std_msgs/msg/UInt8`: 1,085 unknown, 194 red, 7 yellow, and 100 green.
- Startup-only missing-TF/path/velocity warnings and intermittent stale-depth diagnostics
  were handled by safe-stop behavior and recovered when fresh data arrived.
- Log inspection found no autonomy-node runtime `ERROR`, `FATAL`, process death,
  segmentation fault, traceback, collision, or route departure before route completion.
  RViz emitted one visualization-only GLSL shader error. The runner initiated teardown
  only after completion; 20 process-death entries were classified as teardown-only.

The machine-readable metrics, route trace, speed profile, tracking error, traffic-light
state, obstacle distance, and command plots are in
[`mission_results/`](mission_results/README.md). Every successful invocation of
`tools/run_full_mission.py` regenerates that directory from the just-completed mission.

The raw bag and matching ROS logs are retained at
[`Rosbag_reproduction/main-20260801-231823/`](Rosbag_reproduction/README.md). The MCAP file is
tracked with Git LFS and has SHA-256
`fdb33ded0eb9acdf3e44456431f34b26608f09bc922732658b72ed68c346d89f`.
After `git lfs pull`, a professor can reproduce the generated report from `project/` with:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/analyze_mission_run.py \
  --bag Rosbag_reproduction/main-20260801-231823/bag \
  --logs Rosbag_reproduction/main-20260801-231823/ros_logs \
  --trajectory-config src/trajectory_generator/config/trajectory_generator.yaml \
  --controller-config src/controller/config/controller.yaml \
  --output /tmp/team20-mission-results \
  --validation-date 2026-08-01
```

## Tuning order

Tune one layer at a time:

1. YOLO confidence and custom weights;
2. Depth/RGB TF and bbox sampling;
3. lateral/range gates;
4. vision/point-cloud match tolerance;
5. decision timing and vehicle control.

Changing downstream braking thresholds cannot compensate for missed or misaligned
detections. Always return to the motion-disabled gate after replacing the ONNX model.
