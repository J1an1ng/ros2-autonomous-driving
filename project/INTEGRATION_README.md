# YOLO RGB-D integration and validation

This is the integration gate for the non-segmentation-camera solution. The complete
system must first be inspected with motion disabled, then validated by event, and only
then driven through the full route.

## Current release status: full-system-v1.9.0

The v1.9.0 release completed the production two-event full-route run required within the
final gate on 2026-08-01. The post-change run also recorded and audited Team 20's new
time-parameterized trajectory interface. A separate no-event run and retention of the
raw rosbag remain part of the complete optional Gate 4 evidence package. Persisted
metrics and plots are stored in `mission_results/`.

- the shared code-quality gate passed for 53 Python files and the configured C++ sources;
- all nine ROS 2 packages built successfully;
- the complete workspace suite passed with 184 tests, 0 errors, 0 failures, and one
  expected skip;
- Unity and the production ROS graph ran from waypoint 0 through waypoint 63 and reported
  `Route completed at waypoint 63`;
- adaptive lead-vehicle following, the right-side merge slowdown, and the four controlled
  traffic-light scenarios were exercised;
- no collision, route departure, runtime error, process death, or duplicate production
  publisher was observed;
- all ROS nodes exited cleanly after the completed route; Unity required forced
  termination only during the user-requested teardown.

The validation used the production RGB-D YOLO pipeline with the semantic camera disabled.
RViz was disabled for the final benchmark to reduce rendering overhead; this did not
disable Unity, perception, mapping, planning, decision making, or control.

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
The scalar `/planning/target_speed` used by the existing decision/controller interface
is selected from this constrained profile rather than independently commanded.

## Gate 1: static checks

```bash
source /opt/ros/jazzy/setup.bash
bash tools/check_code_quality.sh
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

Pass criteria:

- the model is installed under `share/perception/models/yolov8n.onnx`;
- `yolo_rgbd_detector_node` starts and logs `Loaded YOLO ONNX model`;
- all functional tests pass;
- `ros2 interface show autonomy_interfaces/msg/PlannedTrajectory` succeeds;
- trajectory tests cover the curvature cap, forward acceleration pass, backward braking
  pass, terminal stop, and monotonic time;
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
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=true \
  enable_visualization:=false
```

Run both no-event and two-event configurations from waypoint 0 through 63. Save the
simulation config, perception/decision configs, exact ONNX model checksum, test date and
rosbag with the result. Results produced by the former camera pipeline are historical
decision/controller evidence, not validation of this YOLO perception version.

### v1.9.0 benchmark record

The 2026-08-01 production run used:

```bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=true \
  enable_visualization:=false
```

Observed results:

- Event1 entered `FOLLOWING_VEHICLE` and returned to `CRUISE` after the lead-vehicle
  constraint cleared.
- The merge section entered `SLOWING_FOR_MERGE` around waypoints 47--48 and recovered
  without a collision.
- The controlled signals at waypoints 25, 32, 52, and 59 entered the applicable stopping
  state and released on confirmed green. The red phases at waypoints 32 and 59 reached
  `WAITING_FOR_GREEN` at effectively zero speed before release.
- The route manager advanced through all 64 waypoints and published route completion at
  waypoint 63; the decision and controller layers then maintained a safe stop.
- Mission time was **321.054 s**, defined from the first forward `/car_command`
  (`throttle > 0.01`, `brake < 0.01`) to the first `route_complete`. First measured
  motion (> 0.1 m/s) to completion was 319.116 s.
- The rosbag audit checked 147,717 points across 3,943 `/planning/trajectory` messages.
  Maximum lateral acceleration, longitudinal acceleration, and longitudinal
  deceleration were 3.0000, 2.0005, and 3.0000 m/s². The 0.0005 m/s² serialization/chord
  excess is inside the documented 0.05 m/s² numerical tolerance. There were zero
  non-monotonic point times and zero invalid numeric points.
- Startup-only missing-TF/path/velocity warnings and intermittent stale-depth diagnostics
  were handled by safe-stop behavior and recovered when fresh data arrived.
- Log inspection found no runtime `ERROR`, `FATAL`, process death, segmentation fault,
  traceback, collision, or route departure before route completion.

The machine-readable metrics, route trace, speed profile, tracking error, traffic-light
state, obstacle distance, and command plots are in
[`mission_results/`](mission_results/README.md). Regenerate them from a retained bag with
`tools/analyze_mission_run.py`.

## Tuning order

Tune one layer at a time:

1. YOLO confidence and custom weights;
2. Depth/RGB TF and bbox sampling;
3. lateral/range gates;
4. vision/point-cloud match tolerance;
5. decision timing and vehicle control.

Changing downstream braking thresholds cannot compensate for missed or misaligned
detections. Always return to the motion-disabled gate after replacing the ONNX model.
