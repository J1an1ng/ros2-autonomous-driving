# YOLO RGB-D integration and validation

This is the integration gate for the non-segmentation-camera solution. The complete
system must first be inspected with motion disabled, then validated by event, and only
then driven through the full route.

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

## Tuning order

Tune one layer at a time:

1. YOLO confidence and custom weights;
2. Depth/RGB TF and bbox sampling;
3. lateral/range gates;
4. vision/point-cloud match tolerance;
5. decision timing and vehicle control.

Changing downstream braking thresholds cannot compensate for missed or misaligned
detections. Always return to the motion-disabled gate after replacing the ONNX model.
