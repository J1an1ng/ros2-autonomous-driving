# Perception: YOLO RGB-D pipeline

This package implements the assignment without using the Unity semantic
camera. One YOLOv8 inference on the left RGB image detects vehicles and
traffic lights. Depth-camera pixels are reconstructed in 3-D, transformed
through TF into the left-camera frame, and projected into the YOLO boxes.

```text
RGB left ──> YOLOv8 ONNX ──> vehicle box ─┐
                                          ├─> robust RGB-D range ─> decision
Depth + CameraInfo + TF ─> RGB projection ┘

RGB left ──> YOLO traffic-light box ─> HSV bulb colour ─> temporal filter
```

The semantic camera is disabled in
`simulation_config.json`; it is neither subscribed to nor launched.

## Runtime

The package includes a fixed-size COCO YOLOv8n model and the CPU version of
Microsoft ONNX Runtime 1.22.0. A Python or Ultralytics installation is not
needed at runtime. OpenCV handles letterboxing, NMS, HSV classification and
debug drawing.

Build and start the perception package:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select perception
source install/setup.bash
ros2 launch perception perception.launch.py
```

Use another compatible model without changing the source:

```bash
ros2 launch perception perception.launch.py \
  yolo_model:=/absolute/path/to/best.onnx
```

The launch file starts:

- `occupancy_map_node`, which converts depth to the existing point-cloud and
  occupancy topics;
- `octomap_server_node`;
- `yolo_rgbd_detector_node`, which replaces both former semantic vehicle
  detection and the former traffic-light detector.

## Topics

| Direction | Topic | Type | Meaning |
| --- | --- | --- | --- |
| input | `/OurCar/Sensors/RGBCameraLeft/image_raw` | `sensor_msgs/Image` | YOLO/colour image |
| input | `/OurCar/Sensors/RGBCameraLeft/camera_info` | `sensor_msgs/CameraInfo` | RGB intrinsics |
| input | `/OurCar/Sensors/DepthCamera/image_raw` | `sensor_msgs/Image` | Metric depth |
| input | `/OurCar/Sensors/DepthCamera/camera_info` | `sensor_msgs/CameraInfo` | Depth intrinsics |
| output | `/perception/front_vehicle_detected` | `std_msgs/Bool` | Accepted vehicle box with valid range |
| output | `/perception/front_vehicle_distance` | `std_msgs/Float32` | 20th-percentile box depth in metres, otherwise `inf` |
| output | `/perception/traffic_light_state` | `std_msgs/UInt8` | `0 UNKNOWN`, `1 RED`, `2 YELLOW`, `3 GREEN` |
| output | `/perception/traffic_light_scores` | `std_msgs/Int32MultiArray` | Decision evidence `[red, green]`; yellow remains in the debug string |
| output | `/perception/yolo/debug_image` | `sensor_msgs/Image` | Boxes, class IDs, confidence and range |
| output | `/perception/front_vehicle_debug` | `std_msgs/String` | Vehicle rejection/measurement details |
| output | `/perception/traffic_light_debug` | `std_msgs/String` | Light selection and colour details |

`occupancy_map_node` publishes the depth-derived point-cloud and 2-D mapping interfaces:

| Direction | Topic | Type | Meaning |
| --- | --- | --- | --- |
| output | `/perception/pcl/points` | `sensor_msgs/PointCloud2` | Raw valid depth points transformed to `world`. |
| output | `/perception/pcl/points_preprocessed` | `sensor_msgs/PointCloud2` | Height-filtered obstacle points used by `obstacle_monitor_node`. |
| output | `/perception/depth/projected_map` | `nav_msgs/OccupancyGrid` | Direct 2-D projection of depth-derived obstacle cells. |
| output | `/perception/extended_map` | `nav_msgs/OccupancyGrid` | Inflated 2-D obstacle map for visualization. |

When `enable_octomap:=true`, the separately launched `octomap_server_node` consumes the
preprocessed cloud and publishes these additional visualization/diagnostic products:

| Direction | Topic | Type | Meaning |
| --- | --- | --- | --- |
| output | `/perception/octomap/projected_map` | `nav_msgs/OccupancyGrid` | OctoMap server's projected occupied cells. |
| output | `/perception/octomap/octomap_binary` | `octomap_msgs/Octomap` | Binary 3-D map from `octomap_server_node`. |

The vehicle classes are COCO `car`, `bus`, and `truck`. A measurement is
accepted only when enough reprojected depth samples lie inside the trimmed
box, its robust range is valid, and its lateral offset is within the configured
2.8 m front corridor. The decision node then checks YOLO/RGB-D range against
the planning-path point cloud except in route-gated direct-control windows.

Traffic-light detections can affect control only at waypoints 25, 32, 52 and
59 and within 20 m of the waypoint. The candidate nearest the configured
target point is classified from HSV pixels inside its box and passed through
the existing temporal stability filter. At waypoint 25, the COCO detector can
intermittently lose the roughly 10x20-pixel signal for an entire green phase.
After the route manager has advanced past that stop waypoint, a tightly
calibrated predicted ROI therefore provides green-only fallback evidence. It
accepts only a compact, bright, saturated lower bulb and still requires the
normal temporal and strict-green decision checks. It cannot override a
red/yellow observation elsewhere on the route. Missing images, TF, camera
calibration, model, or inference errors publish fail-safe `UNKNOWN`,
`detected=false`, and `distance=inf`.

All thresholds, class IDs and topic names are in `config/yolo_rgbd.yaml`. The node clamps
invalid scalar values at startup; the production defaults and enforced ranges for the
most safety-relevant groups are:

| Parameter group | Production value | Unit / enforced constraint |
| --- | --- | --- |
| `confidence_threshold`; `traffic_light_confidence_threshold`; `nms_threshold` | `0.25`; `0.01`; `0.45` | Unitless probability/overlap ratio, each clamped to `[0, 1]`. |
| `maximum_depth_age_s` | `1.0` | Seconds, constrained to `>= 0`; older RGB-depth pairs are rejected. |
| `minimum_range_m`; `maximum_range_m`; `maximum_abs_lateral_m` | `0.5`; `35.0`; `2.8` | Metres; minimum and lateral range are `>= 0`, maximum is `>= minimum_range_m`. |
| `distance_percentile` | `0.20` | Unitless quantile, clamped to `[0, 1]`. |
| `bbox_inner_x_ratio`; `bbox_top_trim_ratio`; `bbox_bottom_trim_ratio` | `0.08`; `0.08`; `0.15` | Image-box ratios, each clamped to `[0, 0.45]`. |
| `traffic_light_max_y_ratio`; ROI min/max/target ratios | route-specific | Normalized image coordinates, clamped to `[0, 1]`; each ROI maximum is also constrained to be at least its minimum. All per-waypoint ROI arrays must have equal length or startup fails. |
| `traffic_light_activation_distance_m` | `20.0` | Metres, constrained to `>= 0`. |
| `traffic_consecutive_frames`; `traffic_confident_green_frames` | `4`; `2` | Frames, each constrained to integer `>= 1`. |
| `traffic_confident_green_max_red_score`; `traffic_confident_green_min_score` | `4`; `20` | Pixel evidence score, each constrained to integer `>= 0`. |

Changing the camera model, image size, route, or Unity scene requires revalidating the
ROI and confidence values with motion disabled before a closed-loop run.

## Improve Unity recognition with fine-tuning

The bundled COCO weights provide a runnable baseline. Because Unity vehicles
and small traffic lights differ visually from real COCO images, collect and
label simulator images when baseline recall is insufficient:

1. Start Unity with motion disabled and collect diverse RGB frames:

   ```bash
   source /opt/ros/jazzy/setup.bash
   python3 tools/collect_yolo_frames.py \
     --output-dir datasets/team20_yolo/unlabelled
   ```

2. Label each vehicle with class `vehicle` and the complete signal housing
   with class `traffic_light`. Export YOLO bounding-box labels, then arrange:

   ```text
   datasets/team20_yolo/
     images/train/  images/val/
     labels/train/  labels/val/
   ```

3. Create the isolated training environment and fine-tune:

   ```bash
   python3 -m venv .venv-yolo-training
   source .venv-yolo-training/bin/activate
   python -m pip install -r requirements-yolo-training.txt
   python tools/train_yolo_sim.py \
     --data datasets/team20_yolo.yaml \
     --output src/perception/models/yolov8n.onnx
   ```

4. Change the class mapping in `config/yolo_rgbd.yaml`:

   ```yaml
   class_count: 2
   vehicle_class_ids: [0]
   traffic_light_class_ids: [1]
   ```

5. Rebuild/install `perception`, run with motion disabled, and inspect
   `/perception/yolo/debug_image` before enabling vehicle motion.

Include near/far views, partial occlusion, every intersection, both NPC
events, and red/yellow/green phases in the data. Split sequentially captured
frames by scene or run rather than randomly, so nearly identical adjacent
frames do not leak into validation.

The bundled Ultralytics model/code is AGPL-3.0 by default. The vendored ONNX
Runtime keeps its MIT license and third-party notices in
`third_party/onnxruntime`.
