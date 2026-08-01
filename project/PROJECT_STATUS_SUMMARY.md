# Project Status: full-system-v1.9.0

Updated: 2026-08-01

> This document is the written summary for full-system-v1.9.0. It covers the complete
> system architecture, perception model, route and trajectory generation, decision and
> control design, team contributions, code written or substantively modified by Team 20,
> course-supplied code, and reused third-party components. Package READMEs provide more
> detailed topic, parameter, test, and tuning information.

## 1. Current Status

The project has replaced the semantic-segmentation camera with
`RGB + YOLOv8n + Depth/PointCloud` for object detection. In the production configuration,
`segCamera.enabled` is `false`. The YOLO node reads only regular RGB images, depth,
camera calibration, TF, and limited route context; it does not read semantic images or
semantic labels.

The current implementation can:

- load a closed-loop-validated 64-waypoint route from YAML; publish the local route,
  route semantics, target distance, and speed limit; and publish `route_complete` at
  waypoint 63;
- project the vehicle onto the ordered route and generate `/planning/path` using
  Hermite/Bézier geometric smoothing, equal-arc-length resampling, continuity blending,
  and quality gates;
- cap point speed by curvature/lateral acceleration, run forward acceleration and
  backward braking passes, integrate monotonic time, and publish the result in Team 20's
  custom `/planning/trajectory` message;
- run a fixed-size YOLOv8n ONNX model in a C++ ROS 2 node with ONNX Runtime;
- detect the COCO classes `car`, `bus`, `truck`, and `traffic light` in regular RGB images;
- reconstruct depth pixels in 3-D, transform them with TF, reproject them into YOLO
  vehicle boxes, and estimate the lead-vehicle distance;
- select the traffic light controlling the ego lane at waypoints 25, 32, 52, and 59,
  rejecting opposing and cross-traffic lights;
- determine red, yellow, and green states using HSV colours, bulb layout inside the
  detection box, and multi-frame filtering;
- supplement the waypoint-25 case where a small detection box does not fully include the
  lower green bulb, while rejecting the large green road sign below the signal;
- stably associate the YOLO/RGB-D lead vehicle during the Event1 waypoint 17--20 window,
  allowing route-gated vision range to support longitudinal control when the point cloud
  temporarily cannot produce a match;
- retain lead-vehicle association and a safe following target through brief YOLO
  dropouts, preventing the target speed from oscillating between following speed and the
  14 m/s cruise speed;
- replace the fixed 3 m/s far-field limit before traffic-light waypoints 25, 32, 52, and
  59 with a unified distance-based braking envelope;
- command stops approximately 2.0/4.0/3.4/4.0 m before waypoints 25/32/52/59 and use a
  local route reach radius at waypoint 25 so the 2 m stop-line reference is not advanced
  prematurely;
- generate the final target speed with an explicit state machine whose priority is
  emergency braking > traffic-light stop > stale-merge stop > adaptive following >
  merge slowdown > planning speed;
- track the trajectory with Pure Pursuit, regulate speed with a
  derivative-on-measurement PID controller, and ensure that only the production
  controller publishes `/car_command`; and
- publish conservative safety states when detections, the model, TF, or depth data are
  unavailable or invalid.

On 2026-08-01, the production configuration completed a closed-loop waypoint 0--63 route
validation and entered `route_complete`. Mission time was **321.054 s**, measured from
the first forward `/car_command` to the first `route_complete`. The run exercised Event1
following, emergency braking near waypoint 39, right-side merge slowdown, red-light
stops at waypoints 25, 32, and 59, and an already-green passage at waypoint 52. The logs
showed no collision, route departure, or runtime node crash. All ROS nodes exited cleanly;
Unity required forced termination only during teardown after completion.

## 2. Overall Architecture, Models, and Design Choices

### 2.1 System Layers and Authoritative Interfaces

```text
Unity / simulation bridge
  ├─ RGB + Depth + CameraInfo + TF ──> perception
  │       ├─ YOLO RGB-D vehicle/light observations ──────────────┐
  │       ├─ point cloud ──────────────> obstacle_monitor ────────┤
  │       └─ occupancy map / OctoMap ──> visualization/diagnostics
  ├─ world -> OurCar/INS TF ──> route_manager ──> trajectory_generator
  │                                  │                    │       │
  │                                  └─ route context ────┤       │
  │                                                       └─ path + feasible profile
  └─ vehicle twist/TF ────────────────────────────────────────────┤
                                                                  v
                                                        decision_making
                                                    final speed + stop intent
                                                                  │
                planning path + vehicle twist/TF ─────────────────┤
                                                                  v
                                                              controller
                                                                  │
                                                             /car_command
                                                                  │
                                                                  v
                                                        simulation / Unity
```

Package boundaries are part of the design. `perception` reports observations only;
`route_manager` provides ordered route context; `trajectory_generator` produces the
validated path and dynamically feasible profile; `decision_making` combines traffic
rules and obstacle behaviour;
and `controller` converts the final intent into throttle, steering, and braking.
`/planning/target_speed` is not the final control speed: the controller accepts only
`/decision/target_speed`. `/car_command` must have exactly one authoritative publisher so
that multiple controllers cannot overwrite one another.

### 2.2 Main Models and Trade-offs

| Design area | Current choice | Rationale | Cost / limitation |
| --- | --- | --- | --- |
| Global route | A 64-waypoint YAML route on the fixed map | The course map is fixed, so the route is reproducible, inspectable, and configurable with per-waypoint semantics and speeds. | It cannot automatically perform global replanning when the map or task changes. |
| Local trajectory | Ego projection + Hermite/Bézier smoothing + quality gates + forward/backward time parameterization | Produces a transparent, testable path and feasible speed profile under curvature and longitudinal acceleration/braking limits without MPC complexity. | The point-mass envelope omits jerk, tyre friction circles, steering-rate and drivetrain dynamics; obstacle avoidance remains downstream. |
| Object-detection model | COCO YOLOv8n ONNX on regular RGB, using C++/ONNX Runtime CPU inference | Does not rely on the Unity semantic camera; the lightweight model can run directly with the project. | Small traffic lights and the Unity domain gap require route gates, HSV evidence, and temporal compensation. |
| Range and obstacles | RGB-D reprojection + planning-path point-cloud corridor + near-field safety corridor | YOLO provides the class, while depth/point cloud provides range and geometric consistency, reducing false positives in both directions. | Missing sensor data causes conservative stops; selected event windows require a validated direct-vision fallback. |
| Behaviour decision | Explainable state machine, latches, and explicit priorities | Traffic lights, following, merging, and emergency states are traceable and unit-testable with a clear safety priority. | It does not provide the unknown-scenario generalization of a learned policy. |
| Lateral control | Pure Pursuit with speed-dependent lookahead | Stable enough for the fixed route, with a simple formula and transparent steering-direction checks and tuning. | High-dynamic and limit handling is weaker than MPC. |
| Longitudinal control | PID with the derivative term applied to measured speed | Transparent implementation that avoids derivative kick after target-speed steps. | Depends on Unity speed feedback and scenario-specific tuning. |
| Fault handling | Input timeouts, stop latches, bounded outputs, and safe-stop braking | Stale or missing data cannot continue to drive the vehicle using an old command. | Brief TCP or depth jitter can cause conservative transient stops. |

### 2.3 Perception and Longitudinal-Behaviour Details

```text
Regular left RGB image
  └─ YOLOv8n ONNX
      ├─ vehicle bbox
      │    └─ depth pixel -> 3-D -> TF -> RGB reprojection
      │         └─ robust near-surface range inside bbox -> lead-vehicle detection/range
      └─ traffic-light bbox
           └─ route gate + per-intersection ROI/target position
                └─ HSV + bulb layout + downward colour-region extension when needed
                     └─ multi-frame stabilizer -> RED/YELLOW/GREEN/UNKNOWN
```

PointCloud/Depth does not identify what an object is. YOLO supplies the class and 2-D
box, while Depth/PointCloud supplies range, 3-D position, and geometric safety checks.
Consequently, depth and point-cloud processing remain useful after adding YOLO.

#### 2.3.1 Vehicle Detection and Ranging

- YOLO uses COCO class IDs `2/5/7`, corresponding to `car/bus/truck`.
- The left RGB and depth cameras have an approximately 0.2 m baseline, so depth cannot be
  sampled directly at the same pixel coordinates.
- The node back-projects sampled depth pixels into 3-D, transforms them into the RGB
  optical frame through TF, and projects them into the RGB image.
- It retains valid points within the trimmed vehicle box, requires at least 20 points,
  and uses the 20th percentile range to reduce the effect of background points and
  outliers.
- It also checks the valid range and lateral corridor so roadside and opposing vehicles
  are not treated as the ego-lane lead vehicle.

#### 2.3.2 Traffic-Light Detection and Selection

- Traffic lights use a separate low confidence threshold of `0.01`, because a distant
  target may be only about `7 x 15 px`. Vehicle detection remains at `0.25` and is not
  relaxed by the traffic-light threshold.
- Traffic-light results can affect downstream state only at waypoints 25, 32, 52, and 59
  and within 20 m of the intersection.
- Waypoints 25, 52, and 59 use separate perspective ROIs and expected target positions to
  exclude opposing, cross-traffic, edge-of-image, and unrelated high-confidence lights.
- After route management passes the target waypoint, the gate remains active for one
  waypoint so the light remains observable while the vehicle finishes stopping.
- An unambiguous green may be confirmed after two consecutive frames; ordinary or
  ambiguous states require four. Any non-green observation immediately revokes a
  previous permission to proceed, allowing a fast response to red.

#### 2.3.3 Waypoint-25 Green-Light Dropout and Green-Sign Suppression

The small YOLO traffic-light box sometimes contains only the red bulb and housing and
does not fully include the lower green bulb. The current correction does not treat the
entire enlarged area as a normal traffic-light box. Instead, it:

1. keeps the original YOLO box for normal HSV and bulb-layout classification;
2. adds only a small horizontal margin and downward extension to collect supplementary
   green evidence;
3. accepts only a bright, saturated, compact green connected component in a plausible
   lower position;
4. requires the candidate not to touch the crop boundary and constrains its area and
   shape; and
5. normalizes the score against the original detection-box area before applying the
   strict green threshold.

This recovers a small, bright lower green bulb, while a wide green road sign that occupies
a large area or touches the crop boundary cannot become permission-to-proceed evidence.
Unit tests cover a truncated green-light box, an extension touching the image boundary,
and a large green sign below the signal.

#### 2.3.4 Event1 Lead-Vehicle Detection and Continuous Following

During Event1, the vehicle is inside the ego-lane camera corridor at waypoints 17--20,
but the preprocessed point cloud may temporarily fail to provide a consistent path
distance. Obstacle fusion and decision logic therefore use these measures:

- `vision_direct_control_waypoint_ids` includes waypoints 17--20. Within this validated
  window, a stable YOLO/RGB-D vehicle range inside the corridor may directly support
  lead-vehicle control.
- `vision_vehicle_detection_hold_time=1.3 s` retains the last trusted association through
  brief detection gaps, avoiding a return to full cruise followed by renewed braking.
- Following activates within 20 m and continuously computes the target speed using a
  2.2 s time gap, distance error, and filtered range rate.
- Event1 limits the target-speed decrease to `2.0 m/s²`, reducing it smoothly from the
  current speed.
- A 2 m release hysteresis beyond the 20 m activation boundary reduces repeated entry
  and exit near the threshold.
- Event1 direct vision control stops after waypoint 20 because the lead vehicle begins to
  move toward the roadside, preventing a second unnecessary braking event.
- Independent near-field emergency braking remains active and is not weakened by the
  smooth-following logic.

#### 2.3.5 Traffic-Light Speed Limits and Progressive Braking

The far-field route speed limit at traffic-light waypoints 25, 32, 52, and 59 is `8 m/s`.
This removes the `14 -> 3 m/s` target-speed step that the route layer previously generated
when entering a `stop_zone`. For RED, YELLOW, or UNKNOWN, the decision layer limits the
target speed with this envelope:

```text
v_target = min(planning_speed, 8.0, sqrt(2 * 0.7 * remaining_distance))
remaining_distance = waypoint_distance - active_stop_distance
```

The vehicle therefore decelerates continuously as its stopping margin decreases instead
of braking at full force immediately upon entering the intersection waypoint. Current
stop boundaries are:

| Controlled waypoint | Far-field route limit | Stop boundary | Notes |
| ---: | ---: | ---: | --- |
| 25 | 8 m/s | 2.0 m | Local `reach_distance=1.5 m` prevents the global 3 m arrival radius from advancing to waypoint 26 too early. |
| 32 | 8 m/s | 4.0 m | The 3 m/s turn limits at waypoints 33--35 remain unchanged. |
| 52 | 8 m/s | 3.4 m | The stop line is shifted 0.6 m forward from the default. |
| 59 | 8 m/s | 4.0 m | The final traffic light used for initial braking-envelope validation. |

Traffic-light braking is released after green confirmation. Speed limits on subsequent
turning waypoints remain effective.

### 2.4 Trajectory-Planning Model and Interface

The planner separates geometry from time parameterization. After route projection,
smoothing, arc-length resampling, and quality validation, each point receives a speed
cap:

```text
v_cap[i] = min(route_limit, max_speed,
               sqrt(max_lateral_acceleration / abs(curvature[i])))
```

A forward pass applies `v[i]^2 <= v[i-1]^2 + 2*a_accel*ds`, and a backward pass applies
`v[i]^2 <= v[i+1]^2 + 2*a_decel*ds`. The backward pass anticipates both curved sections
and the zero terminal speed at `route_complete`. Point time is then integrated with
`dt = 2*ds/(v[i-1]+v[i])`. Production limits are 3.0 m/s² lateral acceleration,
2.0 m/s² longitudinal acceleration, and 3.0 m/s² deceleration.

Team 20's `autonomy_interfaces/msg/PlannedTrajectory` contains an ordered array of
`TrajectoryPoint` records: pose, signed curvature, target speed, and time from start. A
topic was chosen because planning output is a continuously refreshed stream; a service
would introduce synchronous request/reply coupling. The standard `/planning/path`
remains the Pure Pursuit reference. The scalar `/planning/target_speed` is taken from the
constrained profile at a 4 m arc-length lookahead, passes through `decision_making`, and
then reaches the existing controller as `/decision/target_speed`.

This design keeps `velocity_profile.py` pure and ROS-independent. Focused unit tests
exercise lateral caps, acceleration reachability, pre-braking, terminal stopping, and
time monotonicity, while rosbag analysis checks the serialized runtime interface.

### 2.5 v1.9.0 Code-Quality and Documentation Alignment

This release aligned Team 20's code style while documenting the newly added Team 20
trajectory message and preserving existing controller-facing interfaces:

- fixed `route_manager` flake8 and PEP 257 issues and added malformed-route tests;
- added the root `.editorconfig` and `project/CODE_STYLE.md` to align readability rules
  across Python, C++, launch files, tests, and tools;
- added `project/tools/check_code_quality.sh` as one gate for Python flake8/PEP 257 and
  C++ cpplint/uncrustify;
- kept the course-supplied `dummy_controller` at the v1.8.0 baseline and outside Team 20
  style modifications;
- protected vendored ONNX Runtime, the JSON header, and other third-party content from
  unrelated editor formatting changes; and
- expanded the root README and `src/README.md` with the four-member contribution record,
  project capabilities, dependency installation, clone/pull, build, test, full-run, and
  troubleshooting instructions.

## 3. Key Files

```text
# Complete system chain
src/route_manager/route_manager/route_manager_node.py
src/route_manager/route_manager/route_map.py
src/route_manager/route_manager/route_progress.py
src/route_manager/config/route_map.yaml
src/trajectory_generator/trajectory_generator/path_processor.py
src/trajectory_generator/trajectory_generator/velocity_profile.py
src/trajectory_generator/trajectory_generator/trajectory_generator_node.py
src/trajectory_generator/test/test_time_parameterization.py
src/autonomy_interfaces/msg/TrajectoryPoint.msg
src/autonomy_interfaces/msg/PlannedTrajectory.msg
src/decision_making/decision_making/obstacle_monitor_node.py
src/decision_making/decision_making/traffic_light_state_machine.py
src/controller/src/control_core.cpp
src/controller/src/controller_node.cpp
src/system_bringup/launch/full_system.launch.py

# Perception and model
src/perception/src/yolo_rgbd_detector_node.cpp
src/perception/include/perception/yolo_postprocess.hpp
src/perception/include/perception/traffic_light_route_gate.hpp
src/perception/include/perception/traffic_light_stability_filter.hpp
src/perception/config/yolo_rgbd.yaml
src/perception/launch/perception.launch.py
src/perception/models/yolov8n.onnx
src/perception/models/README.md
tools/collect_yolo_frames.py
tools/export_yolo_onnx.py
tools/train_yolo_sim.py
datasets/team20_yolo.yaml
datasets/team20_yolo/README.md
requirements-yolo-training.txt

# v1.9.0 code quality and project-level documentation
../.editorconfig
CODE_STYLE.md
tools/check_code_quality.sh
tools/analyze_mission_run.py
docs/ros_graph.svg
mission_results/README.md
mission_results/metrics.json
../README.md
src/README.md

# Event1, intersection speed, and braking configuration/tests
src/decision_making/config/traffic_light_state_machine.yaml
src/decision_making/test/test_traffic_light_state_machine.py
src/route_manager/route_manager/route_progress.py
src/route_manager/config/route_map.yaml
src/route_manager/test/test_route_progress.py
src/route_manager/test/test_route_safety_geometry.py
```

## 4. Team Contributions, Team 20 Code, and Reused Content

“Team 20 code” means implementation, configuration, tests, and integration logic written
or substantively modified by the team for this project. It does not mean the team
invented the public algorithms used. Original ownership of course-supplied code and
third-party dependencies does not transfer to Team 20 when those components receive ROS 2
compatibility, formatting, or integration changes.

### 4.1 Who Did What?

| Member | Primary responsibility | Traceable modules / artifacts |
| --- | --- | --- |
| Lei Fang | `route_manager` | `src/route_manager`: route YAML, waypoint recorder, route loading/validation, progress tracking, local route, and route tests |
| Yida Lyu | `decision`, `controller`, part of integration | `src/decision_making`, `src/controller`: state machine, obstacle behaviour, final target speed, Pure Pursuit, PID, safe stop, and decision-to-command interfaces |
| Tao Zhang | Perception, part of integration | `src/perception`: YOLO RGB-D, traffic-light colour/temporal logic, vehicle range, point cloud/occupancy, model configuration, and perception-to-decision interfaces |
| Jianing Huang | `Trajectory_Generator`, part of integration | `src/trajectory_generator`, `src/autonomy_interfaces`: route projection, smoothing/resampling, continuity/quality gates, constrained time parameterization, custom trajectory messages, and route-to-planning interfaces |

Integration is shared work by Yida Lyu, Tao Zhang, and Jianing Huang. It includes
topic/config alignment at their module boundaries, `system_bringup` integration,
end-to-end testing, dynamic validation, and documentation. The team did not provide a
finer-grained individual split, so this document does not invent one.

### 4.2 Whole-System Ownership Table

| Component | Ownership category | Team 20 work / reuse boundary |
| --- | --- | --- |
| `route_manager` | Team-written / substantively developed | Route model, recording tool, load validation, progress and pass detection, local route, parameters, tests, and documentation; led by Lei Fang. |
| `autonomy_interfaces` and `trajectory_generator` | Team-written / substantively developed | Team 20 messages; route projection; Hermite/Bézier application; resampling; continuity/quality validation; curvature and acceleration-constrained time parameterization; fallback, tests, and documentation; led by Jianing Huang. |
| `decision_making` | Team-written / substantively developed | Traffic-light state machine, obstacle fusion, following/merge/emergency priorities, timeouts/latches, tests, and documentation; led by Yida Lyu. |
| `controller` | Team-written / substantively developed | Project implementation of Pure Pursuit/PID, input validation, output limits, safe stop, parameters, tests, and documentation; led by Yida Lyu. |
| YOLO RGB-D and project mapping logic in `perception` | Team-written / substantively developed around a reused model and libraries | ROS nodes, RGB-D reprojection, candidate selection, HSV/temporal rules, occupancy outputs, configuration, tests, and documentation; led by Tao Zhang. |
| `system_bringup`, project configuration, integration tests/documentation | Team integration | Single launch entry, module switches, safe defaults, RViz, interface integration, and validation; shared by Yida/Tao/Jianing. |
| `simulation` / Unity bridge and assets | Course-supplied, retained/reused | Supplies sensors, vehicle state, and command transport. Team 20 invokes it through `system_bringup` and project configuration and does not claim the simulator, bridge, or original ROS 2 port as its own work. |
| `dummy_controller` | Course-supplied, not production | Example controller retained at the project baseline; it is not part of Team 20's production control or code-style assessment scope. |
| `.editorconfig`, `CODE_STYLE.md`, quality gate, and project-level READMEs | Shared Team 20 deliverables | Style alignment, quality checking, traceability, run instructions, and release documentation; no finer individual split is claimed. |

Reused algorithms and implementation boundaries:

| Method / dependency | Reused content | Team 20 implementation |
| --- | --- | --- |
| Pure Pursuit / PID | Public control methods and mathematical principles | Yida Lyu's C++ implementation, ROS adaptation, safety checks, and parameter tuning for this project. |
| Hermite / Bézier interpolation and constant-acceleration reachability equations | Public geometric and kinematic principles | Jianing Huang's route trimming, tangent constraints, curve joining, resampling, quality gates, continuity logic, forward/backward constraint passes, ROS interface, and project tuning. |
| Finite-state machine pattern | General software design pattern | Yida Lyu's traffic-light, following, merge, emergency-braking states, priorities, and latches. |
| YOLOv8n / COCO weights | Ultralytics network, pretrained weights, and class knowledge | Tao Zhang's ROS 2 ONNX inference integration, RGB-D ranging, route gating, and traffic-light postprocessing. |
| ROS 2 / TF / OpenCV / ONNX Runtime / OctoMap | Third-party frameworks and libraries | Team 20's nodes, topics, parameters, transformations, configuration, and system integration. |

### 4.3 YOLO Integration Code Written or Modified by Team 20

| Content | Files | Team 20 work |
| --- | --- | --- |
| ROS 2 YOLO RGB-D node | `src/perception/src/yolo_rgbd_detector_node.cpp` | Image subscriptions, ONNX Runtime calls, result publication, RGB-D reprojection, vehicle range, traffic-light candidate selection, and debug output. |
| YOLO postprocessing | `src/perception/include/perception/yolo_postprocess.hpp` | Letterbox coordinate recovery, support for multiple YOLO output layouts, per-class NMS, HSV/layout colour classification, waypoint-25 supplementary green evidence, and green-sign suppression. |
| Intersection and temporal logic | `traffic_light_route_gate.hpp`, `traffic_light_stability_filter.hpp` | Activation and post-waypoint hold at four intersections, rapid revocation of green permission, and multi-frame stable confirmation. |
| Parameter and launch integration | YOLO-related parts of `yolo_rgbd.yaml`, `perception.launch.py`, and `CMakeLists.txt` | Classes, thresholds, ROIs, topics, model paths, node setup, and shared-library installation. |
| Data-tool wrappers | `collect_yolo_frames.py`, `train_yolo_sim.py`, `export_yolo_onnx.py`, dataset YAML/README | Project-specific wrappers around ROS image collection and the Ultralytics training/export APIs. |

This code is project logic written around a third-party model and libraries. In
particular, `tools/train_yolo_sim.py` and `tools/export_yolo_onnx.py` are local wrappers
around the public Ultralytics API; the trainer, network architecture, loss functions, and
exporter are not Team 20 code.

### 4.4 Reused Open-Source Models, Code, and Libraries

| Third-party content | Use in this project | Version / source evidence | Licence and ownership statement |
| --- | --- | --- | --- |
| Ultralytics YOLOv8n | Detection architecture, COCO-pretrained weights, offline training, and ONNX export | `yolov8n.onnx`, `requirements-yolo-training.txt`, and the three tool scripts | Repository documentation records the default as AGPL-3.0; the model and Ultralytics implementation are not Team 20 work. |
| COCO-pretrained content | Initial classes and pretrained knowledge of the current model | Configuration uses 80 classes and COCO class IDs. | The dataset and annotations were not created by Team 20. Reports must identify the model as COCO-pretrained and must not describe COCO data as team-owned. |
| Microsoft ONNX Runtime CPU | C++ execution of the ONNX model | Vendored version `1.22.0` | MIT; `third_party/onnxruntime/LICENSE` and `ThirdPartyNotices.txt` are retained. |
| OpenCV | Letterboxing, image processing, NMS, HSV, and debug rendering | Current system version `4.6.0` | The Debian package declares Apache-2.0 AND BSD-3-Clause and includes notices for other third-party components. |
| ROS 2 Jazzy APIs | Nodes, messages, parameters, TF, and launch interfaces | Dependencies such as `rclcpp`, `sensor_msgs`, and `tf2_ros` | External ROS 2 packages; this project only calls their APIs and follows each package's licence. |
| NumPy, PyYAML, rclpy | Collection and training helper scripts | Python imports and training requirements | External Python packages; they are not part of the runtime C++ YOLO node. |

### 4.5 Model and Licence Record

Information for the model currently distributed with the project:

```text
file:    src/perception/models/yolov8n.onnx
size:    12,824,062 bytes
sha256:  8707bb120b03a6bf8a199e7c35c6d47ad5f83de4fdd233110db270f7d355a3be
input:   fixed 640 x 640
classes: 80 COCO classes
runtime: Microsoft ONNX Runtime 1.22.0 CPU
```

`src/perception/package.xml` currently declares the perception package as
`AGPL-3.0-only`, consistent with the repository's licence record for the default
Ultralytics model/code. ONNX Runtime retains its separate MIT licence and third-party
notices, which must not be removed or replaced by the perception package licence.

`requirements-yolo-training.txt` currently constrains only `ultralytics>=8.3,<9`; it does
not pin the exact export version. The repository also does not record a checksum for the
original `yolov8n.pt`. Before a final public report or release, record:

- the exact Ultralytics version used for export;
- the original `.pt` checkpoint download source and SHA-256;
- the export command, opset, and date; and
- for future custom-data fine-tuning, image/annotation authors, collection scope,
  training/validation split, and data licence.

This section is based on files and licence headers in the current workspace. If team
members manually copied snippets from examples, forums, or other projects that are not
recorded in the repository, the relevant author must add those sources to the final
documentation.

### 4.6 Text Ready for Use in the English Report

> The YOLO integration code was developed by Team 20. It includes the ROS 2
> C++ inference node, RGB-D reprojection and ranging, YOLO output parsing and
> non-maximum suppression, route-aware traffic-light selection, HSV/layout
> colour classification, temporal filtering, configuration, diagnostics, and
> runtime diagnostics. The underlying YOLOv8n architecture, COCO-pretrained
> weights, and Ultralytics training/export implementation are reused
> third-party components and are not our own work. Runtime inference uses the
> reused Microsoft ONNX Runtime 1.22.0 library, while image processing uses
> OpenCV 4.6.0 and the node interface uses ROS 2 Jazzy. Their respective
> copyright notices and licences are retained.

## 5. Interfaces and Runtime Parameters

Trajectory-planning inputs and outputs:

```text
inputs:
  /route/local_path                  nav_msgs/msg/Path
  /route/target_pose                 geometry_msgs/msg/PoseStamped
  /route/speed_limit                 std_msgs/msg/Float32
  /route/route_state                 std_msgs/msg/String
  /OurCar/INS/twist                  geometry_msgs/msg/TwistStamped
  TF world -> OurCar/INS

outputs:
  /planning/path                     nav_msgs/msg/Path
  /planning/trajectory               autonomy_interfaces/msg/PlannedTrajectory
  /planning/target_speed             std_msgs/msg/Float32
```

Each `TrajectoryPoint` contains `pose`, `curvature`, `target_speed`, and
`time_from_start`. Configured dynamics are `max_lateral_acceleration=3.0`,
`max_longitudinal_acceleration=2.0`, and `max_longitudinal_deceleration=3.0` in SI units;
the scalar control lookahead is 4.0 m and measured velocity expires after 1.0 s.

Primary inputs:

```text
/OurCar/Sensors/RGBCameraLeft/image_raw
/OurCar/Sensors/RGBCameraLeft/camera_info
/OurCar/Sensors/DepthCamera/image_raw
/OurCar/Sensors/DepthCamera/camera_info
/planning/path
/route/current_waypoint_id
/route/current_waypoint_distance
```

Primary outputs:

```text
/perception/front_vehicle_detected
/perception/front_vehicle_distance
/perception/front_vehicle_debug
/perception/traffic_light_state
/perception/traffic_light_scores
/perception/traffic_light_debug
/perception/yolo/debug_image
```

Event1 following and intersection braking also use these control-chain interfaces:

```text
/route/speed_limit
/planning/target_speed
/decision/front_obstacle_distance
/decision/merge_obstacle_distance
/decision/target_speed
/decision/state
/decision/stop_required
/decision/debug
```

The controller must use `/decision/target_speed`, not `/planning/target_speed`. This
allows Event1 following, traffic-light distance braking, and emergency braking to be
combined by priority into one final target speed.

Fail-safe values are:

```text
front_vehicle_detected = false
front_vehicle_distance = inf
traffic_light_state = UNKNOWN
```

## 6. Validation Status

### 6.1 Current full-system-v1.9.0 Validation

The complete release check was run against the current source on 2026-08-01:

| Validation item | Result |
| --- | --- |
| Project code-quality gate | All 53 Team 20/configured Python files and configured C++ sources passed; the inherited simulator bridge uses the documented exemption. |
| ROS 2 build | All nine workspace packages succeeded. |
| Complete `colcon test` | 184 tests, 0 errors, 0 failures, 1 expected skip. |
| Unity closed-loop route | Completed waypoints 0--63 and entered `route_complete`. |
| Mission time | 321.054 s from first forward command to `route_complete`; 319.116 s from first measured motion. |
| Runtime stability | No runtime `ERROR`, `FATAL`, unexpected process exit, collision, or route departure before completion. |
| Shutdown | All ROS nodes exited cleanly; Unity required forced termination during teardown only. |
| Trajectory audit | 147,717 points / 3,943 messages; all finite, strictly increasing time, and within configured limits plus 0.05 m/s² numeric tolerance. |

The complete run also confirmed that:

- Event1 entered `FOLLOWING_VEHICLE` near waypoint 17 and returned to `CRUISE` after the
  constraint cleared;
- the merge area at waypoints 47--48 entered `SLOWING_FOR_MERGE` and recovered safely;
- traffic-light logic stopped and released at red signals 25, 32, and 59; waypoint 52
  was already green and was passed without an unnecessary stop;
- `/car_command` had only one publisher, the production `controller_node`;
- `/perception/traffic_light_state` had only one publisher, the production YOLO RGB-D
  node; and
- RGB, depth, TF, YOLO, occupancy mapping, route, trajectory, decision, and control
  continued operating without the semantic camera.

Warnings about missing TF/path/velocity may occur during startup, and brief stale-depth
diagnostics may occur during operation. The controller stops safely while inputs are
incomplete and resumes when fresh data arrives. These warnings did not interrupt the
validated run.

### 6.2 Historical Static Validation for full-system-v1.8.0

On 2026-07-29, the route-management and traffic-light/vehicle-decision tests excluding
linters were run against the source at that time:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
pytest -q -m 'not linter' \
  src/route_manager/test \
  src/decision_making/test/test_traffic_light_state_machine.py
```

The result was `87 passed, 3 deselected`. These tests covered:

- the Event1 direct-vision control window at waypoints 17--20;
- Event1 distance-boundary hysteresis, brief detection-loss retention, and smooth
  target-speed reduction;
- the priority between lead-vehicle following and independent near-field emergency
  braking;
- the distance-based braking envelope at waypoints 25, 32, 52, and 59;
- local stop boundaries of 2.0 m at waypoint 25 and 3.4 m at waypoint 52;
- the 1.5 m local route reach radius at waypoint 25; and
- the 8 m/s far-field route limit at all four dynamic traffic-light waypoints, with
  subsequent turn limits unchanged.

These results validate state transitions, speed calculations, route configuration, and
waypoint advancement. They do not replace closed-loop braking-distance tests in Unity.

Before the v1.8.0 release, the five affected packages were rebuilt and tested again with
`colcon`:

| Test group at that time | Cases | Result |
| --- | ---: | --- |
| Perception focused YOLO ROI | 4 | All passed. |
| Controller | 11 | All passed. |
| Decision making | 98 | All passed. |
| Trajectory generator | 37 | All passed. |
| Route manager functional | 15 | All passed; three linter cases were excluded by the command. |
| Total | 165 | All passed. |

The final perception test run before those test sources were removed, on 2026-07-27,
reported:

| Test group | Cases | Result |
| --- | ---: | --- |
| TrafficLightStabilityFilter | 4 | All passed. |
| TrafficLightRouteGate | 4 | All passed. |
| YoloPostprocess | 11 | All passed. |
| YoloModelRuntime | 1 | All passed. |
| Total | 24 | 0 errors, 0 failures, 0 skipped. |

At that time, coverage included:

- letterbox geometry recovery;
- YOLO feature-first, end-to-end, and custom two-class output layouts;
- per-class NMS;
- red/yellow/green HSV and bulb-position classification;
- truncated small green boxes, downward supplementary green evidence, image-boundary
  cropping, and green-sign rejection;
- intersection activation, post-waypoint hold, and recovery after node restart;
- fast confirmation of unambiguous green, delayed confirmation of ambiguous states, and
  immediate revocation on non-green; and
- a forward pass through the real repository model with vendored ONNX Runtime followed
  by output parsing.

This table remains a historical validation record from 2026-07-27. The current version
has restored a focused `test_yolo_postprocess` target and the `ament_cmake_gtest`
dependency. Its four cases cover the predicted ROI's compact green bulb, red-light
rejection, crop-boundary rejection, and large-green-sign rejection. The other historical
route-gate, stabilizer, and real-model runtime test sources have not been restored, so the
complete historical group of 24 tests cannot currently be rerun directly.

### 6.3 Dynamic Validation Record

Current v1.9.0 dynamic results from 2026-08-01:

- the complete waypoint 0--63 route passed and reached `route_complete`;
- mission time was 321.054 s by the first-forward-command definition, with a 319.116 s
  first-motion cross-check;
- Event1 adaptive following and the merge slowdown at waypoints 47--48 activated and
  recovered normally;
- emergency braking activated once near waypoint 39 and recovered without collision;
- red-light stop/green-release behaviour at waypoints 25, 32, and 59 was correct, and
  waypoint 52 passed on green without an unnecessary stop;
- 147,717 serialized trajectory points respected the configured lateral and longitudinal
  constraints; there were zero non-monotonic time pairs and zero invalid values;
- the system remained safely stopped after route completion and all processes shut down
  normally except for Unity's teardown-only forced termination; and
- log keyword scans found no runtime error, crash, collision, or process death before
  route completion.

Historical dynamic results from 2026-07-27:

- the complete waypoint 0--63 route passed and reached `route_complete`;
- after the waypoint-25 correction, true-green supplementary scores were approximately
  90--103 against a strict threshold of 30;
- waypoints 25, 52, and 59 selected the signal controlling the ego lane and passed; and
- the temporary separately started YOLO node was stopped after final testing, leaving no
  duplicate process.

The Event1 continuous-following behaviour, four-intersection smooth deceleration, and
stop-line configuration added after 2026-07-29 are covered by the 2026-08-01 v1.9.0
complete closed-loop route validation described above.

During testing, two YOLO nodes with the same name were once found publishing light state
simultaneously, causing RED and GREEN results to overwrite each other. At runtime,
`/perception/traffic_light_state` and the other perception outputs must have only one YOLO
publisher. This was a duplicate startup/process issue, not a model-classification issue.

## 7. Build and Safety Checks

```bash
source /opt/ros/jazzy/setup.bash
bash tools/check_code_quality.sh
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose

# On first startup, disable motion and inspect boxes, distances, and light state.
ros2 launch system_bringup full_system.launch.py \
  enable_motion:=false \
  enable_visualization:=true

# Confirm that each output has exactly one publisher.
ros2 topic info /perception/traffic_light_state --verbose
```

When substituting a compatible ONNX model:

```bash
ros2 launch system_bringup full_system.launch.py \
  yolo_model:=/absolute/path/to/best.onnx \
  enable_motion:=false
```

## 8. Current Limitations and Future Documentation

- The project currently includes the COCO YOLOv8n baseline, not a custom model already
  fine-tuned on Unity data. `datasets/team20_yolo` currently contains only directory
  instructions and YAML, with no training images or labels attributable to Team 20.
- Traffic lights are very small, so the implementation depends on a low YOLO threshold,
  route gating, and per-intersection ROIs. The complete route has passed, but changing
  camera parameters, resolution, route, or map requires ROI recalibration.
- Trajectory feasibility uses a point-mass speed envelope. It does not jointly optimize
  path and speed or model jerk, tyre-force coupling, steering-rate, vehicle mass,
  drivetrain lag, or moving-obstacle avoidance. Those extensions would require a richer
  vehicle model and likely an optimization-based planner.
- v1.9.0 completed one production closed-loop waypoint 0--63 run containing both events.
  Metrics and figures from the trajectory validation are persisted in `mission_results`,
  but the large raw rosbag is not stored in Git. A separate no-event control run and
  long-term raw bag/config archive remain desirable for complete third-party replay.
- Final documentation must distinguish “Team 20's YOLO integration contribution” from
  the reused “YOLOv8n/COCO/ONNX Runtime/OpenCV/ROS 2” components. It must not state
  broadly that “YOLO was developed by us.”

## 9. ROS Graph, Result Figures, and Bibliography

The package/topic graph below records both runtime flow and the member responsible for
each team component. Grey denotes course-supplied simulation; coloured boxes denote Team
20 modules. The editable source is `docs/ros_graph.dot`.

![Team 20 ROS graph and contribution ownership](docs/ros_graph.svg)

The recorded route trace demonstrates waypoint 0--63 completion. The speed plot shows
that planning produces a curvature/acceleration-constrained target, after which the
decision layer may reduce speed for traffic lights and obstacles. These figures are
generated from the same rosbag as `mission_results/metrics.json`.

![Recorded full-system route trace](mission_results/route_trace.svg)

![Measured, planned, and decision speed](mission_results/speed_profile.svg)

### Bibliography

1. Technical University of Munich, *Introduction to ROS: Autonomous Driving Project
   Assignment*, `AutonomousDriving.pdf`, 2026. This is the assessment specification used
   to derive the trajectory, custom-interface, documentation, and validation criteria.
2. R. C. Coulter, [*Implementation of the Pure Pursuit Path Tracking
   Algorithm*](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm),
   Carnegie Mellon University, CMU-RI-TR-92-01, 1992.
3. G. Farin, *Curves and Surfaces for CAGD: A Practical Guide*, 5th ed., Morgan
   Kaufmann, 2002, ISBN 978-1-55860-737-8. Hermite and Bézier curves are reused public
   methods; Team 20 owns the project-specific cropping, blending, gates, and tuning.
4. K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design, and Tuning*, 2nd ed.,
   Instrument Society of America, 1995, ISBN 1-55617-516-7.
5. Open Robotics, [*ROS 2 Jazzy: Interfaces (topics, services,
   actions)*](https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html),
   accessed 2026-08-01.
6. Ultralytics, [*Model Export with Ultralytics
   YOLO*](https://docs.ultralytics.com/modes/export/), accessed 2026-08-01.
7. Microsoft, [*ONNX Runtime C++ getting-started
   documentation*](https://onnxruntime.ai/docs/get-started/with-cpp.html), accessed
   2026-08-01.
8. A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard, “OctoMap: An
   Efficient Probabilistic 3D Mapping Framework Based on Octrees,” *Autonomous Robots*,
   vol. 34, pp. 189--206, 2013,
   [doi:10.1007/s10514-012-9321-0](https://doi.org/10.1007/s10514-012-9321-0).
