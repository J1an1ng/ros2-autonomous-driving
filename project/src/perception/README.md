# Perception

The perception launch converts Unity depth into point clouds and occupancy maps, detects
the relevant dynamic traffic-light colour, and publishes a diagnostic semantic-vehicle
distance from the aligned semantic and depth cameras.

## Launch

Perception is normally started by the single system launch. Motion remains disabled unless
it is explicitly enabled:

```bash
ros2 launch system_bringup full_system.launch.py enable_motion:=false
```

For perception-only debugging, disable route, decision, trajectory, and controller through
the corresponding full-system launch arguments. OctoMap can be disabled with
`enable_octomap:=false` when only camera/traffic-light tuning is required.

## Outputs

| Topic | Meaning |
| --- | --- |
| `/perception/pcl/points` | Raw depth projection in the depth-camera frame. |
| `/perception/pcl/points_preprocessed` | Ground-filtered obstacle cloud in `world`. |
| `/perception/depth/projected_map` | Local 2D occupied cells at 0.25 m resolution. |
| `/perception/extended_map` | Inflated 2D map. |
| `/perception/octomap/octomap_binary` | Binary 3D OctoMap. |
| `/perception/octomap/projected_map` | OctoMap 2D projection. |
| `/perception/traffic_light_state` | `0 UNKNOWN`, `1 RED`, `2 YELLOW`, `3 GREEN`. |
| `/perception/traffic_light_debug` | Raw/stable state, scores, selected semantic component, ROI, and timestamp delta. |
| `/perception/front_vehicle_detected` | `true` only when a red vehicle component has enough fresh valid depth samples. |
| `/perception/front_vehicle_distance` | Robust near-surface depth in metres for the nearest accepted red vehicle component, otherwise `inf`. |
| `/perception/front_vehicle_debug` | Colour-mask, component, bounding-box, depth-pixel, distance, and synchronization diagnostics. |

The vehicle detector treats red semantic pixels as NPC vehicles and deliberately does not
match the yellow semantic class used by traffic lights. It extracts connected components
inside a configurable central ROI, rejects small colour noise, and fuses each component with
the latest depth image. Its reported distance is the 20th percentile of valid component
depths rather than a single minimum pixel. Depth-camera intrinsics then convert the component
centre into a lateral offset; components outside `max_abs_lateral_m=2.8` remain visible in
debug output but are not reported as front vehicles. Decision making consumes these topics
and requires fresh semantic/point-cloud range agreement outside the validated waypoint
43--48 direct-semantic window. This prevents red cross traffic and roadside vehicles from
causing longitudinal braking elsewhere on the route.

With no simulator running, the isolated fusion check can be repeated with
`python3 tools/smoke_test_semantic_vehicle_detector.py`. It verifies a synthetic red
component at 12 m and verifies that an equally sized yellow component is rejected.

The traffic detector uses the left RGB image for bulb colour and the semantic image only to
select traffic-light components. `signal_group` first selects the component nearest the
expected forward-signal position, then groups vertically separated high/low lamps whose
horizontal centers are within `signal_group_max_dx_ratio=0.16` of the image width and whose
semantic areas are at least `signal_group_min_size_ratio=0.45` of the largest group member.
The relative-size guard rejects a smaller adjacent signal between the two main lamps. Each lamp
is classified independently before group aggregation: red has safety priority across lamp
instances, while green is accepted only when every currently selected lamp is confirmed
green. An unclassified member therefore cannot be masked by another member's green. This lets a
lower lamp take over when its high-mounted partner leaves the camera view without confusing
same-housing red reflections with a real red lamp. Four consecutive group classifications
are required for mixed red/green evidence. A strong green with at most four red-score
points and at least 20 green-score points is accepted after two consecutive frames.
Every red, yellow, or unknown frame still revokes green permission immediately.
If no valid component exists, or the RGB/semantic timestamp difference exceeds
`max_semantic_stamp_delta_s=1.0`, the published state is UNKNOWN. This prevents a frozen
semantic frame from continuously supporting stale green evidence.

Stage 4 verified controlled waypoint IDs 25, 32, 52, and 59. The detector can see lights on
adjacent roads outside those approaches; the decision node's waypoint allowlist is therefore
required and must not be removed during tuning.

The principal detector thresholds remain parameters declared in
`traffic_light_detector_node.cpp`. Inspect `/perception/traffic_light_debug` before changing
them. The deterministic simulator profile deliberately gives red and green broad hue,
saturation, and value gates, while yellow uses the narrower 42--58 degree hue interval and
higher saturation/value requirements. `yellow_over_green_ratio=1.25` additionally requires
yellow evidence to clearly exceed concurrent green evidence before yellow can override it.
A true yellow with no green evidence remains yellow. In the simulator, a lit green bulb may
coexist with red housing reflections, so `green_over_red_ratio=0.80` lets green override them
once its evidence reaches 80% of red; a pure red frame has no green evidence and remains red.
Mixed red/green evidence still needs four consecutive raw green classifications before
publication; unambiguous green needs two.
