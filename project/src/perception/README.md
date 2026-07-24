# Perception

The perception launch converts Unity depth into point clouds and occupancy maps and detects
the relevant dynamic traffic-light colour from aligned RGB and semantic images.

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
are required for ambiguous green evidence. Unambiguous evidence with red score at most 4
and green score at least 20 is accepted after two consecutive frames. Any non-green
observation still revokes green immediately.
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
Ambiguous green still needs four consecutive raw classifications before publication;
strong green uses the two-frame path described above.
