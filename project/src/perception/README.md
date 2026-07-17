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
select traffic-light components. `strict_center` selects the component nearest the image
centre. Four consecutive raw classifications are required before the stable state changes.
If no valid component exists, or the RGB/semantic timestamp difference exceeds
`max_semantic_stamp_delta_s=1.0`, the published state is UNKNOWN. This prevents a frozen
semantic frame from continuously supporting stale green evidence.

Stage 4 verified controlled waypoint IDs 25, 32, 52, and 59. The detector can see lights on
adjacent roads outside those approaches; the decision node's waypoint allowlist is therefore
required and must not be removed during tuning.

The principal detector thresholds remain parameters declared in
`traffic_light_detector_node.cpp`. Inspect `/perception/traffic_light_debug` before changing
them. Stage-4 runs detected stable red, yellow, and green at all four controlled approaches
with the defaults, so threshold changes are not currently required.
