# Full-System Trajectory Validation

Validation date: 2026-08-01

The production stack was launched from waypoint 0 with motion enabled and the
Unity semantic camera disabled. The vehicle reached waypoint
63 and published `route_complete`.

## Mission result

- Mission time: **321.054 s**.
- Timing definition: first forward /car_command (throttle > 0.01, brake < 0.01) to first route_complete.
- Cross-check from first measured vehicle motion (> 0.1 m/s):
  **319.116 s**.
- Maximum measured speed: 5.846 m/s.
- Path-tracking RMSE / P95: 1.0025 m /
  2.6278 m.

## Trajectory constraint audit

The audit checked 147717 points in
3943 `/planning/trajectory` messages.

| Constraint | Configured limit | Maximum observed | Result |
| --- | ---: | ---: | --- |
| Lateral acceleration `v^2 * abs(curvature)` | 3.00 m/s² | 3.0000 m/s² | Pass |
| Longitudinal acceleration | 2.00 m/s² | 2.0005 m/s² | Pass |
| Longitudinal deceleration | 3.00 m/s² | 3.0000 m/s² | Pass |
| Strictly increasing point time | required | 0 violations | Pass |
| Finite numeric trajectory values | required | 0 invalid points | Pass |

The spatial acceleration audit uses adjacent published poses and speeds. A
0.05 m/s² numerical tolerance covers float32 serialization and the difference
between curved arc length and pose-to-pose chord length.

## Scenario evidence

- Route completion: True.
- Emergency-braking entries: 1.
- Merge-slowdown entries: 1.
- Simultaneous nonzero throttle and brake samples:
  0.
- Runtime process deaths before route completion:
  0.
- Runtime path rejection/fallbacks before route completion:
  0.
- Expected terminal-point fallbacks after route completion:
  39.
- Stop requests were recorded at route waypoints
  25, 32, 39, 59.

The post-completion target-pose fallback warnings are expected when the local
route contains only its terminal point; the decision and controller layers are
already latched in the route-complete safe stop. The Unity process was killed
only during user-requested teardown after all ROS nodes had exited cleanly.

## Generated figures

- `route_trace.svg`: driven route.
- `speed_profile.svg`: measured, planning, and final decision speeds.
- `tracking_error.svg`: nearest-reference path tracking error.
- `traffic_light_state.svg`: traffic-light observations.
- `obstacle_distance.svg`: front and merge obstacle distances.
- `command_outputs.svg`: throttle, brake, and steering commands.

`metrics.json` is the machine-readable record. Regenerate these artifacts with:

```bash
source install/setup.bash
python3 tools/analyze_mission_run.py \
  --bag /path/to/bag \
  --logs /path/to/ros_logs \
  --output mission_results
```
