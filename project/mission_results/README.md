# Full-System Trajectory Validation

Validation date: 2026-08-01

The production stack was launched from waypoint 0 with motion enabled and the
Unity semantic camera disabled. The vehicle reached waypoint 63 and published `route_complete`.

## Mission result

- Mission time: **210.231 s**.
- Timing definition: first forward /car_command (throttle > 0.01, brake < 0.01) to first route_complete.
- Cross-check from first measured vehicle motion (> 0.1 m/s):
  **210.051 s**.
- The command and twist topics are recorded independently, so the first
  measured motion sample may precede the first recorded forward-command sample
  by one cycle.
- Maximum measured speed: 5.865 m/s.
- Path-tracking RMSE / P95: 0.6676 m /
  1.6717 m.

## Trajectory constraint audit

The audit checked 70008 points in
2115 `/planning/trajectory` messages.
Effective limits were loaded from `src/trajectory_generator/config/trajectory_generator.yaml`. The
kinematic cap is `min(max_curvature, tan(maximum_steering_angle) / wheelbase)`.

| Constraint | Configured limit | Maximum observed | Result |
| --- | ---: | ---: | --- |
| Bicycle-model curvature | 0.217704 1/m | 0.217622 1/m | Pass |
| Lateral acceleration `v^2 * abs(curvature)` | 3.00 m/s² | 3.0000 m/s² | Pass |
| Longitudinal acceleration | 2.00 m/s² | 2.0008 m/s² | Pass |
| Longitudinal deceleration | 3.00 m/s² | 3.0000 m/s² | Pass |
| Strictly increasing point time | required | 0 violations | Pass |
| Finite numeric trajectory values | required | 0 invalid points | Pass |
| Nonempty runtime trajectory evidence | required | 2096 messages | Pass |
| Empty runtime trajectory fraction | ≤ 5.0% | 0.33% | Pass |
| Longest continuous empty-trajectory interval | ≤ 10.0 s | 0.596 s | Pass |
| Maximum trajectory-message gap | ≤ 1.05 s | 0.192 s | Pass |

The spatial acceleration audit uses adjacent published poses and speeds. A
0.05 m/s² acceleration tolerance and 0.0001 1/m curvature tolerance cover
float32 serialization and the difference between curved arc length and
pose-to-pose chord length. The availability gate allows at most 5% empty
runtime publications and no continuous empty interval longer than 10 s. An
empty publication makes the controller brake, so these bounds allow short
fail-safe recovery while rejecting persistent planner loss. Topic silence is
checked separately against the controller's configured trajectory timeout plus
a 0.05 s timestamp tolerance, from the first forward command through route
completion. Overall trajectory audit:
**Pass**.

## Scenario evidence

- Route completion: True.
- Adaptive-following entries: 4.
- Emergency-braking entries: 0.
- Merge-slowdown entries: 0.
- Traffic-light messages: 1386 as `std_msgs/msg/UInt8`;
  state counts: {'unknown': 1085, 'red': 194, 'yellow': 7, 'green': 100}.
- Simultaneous nonzero throttle and brake samples:
  0.
- Runtime process deaths before route completion:
  0.
- Runtime path rejection/fallbacks before route completion:
  102.
- Runtime fallback reasons:
  {'path exceeds the curvature limit': 102}.
- Expected terminal-point fallbacks after route completion:
  0.
- Stop requests were recorded at route waypoints
  25, 32, 52, 59.

Every runtime rejection in this run was caused by the bicycle-model curvature
gate. The planner then generated its straight target-pose fallback and applied
the same curvature, acceleration, time, and finite-value validation before
publication; rejected fallbacks produce an empty trajectory and controller safe
stop. Thus the count records conservative degradation, not unchecked output.

No target-pose fallback was recorded after route completion. The standard
runner initiated teardown only after route completion. Its log audit classified
20 process-death entries as teardown-only and 0 as occurring before completion.

## Generated figures

- `route_trace.svg`: driven route.
- `speed_profile.svg`: measured, planning, and final decision speeds.
- `tracking_error.svg`: nearest-reference path tracking error.
- `traffic_light_state.svg`: traffic-light observations.
- `obstacle_distance.svg`: front and merge obstacle distances.
- `command_outputs.svg`: throttle, brake, and steering commands.

`metrics.json` is the machine-readable result. The standard mission runner also
writes `validation_provenance.json` with the launch command, Git state,
renderer, and raw evidence location, plus `ros_logs.sha256` for the run logs.
This run's raw bag and logs are retained in
[`../Rosbag_reproduction/main-20260801-231823/`](../Rosbag_reproduction/README.md). The MCAP
SHA-256 is
`fdb33ded0eb9acdf3e44456431f34b26608f09bc922732658b72ed68c346d89f`.
Regenerate derived artifacts from that retained evidence with:

```bash
source install/setup.bash
python3 tools/analyze_mission_run.py \
  --bag Rosbag_reproduction/main-20260801-231823/bag \
  --logs Rosbag_reproduction/main-20260801-231823/ros_logs \
  --trajectory-config \
    src/trajectory_generator/config/trajectory_generator.yaml \
  --controller-config src/controller/config/controller.yaml \
  --output /tmp/team20-mission-results \
  --validation-date 2026-08-01
```
