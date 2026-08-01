# Release notes

## full-system-v1.9.0

Code-quality, documentation, and full closed-loop validation release built on
`full-system-v1.8.0`.

Improvements:

- Extends `trajectory_generator` from geometric path smoothing to deterministic
  kinematic/dynamic time parameterization. Curvature limits lateral acceleration; a
  forward pass limits acceleration; a backward pass limits deceleration and anticipates
  curves or terminal stops; trapezoidal integration assigns monotonic point times.
- Links path curvature to the production bicycle model with
  `min(max_curvature, tan(maximum_steering_angle) / wheelbase)`, and preserves measured
  initial speed as a hard boundary. An insufficient braking horizon is reported as
  infeasible and produces a safe stop instead of a falsely feasible profile.
- Adds Team 20's `autonomy_interfaces` package with `TrajectoryPoint` and
  `PlannedTrajectory` messages. `/planning/trajectory` exposes pose, signed curvature,
  feasible speed, and time from start. The controller now consumes this message directly,
  uses the speed attached to its Pure Pursuit lookahead point, and caps it with the final
  decision-layer speed.
- Adds focused trajectory constraint tests, a rosbag mission-analysis tool, a ROS graph
  with contribution ownership, and persisted trajectory/mission plots and metrics.
- Adds `tools/run_full_mission.py` as the standard moving-run entry point. It records
  each run, waits for route completion, audits the bag, and atomically regenerates
  `mission_results`; failed runs preserve the previous valid result.
- Injects one vehicle-model parameter pair into planner and controller, makes the
  controller independently reject over-curvature trajectories, and adds a ROS launch
  regression for valid, malformed, empty, over-curvature, and stale trajectory inputs.
- Tightens the rosbag acceptance gate so route completion cannot pass without nonempty
  trajectory evidence. It now records runtime empty-message fraction and continuous
  unavailability, classifies fallback reasons, and exits nonzero on route/trajectory
  audit failure.
- Resolves the `route_manager` flake8 and PEP 257 findings, simplifies the
  route-manager node structure, reformats the waypoint recorder, strengthens
  route-map validation, and adds regression coverage for malformed route data.
- Introduces a shared repository style through `.editorconfig`,
  `project/CODE_STYLE.md`, and `project/tools/check_code_quality.sh`. The gate
  checks Team 20 Python and C++ plus the maintained simulator bridge while
  preserving documented simulator exemptions.
- Applies consistent naming, imports, formatting, and documentation across
  perception, route management, trajectory generation, decision making,
  controller, simulation launch code, system bringup, tests, and project tools.
  Cleans package metadata in the affected Team 20 manifests and setup files
  without changing their ROS interfaces.
- Keeps the project-supplied `dummy_controller` at its v1.8.0 baseline and out
  of Team 20 style ownership. Editor rules also protect vendored third-party
  sources from automatic formatting.
- Fixes Event1 adaptive following so a brief vehicle-detection dropout retains
  the last safe target exactly instead of recomputing it from a slowing ego
  speed. A stable lead with surplus clearance may now recover gradually, which
  prevents the previous stop-and-restart behaviour without weakening the
  independent emergency-braking thresholds.
- Expands the root and workspace READMEs with exact team contributions,
  supported functions, dependency setup, release-branch clone/pull commands,
  build and test instructions, safe startup, full-route launch, live
  validation, and troubleshooting.

Validation:

The values below record one release-acceptance run. They are traceable evidence for this
version, not fixed timing, distance, or message-count guarantees for every simulation.

- Passed the repository code-quality gate across 57 Python files and the
  configured C++ sources with no findings.
- Built all nine ROS 2 workspace packages successfully.
- Passed the complete workspace suite: 216 tests, 0 errors, 0 failures, and
  one expected skip.
- Completed a production Unity run on 2026-08-01 from waypoint 0 through 63
  and logged `Route completed at waypoint 63`.
- Recorded a mission time of 211.065 seconds from the first forward control command to
  `route_complete` (210.925 seconds from first measured motion). Audited 68,355 points
  in 2,123 `/planning/trajectory` messages: maximum curvature was 0.215946 1/m against
  the 0.217704 1/m vehicle limit; maxima were 3.0000 m/s² lateral, 2.0003 m/s²
  longitudinal acceleration, and 3.0000 m/s² deceleration, with zero non-monotonic times
  and zero invalid points. Before completion, 2,098 of 2,112 trajectory messages were
  nonempty; the 0.66% empty fraction and 0.902 s longest empty interval passed the
  documented 5% / 10 s availability gate. Maximum message gap was 0.162 s against
  the controller-timeout-derived 1.05 s limit.
- Classified 105 pre-completion local-path rejections: 104 curvature-limit rejections
  and one path with fewer than two points.
  The planner used its straight target-pose degradation, reapplied the complete
  feasibility gate, and otherwise would have published an empty trajectory for safe stop.
- Observed adaptive following for both vehicle events. During Event1, measured speed
  remained at or above 2.536 m/s and the decision target remained at or above 2.949 m/s,
  with no near-zero samples; the previous run reached 0.000 m/s. Event2 reached a
  measured minimum distance of 7.535 m and did not need
  emergency braking or the merge-slowdown state. No collision occurred. Traffic-light
  stop/release behavior was recorded at waypoints 25, 32, and 59; waypoint 52 did not
  require a stop in this signal cycle.
- Confirmed that production perception used RGB, depth, TF, and YOLO rather
  than the disabled semantic camera. The bag contains 1,459 UInt8 traffic-light
  messages (1,170 unknown, 196 red, 7 yellow, and 86 green), and `/car_command` had one
  production publisher.
- Found no autonomy-node runtime `ERROR`, `FATAL`, process death, segmentation fault,
  traceback, collision, or route departure before route completion. RViz emitted one
  visualization-only GLSL shader error. After completion, the runner initiated teardown;
  all 30 logged process-death entries were classified as teardown-only, and Unity was
  terminated after the launch shutdown grace period.

## full-system-v1.8.0

Event1 detection, speed-profile, and braking update built on
`full-system-v1.7.0`.

Improvements:

- Extends route-gated YOLO/RGB-D direct vehicle control to Event1 waypoints
  17--20. A 1.3 s association/dropout hold and a 2 m release margin prevent
  brief detector gaps or activation-boundary chatter from returning the
  vehicle to 14 m/s cruise.
- Adds a smooth Event1 following profile with a 2.0 m/s² downward target slew
  while retaining the independent near-field emergency brake.
- Removes unnecessary semantic intersection caps from the trajectory layer
  and restores the surrounding road speed at straight-through intersections.
- Raises traffic-light waypoints 25, 32, 52, and 59 to an 8 m/s far-field
  ceiling. The decision layer now reduces speed continuously with a
  distance-based `sqrt(2*a*d)` envelope instead of applying an early fixed
  3 m/s step.
- Calibrates stop boundaries to 2.0 m at waypoint 25, 3.4 m at waypoint 52,
  and 4.0 m at waypoints 32 and 59. Waypoint 25 uses a local 1.5 m route reach
  radius so its 2 m stop reference cannot switch prematurely to waypoint 26.
- Changes the longitudinal controller to derivative-on-measurement, avoiding
  a one-cycle braking kick when target speed changes, and raises the validated
  throttle ceiling to 0.70.
- Adds focused green-lamp ROI regressions and expands decision, route,
  controller, trajectory, and Event1 fusion tests.

Validation:

- Rebuilt the affected perception, controller, decision-making, trajectory,
  and route-manager packages successfully.
- Passed all 165 selected release tests: 4 focused perception ROI tests,
  11 controller tests, 98 decision-making tests, 37 trajectory-generator
  tests, and 15 route-manager functional tests. Three repository-wide
  route-manager linter checks were intentionally excluded.
- The 2026-07-27 production baseline completed waypoint 0--63 with both NPC
  events. A new full closed-loop Unity run remains required for the v1.8.0
  Event1 and stop-line calibration changes.

## full-system-v1.7.0

YOLO RGB-D perception release built on `full-system-v1.6.0`.

Improvements:

- Replaces the Unity semantic vehicle and traffic-light detectors with a
  normal-RGB YOLOv8n ONNX pipeline. The production simulator configuration
  disables the segmentation camera.
- Runs inference in a ROS 2 C++ node through Microsoft ONNX Runtime 1.22.0.
  Vehicle boxes are combined with depth-camera 3-D reconstruction, TF, and
  RGB reprojection for robust front-vehicle ranging.
- Adds route-aware traffic-light selection for waypoints 25, 32, 52, and 59,
  including per-intersection perspective profiles that reject opposing and
  cross-traffic signals.
- Handles very small traffic-light boxes that omit the lower green bulb while
  rejecting large green road signs below the signal. Non-green evidence
  immediately revokes permission, while an unambiguous green can release
  without unnecessary delay.
- Refines Event 2 following and recovery so the ego vehicle can maintain a
  safe gap or resume smoothly after the lead vehicle moves, without requiring
  aggressive acceleration.
- Removes obsolete semantic-camera nodes, one-off YOLO diagnostics, and the
  perception unit-test sources from the compact submission. Runtime debug
  topics and model/license documentation remain available.

Validation:

- Built the final production `perception` package successfully after cleanup;
  it contains the `yolo_rgbd_detector_node` and `occupancy_map_node` targets.
- Before removing the perception test sources, all 24 YOLO post-processing,
  model-runtime, route-gate, and traffic-light stability tests passed with no
  errors, failures, or skips.
- Completed the waypoint 0--63 Unity route with both NPC events enabled and
  reached `route_complete`. The route passed the controlled signals after the
  waypoint 25, 52, and 59 target-selection fixes.
- The bundled fixed-size 640 x 640 COCO YOLOv8n model has SHA-256
  `8707bb120b03a6bf8a199e7c35c6d47ad5f83de4fdd233110db270f7d355a3be`.

## full-system-v1.6.0

Safety backport release built on `full-system-v1.5.0`.

Improvements:

- Retains an active merge-obstacle latch across short point-cloud gaps. If the
  observation is more than 3 seconds old, the decision layer now enters
  `merge_obstacle_stale_stop` instead of incorrectly returning to cruise; only
  a fresh safe observation from the actual merge source clears the latch.
  Front/semantic-only updates use independent freshness and cannot synthesize
  merge clearance.
- Restores the closed-loop-validated corner geometry and conservative speed
  limits at waypoints 33--35. Waypoints 39--42 retain the faster v1.5
  event-intersection geometry and speed profile. Regression tests protect both
  route profiles and route completeness.
- Lets a confirmed green immediately cancel an existing traffic-light brake
  latch. Unambiguous green is confirmed in two consecutive frames, while
  mixed red/green evidence still requires four consecutive frames.
- Preserves all v1.5 semantic-vehicle fusion, adaptive-following, and
  waypoint 0--63 route-completion behavior.

Validation:

- Built all eight ROS 2 workspace packages successfully.
- Passed all 70 decision-making tests, all four traffic-light stability-filter
  tests, all 10 controller tests, all 34 trajectory tests, and all nine
  route-manager functional tests. The route-manager flake8 check remains at
  its unchanged baseline of 260 existing style issues.
- Completed the waypoint 0--63 Unity run with both NPC events enabled in
  370.258 recorded seconds. The restored waypoint 33--35 turn and unchanged
  v1.5 waypoint 39--42 turn completed without intervention. Relative to the
  rejected 2 m/s waypoint 39--42 profile, the waypoint 39-to-43 traversal
  improved from 9.505 to 6.805 seconds and measured mean speed improved from
  1.867 to 2.859 m/s.
- Stopped and released at all four controlled traffic lights, with stop
  requests issued approximately 3.63, 3.82, 3.82, and 3.51 m from waypoints
  25, 32, 52, and 59 respectively.
- Entered adaptive following in the NPC-event region with no emergency-brake
  entry, no simultaneous throttle/brake output, no runtime process death, and
  no path fallback before route completion.

## full-system-v1.5.0

Semantic-vehicle fusion and adaptive-following release packaged directly from
the validated `full-system-v1.2.0` working directory.

Improvements:

- Adds a segmentation-based vehicle detector for red semantic components,
  paired with aligned depth and explicit yellow traffic-light exclusion.
- Fuses fresh semantic detections with the path-corridor point cloud across
  the whole route. Event 2 retains a bounded waypoint 43--48 direct-distance
  fallback for the crossing and merging geometry.
- Rejects stale, laterally implausible, and discontinuous detections, and
  suppresses the known fixed structure near waypoint 48 without weakening the
  short-range raw safety guard.
- Adds adaptive following based on a 6 m standstill gap, a 2.2 s time gap, and
  relative range rate. Braking now starts earlier and releases progressively
  when the lead vehicle moves instead of forcing an unnecessary full stop.
- Restricts merge behavior to its intended waypoint window while keeping
  ordinary front-vehicle handling active over the full route.
- Removes one-off Event 2 configuration selectors, monitor scripts, backup
  files, and dedicated temporary diagnostic topics. Consolidated runtime
  diagnostics remain available on `/decision/debug`.
- Updates the project status and integration documentation with the validated
  production configuration and test procedure.

Validation:

- Built all eight ROS 2 workspace packages successfully.
- Passed all 65 decision-making, 10 controller, and 34 trajectory functional
  tests.
- Passed the route-manager functional tests; its existing repository-wide
  flake8 debt remains unchanged.
- Completed the waypoint 0--63 no-event run without false following,
  emergency, or merge behavior.
- Completed the waypoint 0--63 all-events run with adaptive following through
  waypoints 43--50, a minimum fused front distance of approximately 9.92 m,
  no collision, and normal route completion.

## full-system-v1.3.0

Route and performance update built on `full-system-v1.2.0`.

Improvements:

- Integrates the latest recorded route geometry and extends the mission from
  waypoint 59 through waypoint 63, including matching segment and zone
  metadata.
- Retains the validated non-zero planning speed at dynamic traffic-light stop
  waypoints 25, 32, 52, and 59 so a stable green can release the vehicle.
- Aligns the trajectory-generator and controller built-in defaults with the
  configured 14 m/s (50.4 km/h) straight-route ceiling while preserving lower
  curve, intersection, traffic-light, and NPC-event limits.
- Raises the maximum throttle command from 0.35 to 0.50 so the vehicle reaches
  the intended route and event pace sooner without changing the maximum brake
  command or safety-stop behavior.
- Updates route, trajectory, and controller documentation for the new route
  and runtime limits.

Validation:

- Validated all 64 waypoint IDs, required fields, segment bounds, and zone
  references in the release route map.
- Built all eight ROS 2 workspace packages successfully.
- Passed the route, trajectory, decision, and controller functional tests.
- Confirmed the launched trajectory generator loads `max_speed=14.0` and the
  launched controller loads `maximum_target_speed=14.0` and
  `maximum_throttle=0.50`.
- Full closed-loop Unity validation of the new waypoint 60--63 extension
  remains required; earlier benchmark evidence covers the prior waypoint
  0--59 route.

## full-system-v1.2.0

Traffic-light safety release built on `full-system-v1.1.0`.

Improvements:

- Rejects vertically clipped secondary traffic lights so an unrelated
  back-facing red signal cannot veto the forward green signal.
- Revokes green authorization immediately on red, yellow, or unknown
  perception instead of retaining a stale stable green.
- Publishes paired red/green evidence scores on
  `/perception/traffic_light_scores` for decision-layer validation.
- Applies a waypoint-25-specific green release gate: green must clearly exceed
  red and residual red evidence must remain at or below the validated score of
  4. Other controlled lights retain their previously validated thresholds.
- Adds decision debug output for the current red/green evidence scores.

Validation:

- Built the `perception` and `decision_making` packages successfully.
- Passed all 35 related tests, including regressions for the observed 36/40
  and 44/60 false-green evidence and the verified 0/40 true-green release.
- Confirmed normal full-system startup after removing duplicate ROS graph
  publishers from earlier diagnostic sessions.

## full-system-v1.1.0

Validated full-system release built on `full-system-mvp-v1.0.0`.

Improvements:

- Raises the controlled-intersection approach limit to 3 m/s while retaining
  the validated 4 m stop boundary.
- Groups the high and low traffic lights belonging to the same approach, lets
  the lower light take over when the higher light leaves view, and filters
  smaller adjacent-junction signals.
- Applies conservative group aggregation: any red or yellow prevents release,
  and all currently selected lights must be green before release.
- Raises the straight-route controller and trajectory ceilings to 14 m/s and
  keeps intersection/turn limits in the route map and decision layer.
- Enables both Unity NPC events in the default full simulation.
- Adds live RViz views for the 2D obstacle map, planned paths, driven trace,
  speed, throttle, steering-wheel command, and heading.

Validation:

- Built the perception, simulation, system bringup, decision, controller, and
  trajectory packages successfully.
- Passed the selected controller, decision, and trajectory test suites.
- Completed two consecutive simulations from waypoint 0 to waypoint 59 with
  both NPC events enabled; the route entered `route_complete` both times.
- Verified stop-and-release behavior at the controlled traffic-light
  intersections, including the high/low dual-light view transition.

## full-system-mvp-v1.0.0

Previous minimal full-system baseline at commit `9cbaab4`. This is the rollback
point from immediately before the v1.1.0 speed, signal-group, NPC, and live
visualization changes.

To inspect or run the previous baseline without moving the current branch:

```bash
git switch --detach full-system-mvp-v1.0.0
```

To return to the development branch:

```bash
git switch temp_inte
```

To inspect the new release tag directly:

```bash
git switch --detach full-system-v1.1.0
```
