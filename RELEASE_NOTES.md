# Release notes

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
  remains required; the existing Stage 6 benchmark evidence covers the prior
  waypoint 0--59 route.

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
