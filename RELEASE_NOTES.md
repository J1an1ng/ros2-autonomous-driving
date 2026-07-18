# Release notes

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
