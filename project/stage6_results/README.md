# Stage 6 benchmark results

These artifacts summarize three consecutive full Unity missions made on 2026-07-17 from
Stage 5 baseline commit `bfc5c00`. Every run used spawn0, route index0, both NPC events, all
production nodes, and completed waypoint59 without observed collision, road departure, or
manual intervention.

## Files

- `summary.csv`: compact per-run acceptance metrics.
- `metrics.json`: metrics, safe-stop reasons, and traffic-stop events in machine-readable
  form.
- `route_trace.svg`: the three driven routes.
- `tracking_error.svg`: nearest planning-path tracking error over time.
- `speed_profile.svg`: measured, planning, and decision speed profiles.
- `traffic_light_state.svg`: perceived traffic-light states.
- `obstacle_distance.svg`: front and merge obstacle distances.
- `command_outputs.svg`: throttle, steering, and brake commands.

The source bags are intentionally not committed. They are stored at
`/tmp/team20_stage6_evidence/run{1,2,3}/bag` with matching launch/node logs. Regenerate all
artifacts with:

```bash
source /opt/ros/jazzy/setup.bash
source <workspace-install>/setup.bash
python3 project/tools/analyze_stage6_bags.py \
  --evidence-root /tmp/team20_stage6_evidence \
  --output project/stage6_results
```

The analyzer requires the Jazzy `rosbag2_py` package. Install `ros-jazzy-rosbag2` when it is
not already available.

## Interpretation

Mission times were 648.845, 651.614, and 651.287 seconds. Tracking RMSE was 0.7440--0.7775 m,
maximum measured speed was 3.162 m/s, and Event1/Event2 right-front minima were respectively
2.377--2.438 m and 2.394--2.444 m. No run recorded simultaneous throttle/brake, a runtime
node death, or a path rejection/fallback.

Event2 did not create a finite front-path obstacle or emergency-braking transition in any
run. It was handled as the merge-side behavior Unity actually generated; this evidence must
not be interpreted as validation of the assignment's intended NPC brake-in-lane maneuver.
See `project/INTEGRATION_README.md` for the full acceptance record and limitations.
