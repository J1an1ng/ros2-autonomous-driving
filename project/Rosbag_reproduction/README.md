# Rosbag reproduction

The latest complete `main` validation run is retained under
`main-20260801-231823/`. Its raw MCAP bag and ROS logs produced the metrics and figures
in `../mission_results/`.

## Evidence identity

- Source commit: `40b1ecc6150752e022f67385f9812de72badbbb8`
- Launch time: 2026-08-01 23:18:23 CEST
- Launch command: `python3 tools/run_full_mission.py --visualization`
- Bag: `main-20260801-231823/bag/bag_0.mcap`
- Bag size: 6,588,762 bytes
- Bag SHA-256: `fdb33ded0eb9acdf3e44456431f34b26608f09bc922732658b72ed68c346d89f`
- Metadata SHA-256: `e73f64581ae878c0944d027fc6d9015793657ac0cca84904c26f86604dd1527f`
- Recorded messages: 48,416 across the 15 mission topics
- Recorded duration: 217.133 s
- Workspace tests: 216 tests, 0 errors, 0 failures, 1 skipped

`mission_results/validation_provenance.json` records `working_tree_dirty: true` because
documentation cleanup and deletion of unused auxiliary tools were not committed when the
run started. No production package source, configuration, launch file, or installed
runtime artifact differed from the source commit above.

The `.mcap` file is stored with Git LFS. After cloning, retrieve and verify it with:

```bash
git lfs pull
sha256sum project/Rosbag_reproduction/main-20260801-231823/bag/bag_0.mcap
```

## Reproduce the report

From `project/`, build and source the workspace, then analyze the retained bag and logs:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 tools/analyze_mission_run.py \
  --bag Rosbag_reproduction/main-20260801-231823/bag \
  --logs Rosbag_reproduction/main-20260801-231823/ros_logs \
  --trajectory-config src/trajectory_generator/config/trajectory_generator.yaml \
  --controller-config src/controller/config/controller.yaml \
  --output /tmp/team20-mission-results \
  --validation-date 2026-08-01
```

Compare `/tmp/team20-mission-results/metrics.json` and the generated SVG figures with
`mission_results/`. The report is deterministic for the retained bag and configuration.
