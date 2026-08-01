# Autonomy Interfaces

This Team 20 package defines the custom ROS 2 message used by the trajectory planner.

`autonomy_interfaces/msg/PlannedTrajectory` contains an ordered list of
`TrajectoryPoint` messages. Each point carries:

- a pose in the frame named by the trajectory header;
- signed curvature in `1/m`;
- dynamically feasible target speed in `m/s`; and
- monotonic `time_from_start`.

The trajectory generator publishes this message on `/planning/trajectory`. The existing
`/planning/path` and `/planning/target_speed` topics remain available for visualization
and compatibility with the decision and controller packages.
