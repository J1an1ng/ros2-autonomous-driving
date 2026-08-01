# Autonomy Interfaces

This Team 20 package defines the custom ROS 2 message used by the trajectory planner.

`autonomy_interfaces/msg/PlannedTrajectory` contains an ordered list of
`TrajectoryPoint` messages. Each point carries:

- a pose in the frame named by the trajectory header;
- signed curvature in `1/m`;
- dynamically feasible target speed in `m/s`; and
- monotonic `time_from_start`.

The trajectory generator publishes this message on `/planning/trajectory`, and the
production controller consumes it directly for both Pure Pursuit geometry and the speed
stored at the selected lookahead point. `/planning/path` remains available to the
obstacle corridor and RViz; `/planning/target_speed` remains the planning input to the
decision state machine.
