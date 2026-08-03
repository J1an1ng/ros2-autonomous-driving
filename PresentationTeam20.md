# Team 20 Autonomous Driving — Five-Minute Presentation Script

**Target duration:** approximately 4 minutes 45 seconds to 5 minutes at a clear pace.
**Format:** four-speaker version. It can also be read by one presenter by omitting the
speaker names and transition sentences.

## Slide 1 — Goal and system overview (Lei, 0:00–0:50)

Hello, we are Team 20: Lei Fang, Yida Lyu, Tao Zhang, and Jianing Huang.

Our system completes the supplied urban course from waypoint zero to sixty-three, responds
to four traffic-light areas, and adapts to other vehicles. It runs on ROS 2 Jazzy and Ubuntu
24.04 and starts from one launch file. Motion is disabled by default, and the production
controller is the only publisher of the car command.

Unity provides RGB, depth, vehicle state, and TF. Perception produces traffic-light,
vehicle-distance, point-cloud, and occupancy information. Route and trajectory modules
determine where and how fast to move. Decision making applies traffic rules and safety
priorities, and the controller produces steering, throttle, and brake commands.

## Slide 2 — Perception, route, and trajectory (Tao and Jianing, 0:50–2:25)

**Tao:** We deliberately do not use the semantic camera. YOLOv8n runs on normal RGB, while
aligned depth and camera calibration provide metric vehicle distance. Depth pixels are
back-projected with the camera intrinsics. Filtered depth also produces point clouds, a
projected occupancy map, and a three-dimensional OctoMap.

Because traffic lights are small, we combine route-aware regions, colour evidence, and
temporal filtering. This reduces false state changes and associates each observation with
the correct intersection.

**Jianing:** Lei's route manager validates a fixed sixty-four-waypoint YAML route. It
projects the ego pose onto the route and publishes progress, a local path, a target pose,
intersection context, and the speed limit.

The trajectory generator smooths and resamples the local route, then applies steering,
lateral-acceleration, acceleration, and braking constraints. Each point contains pose,
signed curvature, target speed, and relative time in our own `TrajectoryPoint` and
`PlannedTrajectory` ROS messages.

## Slide 3 — Decision making, control, and safety (Yida, 2:25–3:25)

Decision making uses a priority-based state machine. It combines traffic lights, route
context, planning speed, vehicle distance, merge risk, and stale-input checks. Emergency
stopping has highest priority; otherwise the car can stop for red, follow a slower vehicle,
handle a merge, or continue at the planned speed.

Pure Pursuit calculates steering and PID regulates speed. The controller checks message
age, trajectory geometry, curvature, and command limits. Missing, stale, or infeasible input
causes braking. Planning proposes a speed, decision making publishes the final permitted
speed and stop request, and only the controller publishes `/car_command`.

## Slide 4 — Results (Jianing, 3:25–4:10)

In our retained full-system run, the car reached waypoint sixty-three in 210.231 seconds.
Reference-path tracking error was 0.668 metres RMSE, with a 1.672 metre ninety-fifth
percentile.

The rosbag contains more than 70,000 trajectory points. Our audit confirmed finite values,
monotonic timing, availability, and all configured dynamic limits. Stop requests occurred at
all four controlled lights, and adaptive following occurred four times. All nine packages
build, with 216 tests and no failures or errors.

## Slide 5 — Limitations, issues, and feedback (Lei, 4:10–5:00)

The main limitation is the fixed global route. The system can stop for an obstacle but does
not plan a new free-space route around it. Traffic-light recognition depends on route gates
and calibrated image regions, and the YOLO model is a COCO baseline rather than a Unity-
specific model. The trajectory model does not include jerk, tyre forces, steering rate, or
full drivetrain dynamics. The project is therefore validated only in the supplied simulator.

The hardest issue was making asynchronous perception, planning, and control agree on timing,
frames, and safety semantics. Clear topic ownership, custom messages, watchdogs, rosbag
recording, and automated audits made this manageable.

As feedback, the project was valuable because it required real integration. The simulator
and configurable events were especially useful. A standard evaluation script and clearer
sensor-timestamp and benchmark definitions would make debugging and comparison easier.

Thank you. The detailed architecture, contribution ownership, plots, limitations, and
bibliography are available in our standalone documentation.
