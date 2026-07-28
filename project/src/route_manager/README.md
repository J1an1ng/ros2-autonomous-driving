# Route Manager

This package provides waypoint recording and route target selection for the autonomous driving project.

The route manager uses a predefined route map stored in `config/route_map.yaml`. The route map contains waypoints, waypoint types, speed limits. During runtime, the route manager reads the current vehicle pose from TF and publishes the next target waypoint for the trajectory planning module.

## Nodes

### 1. `waypoint_recorder`

The `waypoint_recorder` node is an interactive tool for recording waypoints from the current vehicle pose.

It reads the TF transform: world -> OurCar/INS

and stores the current vehicle position as a waypoint when the user enters a command in the terminal.

Supported waypoint types include:
n      normal
ca     curve_approach
ce     curve_entry
c      curve_mid
cx     curve_exit
ia     intersection_approach
s      stop_zone
i      intersection
ix     intersection_exit
slow   slow_zone
The recorded waypoints are saved into: config/route_map.yaml
2. route_manager_node

The route_manager_node is the main runtime node of this package.

It performs the following tasks:

Loads the predefined route from route_map.yaml.
Reads the current vehicle pose from TF.
Checks the distance and forward progress relative to the current waypoint.
Switches to the next waypoint when the vehicle reaches it or crosses its
forward progress plane, so a slightly missed waypoint cannot stall the route.
Publishes the selected waypoint as the current target pose.
/route/target_pose

Output Topics
/route/target_pose

Type: geometry_msgs/msg/PoseStamped

Description:The currently selected target waypoint. This topic can be used by the trajectory planner as the next short-term goal.

/route/local_path

Type:

nav_msgs/msg/Path

Description:
A local navigation path containing up to five consecutive waypoints. It starts
at the previous waypoint when available, allowing the trajectory generator to
project the ego pose onto the active route segment. For example, if the current
route index is 10, the local path normally contains:

9
10
11
12
13

Near the end of the route, fewer than five waypoints may be published.

This topic provides the trajectory planner with information about the road geometry ahead, especially at curves and intersections.

/route/speed_limit

Type: std_msgs/msg/Float32

Description:The recommended speed limit for the current target waypoint, in meters per second.

/route/current_waypoint_id

Type: std_msgs/msg/Int32

Description: The id of the currently selected target waypoint.

Note: This id represents the current target waypoint, not necessarily the last reached waypoint.

`/route/current_waypoint_distance` (`std_msgs/msg/Float32`) publishes the
planar ego-to-target distance in metres. The decision layer uses this existing
route geometry to approach an unclassified traffic light slowly while retaining
a hard stop boundary.

/route/route_state

Type: std_msgs/msg/String

Description: The semantic type of the current target waypoint.

Possible values include:
normal
curve_approach
curve_entry
curve_mid
curve_exit
intersection_approach
stop_zone
intersection
intersection_exit
route_complete

At `route_complete`, the speed limit is published as zero and the final
waypoint remains the target.

The initial stop waypoint (ID 0) has a zero route speed. Dynamic traffic-light
stop zones 25, 32, and 52 use a conservative 3 m/s route approach speed.
Waypoint 59 uses an 8 m/s far-field ceiling so the decision node's
distance-based braking envelope can reduce speed smoothly toward its stop
boundary instead of applying a 44 m early speed step. The decision node owns
the red/yellow stop at all four lights. Keeping a non-zero planning speed at
those waypoints also lets the vehicle leave after a stable green release and
prevents a route-progress deadlock.
The waypoint32 T-junction uses the closed-loop-validated waypoint33--35
geometry with a 3 m/s corner limit. The event-intersection turn retains the
faster v1.5 waypoint39--42 geometry and speed profile (3, 8, 8, and 14 m/s),
and the road-end turn uses the updated waypoint47--50 samples to keep the local
path aligned with the recorded route through all bends.

Waypoints60--63 extend the route beyond the final traffic-light stop at
waypoint59 and complete the turn back toward the starting area. The route map
also includes matching segment and zone metadata for all 64 waypoints.

Runtime parameters include `initial_waypoint_index` (default `0`). This is a
diagnostic aid for local segment tests with a matching Unity spawn; benchmark
runs must retain index 0 and spawn 0.

Run Waypoint Recorder

Before running the recorder, make sure the simulation bridge is running and the TF transform world -> OurCar/INS is available.

ros2 run route_manager waypoint_recorder \
  --ros-args \
  -p map_frame:=world \
  -p vehicle_frame:=OurCar/INS \
  -p output_file:=/home/linux/ros_course/introduction-to-ros-2026/project/src/route_manager/config/route_map.yaml

Useful commands inside the recorder:

n      record normal waypoint
c      record curve_mid waypoint
s      record stop_zone waypoint
i      record intersection waypoint
undo   remove the last recorded waypoint
list   show recent waypoints
q      save and quit

Run Route Manager

Before running the route manager, make sure the simulation bridge is running and the TF transform world -> OurCar/INS is available.

ros2 run route_manager route_manager_node \
  --ros-args \
  -p route_file:=/home/linux/ros_course/introduction-to-ros-2026/project/src/route_manager/config/route_map.yaml \
  -p map_frame:=world \
  -p vehicle_frame:=OurCar/INS \
  -p waypoint_reach_distance:=3.0 \
  -p lookahead_index:=0 \
  -p initial_waypoint_index:=0
  
Check Published Topics
ros2 topic echo /route/target_pose
ros2 topic echo /route/speed_limit
ros2 topic echo /route/current_waypoint_id
ros2 topic echo /route/route_state
ros2 topic echo /route/local_path
