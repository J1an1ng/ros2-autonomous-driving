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
Checks the distance between the vehicle and the current waypoint.
Switches to the next waypoint when the vehicle is close enough.
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
A local navigation path containing up to five consecutive waypoints starting from the current route index.
For example, if the current route index is 10, the local path normally contains:

10
11
12
13
14

Near the end of the route, fewer than five waypoints may be published.

This topic provides the trajectory planner with information about the road geometry ahead, especially at curves and intersections.

/route/speed_limit

Type: std_msgs/msg/Float32

Description:The recommended speed limit for the current target waypoint, in meters per second.

/route/current_waypoint_id

Type: std_msgs/msg/Int32

Description: The id of the currently selected target waypoint.

Note: This id represents the current target waypoint, not necessarily the last reached waypoint.

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
  -p lookahead_index:=0
  
Check Published Topics
ros2 topic echo /route/target_pose
ros2 topic echo /route/speed_limit
ros2 topic echo /route/current_waypoint_id
ros2 topic echo /route/route_state
ros2 topic echo /route/local_path
